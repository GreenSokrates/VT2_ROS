/*
 * CoE.cpp
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 14.07.2017
 *      Author: Marcel Honegger
 */

#include <sstream>
#include "smc_grippers/Thread.h"
#include "smc_grippers/CoE.h"

using namespace std;

template <class T> inline string type2String(const T& t) {
	
	stringstream out;
	out << t;
	
	return out.str();
}

const int32_t CoE::PRIORITY = RealtimeThread::RT_MAX_PRIORITY-11;   // priority level of private thread

/**
 * Creates an EtherCAT datagram for CANopen over EtherCAT mailbox communication.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, usually the address
 * in the process data RAM of the EtherCAT slave controller for mailbox communication.
 * @param mailboxLength the length of the entire payload of this datagram.
 * @param messageType the CANopen message type, i.e. MESSAGE_TYPE_EMERGENCY_MESSAGE, MESSAGE_TYPE_SDO_REQUEST or MESSAGE_TYPE_SDO_RESPONSE.
 * @param index the index of the CANopen service data object (16 bit).
 * @param subindex the subindex of the CANopen service data object (8 bit).
 * @param value the value to write into this CANopen mailbox datagram (8 - 32 bit).
 * @param length the number of bytes the value consists of, usually 1, 2 or 4.
 */
CoE::CANopenMailboxDatagram::CANopenMailboxDatagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t mailboxLength, uint16_t messageType, uint16_t index, uint8_t subindex, uint32_t value, uint16_t length) : MailboxDatagram(command, deviceAddress, offsetAddress, mailboxLength, EtherCAT::MAILBOX_TYPE_COE, 10) {
    
    data[16] = static_cast<uint8_t>((0 | (messageType << 12)) & 0xFF);
	data[17] = static_cast<uint8_t>(((0 | (messageType << 12)) >> 8) & 0xFF);
	
	data[18] = (length == 0) ? static_cast<uint8_t>(0x40) : static_cast<uint8_t>(0x23 | (((4-length) << 2) & 0x0C));
	data[19] = static_cast<uint8_t>(index & 0xFF);
	data[20] = static_cast<uint8_t>((index >> 8) & 0xFF);
	data[21] = static_cast<uint8_t>(subindex);
	data[22] = static_cast<uint8_t>(value & 0xFF);
	data[23] = static_cast<uint8_t>((value >> 8) & 0xFF);
	data[24] = static_cast<uint8_t>((value >> 16) & 0xFF);
	data[25] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

/**
 * Deletes the CANopen mailbox datagram object.
 */
CoE::CANopenMailboxDatagram::~CANopenMailboxDatagram() {}

/**
 * Gets the CANopen message type.
 * @return the CANopen message type, i.e. MESSAGE_TYPE_EMERGENCY_MESSAGE, MESSAGE_TYPE_SDO_REQUEST or MESSAGE_TYPE_SDO_RESPONSE.
 */
uint16_t CoE::CANopenMailboxDatagram::getMessageType() {
    
    return static_cast<uint16_t>((data[17] >> 4) & 0x0F);
}

CoE::SlaveDevice::SlaveDevice() {}

CoE::SlaveDevice::~SlaveDevice() {}

/**
 * This method is called by the communication handler just before a new
 * EtherCAT frame is transmitted on the fieldbus. It allows a slave device
 * driver to update its datagrams with output process data.
 */
void CoE::SlaveDevice::writeDatagram() {}

/**
 * This method is called by the communication handler just after a new
 * EtherCAT frame was received from the fieldbus. It allows a slave device
 * driver to read its datagrams with input process data.
 */
void CoE::SlaveDevice::readDatagram() {}

CoE::CoE(EtherCAT& etherCAT, double period) : RealtimeThread("CoE", STACK_SIZE, PRIORITY, period), etherCAT(etherCAT) {
    
    // start handler
    
    start();
}

/**
 * Deletes the <code>CoE</code> device driver object and releases all allocated resources.
 */
CoE::~CoE() {

    // stop handler
    
    stop();
}

/**
 * This method must be called by slave device drivers that implement the <code>CoE::SlaveDevice</code> class
 * to register themselves with this <code>CoE</code> object, so that their callback methods are invoked by the
 * communication handler.
 */
void CoE::addSlaveDevice(SlaveDevice* slaveDevice) {
    
    slaveDevices.push_back(slaveDevice);
}

/**
 * This methods allows to register one or several EtherCAT datagrams that contain process
 * data, so that the communication handler transmits them on the fieldbus.
 * <br/>
 * Typically, a slave device driver registers a datagram with output process data and another one
 * with input process data. And these datagrams then need to be updated and processed in the callback
 * methods <code>SlaveDevice::writeDatagram()</code> and <code>SlaveDevice::readDatagram()</code>.
 * @param datagram a pointer to a datagram to register.
 */
void CoE::registerDatagram(EtherCAT::Datagram* datagram) {
    
    datagrams.push_back(datagram);
}

/**
 * Writes an expedited CANopen service data object (SDO) to an EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param mailboxOutAddress the mailbox address for outgoing data within the EtherCAT slave controller.
 * @param mailboxOutSize the size of the mailbox, given in [bytes].
 * @param mailboxInAddress the mailbox address for incoming data within the EtherCAT slave controller.
 * @param mailboxInSize the size of the mailbox, given in [bytes].
 * @param index the index of the CANopen service data object (16 bit).
 * @param subindex the subindex of the CANopen service data object (8 bit).
 * @param value the value to write into this service data object (8 - 32 bit).
 * @param length the number of bytes the value consists of, usually 1, 2 or 4.
 */
void CoE::writeSDO(uint16_t deviceAddress, uint16_t mailboxOutAddress, uint16_t mailboxOutSize, uint16_t mailboxInAddress, uint16_t mailboxInSize, uint16_t index, uint8_t subindex, uint32_t value, uint16_t length) {
    
    vector<EtherCAT::Datagram*> datagrams;
    
    for (uint16_t i = 0; i < RETRIES; i++) {
        
        // read inbox
        
        datagrams.push_back(new CANopenMailboxDatagram(EtherCAT::COMMAND_APRD, deviceAddress, mailboxInAddress, mailboxInSize, MESSAGE_TYPE_SDO_REQUEST, index, subindex, 0, 0));
        try {
            etherCAT.sendDatagrams(datagrams);
        } catch (exception& e) {
            cerr << e.what() << endl;
        }
        while (datagrams.size()) {
            delete datagrams.back();
            datagrams.pop_back();
        }
        
        // transmit SDO request
        
        datagrams.push_back(new CANopenMailboxDatagram(EtherCAT::COMMAND_APWR, deviceAddress, mailboxOutAddress, mailboxOutSize, MESSAGE_TYPE_SDO_REQUEST, index, subindex, value, length));
        try {
            etherCAT.sendDatagrams(datagrams);
        } catch (exception& e) {
            cerr << e.what() << endl;
        }
        if (datagrams[0]->getWorkingCounter() > 0) {
            
            while (datagrams.size()) {
                delete datagrams.back();
                datagrams.pop_back();
            }
            
            for (uint16_t j = 0; j < RETRIES; j++) {
                
                // receive SDO response
                
                datagrams.push_back(new CANopenMailboxDatagram(EtherCAT::COMMAND_APRD, deviceAddress, mailboxInAddress, mailboxInSize, MESSAGE_TYPE_SDO_REQUEST, index, subindex, 0, 0));
                try {
                    etherCAT.sendDatagrams(datagrams);
                } catch (exception& e) {
                    cerr << e.what() << endl;
                }
                if (datagrams[0]->getWorkingCounter() > 0) {
                    
                    //cout << "CoE::writeSDO: index=0x" << hex << index << " subindex=0x" << subindex << " value=" << dec << value << endl;
                    //cout << "     response:";
                    //for (int k = 0; k < 18; k++) cout << hex << " 0x" << ((unsigned int)(datagrams[0]->data[k]) & 0xFF);
                    //cout << endl;
                    //cout << "                     ";
                    //for (int k = 18; k < 26; k++) cout << hex << " 0x" << ((unsigned int)(datagrams[0]->data[k]) & 0xFF);
                    //cout << endl;
                    
                    if ((static_cast<CANopenMailboxDatagram*>(datagrams[0])->getMessageType() == MESSAGE_TYPE_SDO_REQUEST) || (static_cast<CANopenMailboxDatagram*>(datagrams[0])->getMessageType() == MESSAGE_TYPE_SDO_RESPONSE)) {
                        
                        while (datagrams.size()) {
                            delete datagrams.back();
                            datagrams.pop_back();
                        }
                        
                        //cout << "CoE::writeSDO: i=" << dec << i << " j=" << j << endl;
                        
                        return;
                        
                    } else {
                        
                        while (datagrams.size()) {
                            delete datagrams.back();
                            datagrams.pop_back();
                        }
                        
                        Thread::sleep(1);
                        //throw runtime_error("CoE: wrong response from device!");
                    }
                    
                } else {
                    
                    while (datagrams.size()) {
                        delete datagrams.back();
                        datagrams.pop_back();
                    }
                    
                    Thread::sleep(1);
                }
            }
            
        } else {
            
            while (datagrams.size()) {
                delete datagrams.back();
                datagrams.pop_back();
            }
            
            Thread::sleep(1);
        }
    }
    
    throw runtime_error("CoE: mailbox datagram was not processed by device!");
}

/**
 * Reads an expedited CANopen service data object (SDO) from an EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param mailboxOutAddress the mailbox address for outgoing data within the EtherCAT slave controller.
 * @param mailboxOutSize the size of the mailbox, given in [bytes].
 * @param mailboxInAddress the mailbox address for incoming data within the EtherCAT slave controller.
 * @param mailboxInSize the size of the mailbox, given in [bytes].
 * @param index the index of the CANopen service data object (16 bit).
 * @param subindex the subindex of the CANopen service data object (8 bit).
 * @return the value of this service data object.
 */
uint32_t CoE::readSDO(uint16_t deviceAddress, uint16_t mailboxOutAddress, uint16_t mailboxOutSize, uint16_t mailboxInAddress, uint16_t mailboxInSize, uint16_t index, uint8_t subindex) {
    
    vector<EtherCAT::Datagram*> datagrams;
    
    for (uint16_t i = 0; i < RETRIES; i++) {
        
        // read inbox
        
        datagrams.push_back(new CANopenMailboxDatagram(EtherCAT::COMMAND_APRD, deviceAddress, mailboxInAddress, mailboxInSize, MESSAGE_TYPE_SDO_REQUEST, index, subindex, 0, 0));
        try {
            etherCAT.sendDatagrams(datagrams);
        } catch (exception& e) {
            cerr << e.what() << endl;
        }
        while (datagrams.size()) {
            delete datagrams.back();
            datagrams.pop_back();
        }
        
        // transmit SDO request
		
        datagrams.push_back(new CANopenMailboxDatagram(EtherCAT::COMMAND_APWR, deviceAddress, mailboxOutAddress, mailboxOutSize, MESSAGE_TYPE_SDO_REQUEST, index, subindex, 0, 0));
        try {
            etherCAT.sendDatagrams(datagrams);
        } catch (exception& e) {
            cerr << e.what() << endl;
        }
        if (datagrams[0]->getWorkingCounter() > 0) {
            
            while (datagrams.size()) {
                delete datagrams.back();
                datagrams.pop_back();
            }
            
            for (uint16_t j = 0; j < RETRIES; j++) {
                
                // receive SDO response
                
                datagrams.push_back(new CANopenMailboxDatagram(EtherCAT::COMMAND_APRD, deviceAddress, mailboxInAddress, mailboxInSize, MESSAGE_TYPE_SDO_REQUEST, index, subindex, 0, 0));
                try {
                    etherCAT.sendDatagrams(datagrams);
                } catch (exception& e) {
                    cerr << e.what() << endl;
                }
                if (datagrams[0]->getWorkingCounter() > 0) {
                    
                    //cout << "CoE::readSDO: index=0x" << hex << index << " subindex=0x" << subindex << endl;
                    //cout << "     response:";
                    //for (int k = 0; k < 18; k++) cout << hex << " 0x" << ((unsigned int)(datagrams[0]->data[k]) & 0xFF);
                    //cout << endl;
                    //cout << "                    ";
                    //for (int k = 18; k < 26; k++) cout << hex << " 0x" << ((unsigned int)(datagrams[0]->data[k]) & 0xFF);
                    //cout << endl;
                    
                    if ((static_cast<CANopenMailboxDatagram*>(datagrams[0])->getMessageType() == MESSAGE_TYPE_SDO_REQUEST) || (static_cast<CANopenMailboxDatagram*>(datagrams[0])->getMessageType() == MESSAGE_TYPE_SDO_RESPONSE)) {
                        
                        if ((datagrams[0]->data[18] & 0x80) > 0) {
                            
                            while (datagrams.size()) {
                                delete datagrams.back();
                                datagrams.pop_back();
                            }
                            
                            throw runtime_error("CoE: error message from device, class="+type2String(static_cast<uint16_t>(datagrams[0]->data[25]) & 0xFF)+", code="+type2String(static_cast<uint16_t>(datagrams[0]->data[24]) & 0xFF)+".");
                        }
                        
                        uint8_t n = 4-((datagrams[0]->data[18] >> 2) & 0x03);
                        uint32_t value = 0;
                        
                        if (n == 1) value = static_cast<uint32_t>(datagrams[0]->data[22]) & 0xFF;
                        else if (n == 2) value = (static_cast<uint32_t>(datagrams[0]->data[22]) & 0xFF) | ((static_cast<uint32_t>(datagrams[0]->data[23]) & 0xFF) << 8);
                        else value = (static_cast<uint32_t>(datagrams[0]->data[22]) & 0xFF) | ((static_cast<uint32_t>(datagrams[0]->data[23]) & 0xFF) << 8) | ((static_cast<uint32_t>(datagrams[0]->data[24]) & 0xFF) << 16) | ((static_cast<uint32_t>(datagrams[0]->data[25]) & 0xFF) << 24);
                        
                        while (datagrams.size()) {
                            delete datagrams.back();
                            datagrams.pop_back();
                        }
                        
                        //cout << "CoE::readSDO: i=" << dec << i << " j=" << j << endl;
                        
                        return value;
                        
                    } else {
                        
                        while (datagrams.size()) {
                            delete datagrams.back();
                            datagrams.pop_back();
                        }
                        
                        Thread::sleep(1);
                        //throw runtime_error("CoE: wrong response from device!");
                    }
                    
                } else {
                    
                    while (datagrams.size()) {
                        delete datagrams.back();
                        datagrams.pop_back();
                    }
                    
                    Thread::sleep(1);
                }
            }
            
        } else {
            
            while (datagrams.size()) {
                delete datagrams.back();
                datagrams.pop_back();
            }
            
            Thread::sleep(1);
        }
    }
    
    throw runtime_error("CoE: mailbox datagram was not processed by device!");
}

/**
 * This run method implements the periodic communication loop for the fieldbus.
 */
void CoE::run() {

    while (waitForNextPeriod()) {
        
        for (uint16_t i = 0; i < datagrams.size(); i++) datagrams[i]->resetWorkingCounter();
        for (uint16_t i = 0; i < slaveDevices.size(); i++) slaveDevices[i]->writeDatagram();
        try {
            etherCAT.sendDatagrams(datagrams);
        } catch (exception& e) {
            cerr << e.what() << endl;
        }
        for (uint16_t i = 0; i < slaveDevices.size(); i++) slaveDevices[i]->readDatagram();
    }
}
