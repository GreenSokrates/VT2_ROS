/*
 * EtherCAT.cpp
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.07.2017
 *      Author: Marcel Honegger
 */

#include <iostream>
#include <cstring>
#include <errno.h>
#include <strings.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "smc_grippers/Thread.h"
#include "smc_grippers/EtherCAT.h"

using namespace std;

const string EtherCAT::MULTICAST_IP_ADDRESS = "224.0.2.1";      // routable multicast address
const uint16_t EtherCAT::PORT_NUMBER = 0x88A4;                  // EtherCAT port for UDP Datagrams

/**
 * Creates a simple EtherCAT datagram.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. to access slave controller registers.
 * @param length the length of the payload of this datagram.
 */
EtherCAT::Datagram::Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t length) {
    
    this->data = new uint8_t[10+length+2];
    
    // fill in datagram header
    
    this->data[0] = command;
    this->data[1] = 0;
    this->data[2] = static_cast<uint8_t>(deviceAddress & 0xFF);
    this->data[3] = static_cast<uint8_t>((deviceAddress >> 8) & 0xFF);
    this->data[4] = static_cast<uint8_t>(offsetAddress & 0xFF);
    this->data[5] = static_cast<uint8_t>((offsetAddress >> 8) & 0xFF);
    this->data[6] = static_cast<uint8_t>(length & 0xFF);
    this->data[7] = static_cast<uint8_t>((length >> 8) & 0x07);
    this->data[8] = 0;
    this->data[9] = 0;
    
    // fill in datagram data
    
    bzero((void*)&(this->data[10]), length);
    
    this->data[10+length+0] = 0;
    this->data[10+length+1] = 0;
    
    this->length = 10+length+2;
}

/**
 * Creates a simple EtherCAT datagram.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. to access slave controller registers.
 * @param data an array of bytes with the payload of this datagram.
 * @param length the length of the payload of this datagram.
 */
EtherCAT::Datagram::Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint8_t data[], uint16_t length) {
    
    this->data = new uint8_t[10+length+2];
    
    // fill in datagram header
    
    this->data[0] = command;
    this->data[1] = 0;
    this->data[2] = static_cast<uint8_t>(deviceAddress & 0xFF);
    this->data[3] = static_cast<uint8_t>((deviceAddress >> 8) & 0xFF);
    this->data[4] = static_cast<uint8_t>(offsetAddress & 0xFF);
    this->data[5] = static_cast<uint8_t>((offsetAddress >> 8) & 0xFF);
    this->data[6] = static_cast<uint8_t>(length & 0xFF);
    this->data[7] = static_cast<uint8_t>((length >> 8) & 0x07);
    this->data[8] = 0;
    this->data[9] = 0;
    
    // fill in datagram data
    
    memcpy((void*)&(this->data[10]), (void*)data, length);
    
    this->data[10+length+0] = 0;
    this->data[10+length+1] = 0;
    
    this->length = 10+length+2;
}

/**
 * Creates a simple EtherCAT datagram.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. to access slave controller registers.
 * @param value the payload of this datagram.
 * @param length the length of the payload of this datagram.
 */
EtherCAT::Datagram::Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint32_t value, uint16_t length) {
    
    if (length < 4) length = 4;
    
    data = new uint8_t[10+length+2];
    
    // fill in datagram header
    
    data[0] = command;
    data[1] = 0;
    data[2] = static_cast<uint8_t>(deviceAddress & 0xFF);
    data[3] = static_cast<uint8_t>((deviceAddress >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(offsetAddress & 0xFF);
    data[5] = static_cast<uint8_t>((offsetAddress >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(length & 0xFF);
    data[7] = static_cast<uint8_t>((length >> 8) & 0x07);
    data[8] = 0;
    data[9] = 0;
    
    // fill in datagram data
    
    if (length > 0) data[10] = static_cast<uint8_t>((value >> 0) & 0xFF);
    if (length > 1) data[11] = static_cast<uint8_t>((value >> 8) & 0xFF);
    if (length > 2) data[12] = static_cast<uint8_t>((value >> 16) & 0xFF);
    if (length > 3) data[13] = static_cast<uint8_t>((value >> 24) & 0xFF);
    
    data[10+length+0] = 0;
    data[10+length+1] = 0;
    
    this->length = 10+length+2;
}

/**
 * Creates a simple EtherCAT datagram.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. to access slave controller registers.
 * @param value the payload of this datagram.
 * @param length the length of the payload of this datagram.
 */
EtherCAT::Datagram::Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint64_t value, uint16_t length) {
    
    if (length < 8) length = 8;
    
    data = new uint8_t[10+length+2];
    
    // fill in datagram header
    
    data[0] = command;
    data[1] = 0;
    data[2] = static_cast<uint8_t>(deviceAddress & 0xFF);
    data[3] = static_cast<uint8_t>((deviceAddress >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(offsetAddress & 0xFF);
    data[5] = static_cast<uint8_t>((offsetAddress >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(length & 0xFF);
    data[7] = static_cast<uint8_t>((length >> 8) & 0x07);
    data[8] = 0;
    data[9] = 0;
    
    // fill in datagram data
    
    if (length > 0) data[10] = static_cast<uint8_t>((value >> 0) & 0xFF);
    if (length > 1) data[11] = static_cast<uint8_t>((value >> 8) & 0xFF);
    if (length > 2) data[12] = static_cast<uint8_t>((value >> 16) & 0xFF);
    if (length > 3) data[13] = static_cast<uint8_t>((value >> 24) & 0xFF);
    if (length > 4) data[14] = static_cast<uint8_t>((value >> 32) & 0xFF);
    if (length > 5) data[15] = static_cast<uint8_t>((value >> 40) & 0xFF);
    if (length > 6) data[16] = static_cast<uint8_t>((value >> 48) & 0xFF);
    if (length > 7) data[17] = static_cast<uint8_t>((value >> 56) & 0xFF);
    
    data[10+length+0] = 0;
    data[10+length+1] = 0;
    
    this->length = 10+length+2;
}

/**
 * Creates a simple EtherCAT datagram.
 * @param datagram a reference to another datagram to copy.
 */
EtherCAT::Datagram::Datagram(Datagram& datagram) {
    
    this->data = new uint8_t[datagram.length];
	this->length = datagram.length;
	
    memcpy((void*)(this->data), (void*)(datagram.data), this->length);
}

/**
 * Deletes the datagram object.
 */
EtherCAT::Datagram::~Datagram() {
    
    delete[] data;
}

/**
 * Sets or clears the flag that defines if there are more datagrams in the same EtherCAT frame.
 * @param moreDatagrams the flag that defines if there are more datagrams.
 */
void EtherCAT::Datagram::setMoreDatagrams(bool moreDatagrams) {
    
    if (moreDatagrams) {
        
        data[7] |= static_cast<uint8_t>(0x80);
        
    } else {
        
        data[7] &= ~static_cast<uint8_t>(0x80);
    }
}

/**
 * Checks if there are more datagrams in the same EtherCAT frame.
 * @return <code>true</code> if there are more datagrams in the same EtherCAT frame,
 * <code>false</code> otherwise.
 */
bool EtherCAT::Datagram::hasMoreDatagrams() {
    
    return ((data[7] & 0x80) > 0);
}

/**
 * Resets the working counter of the datagram to 0.
 * The working counter denotes how many slave devices this datagram was passing by.
 */
void EtherCAT::Datagram::resetWorkingCounter() {
    
    data[length-2] = 0;
    data[length-1] = 0;
}

/**
 * Gets the working counter of this datagram.
 * @return the working counter. This number denotes how many slave devices this datagram was passing by.
 */
uint16_t EtherCAT::Datagram::getWorkingCounter() {
    
    return ((static_cast<uint16_t>(data[length-2]) & 0xFF) | ((static_cast<uint16_t>(data[length-1]) & 0xFF) << 8));
}

uint8_t EtherCAT::MailboxDatagram::counter = 0;

/**
 * Creates an EtherCAT datagram for mailbox communication.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, usually the address
 * in the process data RAM of the EtherCAT slave controller for mailbox communication.
 * @param mailboxLength the length of the entire payload of this datagram.
 * @param type the mailbox type of this datagram, i.e. MAILBOX_TYPE_COE or MAILBOX_TYPE_FOE.
 * @param length the length of the mailbox content.
 */
EtherCAT::MailboxDatagram::MailboxDatagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t mailboxLength, uint8_t type, uint16_t length) : Datagram(command, deviceAddress, offsetAddress, mailboxLength) {
    
    // increment mailbox counter
    
    counter++;
    if (counter > 7) counter = 1;
    
    // fill in mailbox header
    
    this->data[10] = static_cast<uint8_t>(length & 0xFF);	// length of following data
    this->data[11] = static_cast<uint8_t>((length >> 8) & 0xFF);
    this->data[12] = 0;	// station address of originator
    this->data[13] = 0;
    this->data[14] = 0;	// reserved for future use
    this->data[15] = static_cast<uint8_t>((type & 0x0F) | ((counter << 4) & 0xF0));
    
    // fill in mailbox data
    
    bzero((void*)&(this->data[10+6]), length);
    
    this->data[10+mailboxLength+0] = 0;
    this->data[10+mailboxLength+1] = 0;
}

/**
 * Creates an EtherCAT datagram for mailbox communication.
 * @param command the command of this datagram, i.e. COMMAND_APRD or COMMAND_APWR.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, usually the address
 * in the process data RAM of the EtherCAT slave controller for mailbox communication.
 * @param mailboxLength the length of the entire payload of this datagram.
 * @param type the mailbox type of this datagram, i.e. MAILBOX_TYPE_COE or MAILBOX_TYPE_FOE.
 * @param data an array of bytes with the mailbox content.
 * @param length the length of the mailbox content.
 */
EtherCAT::MailboxDatagram::MailboxDatagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t mailboxLength, uint8_t type, uint8_t data[], uint16_t length) : Datagram(command, deviceAddress, offsetAddress, mailboxLength) {
    
    // increment mailbox counter
    
    counter++;
    if (counter > 7) counter = 1;
    
    // fill in mailbox header
    
    this->data[10] = static_cast<uint8_t>(length & 0xFF);	// length of following data
    this->data[11] = static_cast<uint8_t>((length >> 8) & 0xFF);
    this->data[12] = 0;	// station address of originator
    this->data[13] = 0;
    this->data[14] = 0;	// reserved for future use
    this->data[15] = static_cast<uint8_t>((type & 0x0F) | ((counter << 4) & 0xF0));
    
    // fill in mailbox data
    
    memcpy((void*)&(this->data[10+6]), (void*)data, length);
    
    this->data[10+mailboxLength+0] = 0;
    this->data[10+mailboxLength+1] = 0;
}

/**
 * Deletes the mailbox datagram object.
 */
EtherCAT::MailboxDatagram::~MailboxDatagram() {}

/**
 * Creates an <code>EtherCAT</code> object.
 * @param interfaceAddress the ip address of the network interface to use, i.e. "192.168.1.10" or "127.0.0.1".
 */
EtherCAT::EtherCAT(string interfaceAddress) : interfaceAddress(interfaceAddress) {
    
    // configure multicast socket for communication
    
    networkSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (networkSocket == -1) {
        throw runtime_error("EtherCAT: couldn't create network socket.");
    }
    
    int8_t loopback = 1;
    if (setsockopt(networkSocket, IPPROTO_IP, IP_MULTICAST_LOOP, &loopback, sizeof(loopback)) < 0) {
        throw runtime_error("EtherCAT: couldn't set multicast loop.");
    }
    
    uint8_t ttl = 1;
    if (setsockopt(networkSocket, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0) {
        throw runtime_error("EtherCAT: couldn't change ttl.");
    }
    
    in_addr localInterface;
    localInterface.s_addr = inet_addr(interfaceAddress.c_str());
    if (setsockopt(networkSocket, IPPROTO_IP, IP_MULTICAST_IF, &localInterface, sizeof(localInterface)) < 0) {
        throw runtime_error("EtherCAT: couldn't set local interface.");
    }
    
    ip_mreq group;
    group.imr_multiaddr.s_addr = inet_addr(MULTICAST_IP_ADDRESS.c_str());
    group.imr_interface.s_addr = inet_addr(interfaceAddress.c_str());
    if (setsockopt(networkSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group)) < 0) {
        throw runtime_error("EtherCAT: couldn't add group membership.");
    }
    
    sockaddr_in localSocket;
    memset((int8_t*)&localSocket, 0, sizeof(localSocket));
    localSocket.sin_family = AF_INET;
    localSocket.sin_addr.s_addr = htonl(INADDR_ANY);
    localSocket.sin_port = htons(PORT_NUMBER);
    if (bind(networkSocket, (sockaddr*)&localSocket, sizeof(localSocket))) {
        throw runtime_error("EtherCAT: couldn't bind socket.");
    }
}

/**
 * Deletes the EtherCAT object.
 */
EtherCAT::~EtherCAT() {}

/**
 * Assembles an EtherCAT frame from the list of given datagrams and transmits that
 * EtherCAT frame on the fielbus. Calling this method also reads back the received
 * (processed) datagrams, meaning that the given datagrams will be modified by the
 * EtherCAT slave devices after calling this method.
 * @param datagrams a list of datagrams to transmit on the EtherCAT fieldbus.
 */
void EtherCAT::sendDatagrams(vector<Datagram*> datagrams) {
    
    if (datagrams.size() > 0) {
        
        mutex.lock();
        
        // create byte array with EtherCAT header and datagrams
        
        uint16_t length = 0;    // length of all datagrams, without EtherCAT header, in [bytes]
        for (uint16_t i = 0; i < datagrams.size(); i++) length += datagrams[i]->length;
        
        for (uint16_t i = 0; i < datagrams.size()-1; i++) datagrams[i]->setMoreDatagrams(true);
        datagrams[datagrams.size()-1]->setMoreDatagrams(false);
        
        uint8_t data[2+length]; // byte array constists of header (first 2 bytes) and  all datagrams
        uint16_t header = length | (1 << 12);
        
        data[0] = static_cast<uint8_t>(header & 0xFF);
        data[1] = static_cast<uint8_t>((header >> 8) & 0xFF);
        
        uint16_t index = 2;
        for (uint16_t i = 0; i < datagrams.size(); i++) {
            memcpy((void*)&(data[index]), (void*)(datagrams[i]->data), datagrams[i]->length);
            index += datagrams[i]->length;
        }
        
        // send EtherCAT frame using a UDP datagram
        
        try {
            
            sendMulticastDatagram(data, 2+length);
            
        } catch (exception& e) {
            
            cerr << e.what() << endl;
        }
        
        // receive returning EtherCAT frame
        
        try {
            
            // read loopback response
            
            ssize_t received = receiveMulticastDatagram(data, 2+length);
            
            // read response from EtherCAT slave
            
            bzero((void*)data, 2+length);
            
            received = receiveMulticastDatagram(data, 2+length);
            
            if (received >= 2+length) {
                
                // copy response back into datagrams
                
                index = 2;
                for (uint16_t i = 0; i < datagrams.size(); i++) {
                    memcpy((void*)&(datagrams[i]->data[10]), (void*)&(data[index+10]), datagrams[i]->length-10);
                    index += datagrams[i]->length;
                }
            }
            
        } catch (exception& e) {
            
            cerr << e.what() << endl;
        }
        
        mutex.unlock();
    }
}

/**
 * An utility function to write a simple value to a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @param value the value to write.
 */
void EtherCAT::write8(uint16_t deviceAddress, uint16_t offsetAddress, uint8_t value) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APWR, deviceAddress, offsetAddress, static_cast<uint32_t>(value), 1));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    delete datagrams.back();
    datagrams.pop_back();
}

/**
 * An utility function to read a simple value from a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @return the value at the given offset address.
 */
uint8_t EtherCAT::read8(uint16_t deviceAddress, uint16_t offsetAddress) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APRD, deviceAddress, offsetAddress, static_cast<uint32_t>(0), 1));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    uint8_t value = datagrams[0]->data[10];
    delete datagrams.back();
    datagrams.pop_back();
    
    return value;
}

/**
 * An utility function to write a simple value to a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @param value the value to write.
 */
void EtherCAT::write16(uint16_t deviceAddress, uint16_t offsetAddress, uint16_t value) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APWR, deviceAddress, offsetAddress, static_cast<uint32_t>(value), 2));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    delete datagrams.back();
    datagrams.pop_back();
}

/**
 * An utility function to read a simple value from a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @return the value at the given offset address.
 */
uint16_t EtherCAT::read16(uint16_t deviceAddress, uint16_t offsetAddress) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APRD, deviceAddress, offsetAddress, static_cast<uint32_t>(0), 2));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    uint16_t value = (static_cast<uint16_t>(datagrams[0]->data[10]) & 0xFF) | ((static_cast<uint16_t>(datagrams[0]->data[11]) & 0xFF) << 8);
    delete datagrams.back();
    datagrams.pop_back();
    
    return value;
}

/**
 * An utility function to write a simple value to a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @param value the value to write.
 */
void EtherCAT::write32(uint16_t deviceAddress, uint16_t offsetAddress, uint32_t value) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APWR, deviceAddress, offsetAddress, value, 4));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    delete datagrams.back();
    datagrams.pop_back();
}

/**
 * An utility function to read a simple value from a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @return the value at the given offset address.
 */
uint32_t EtherCAT::read32(uint16_t deviceAddress, uint16_t offsetAddress) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APRD, deviceAddress, offsetAddress, static_cast<uint32_t>(0), 4));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    uint32_t value = (static_cast<uint32_t>(datagrams[0]->data[10]) & 0xFF) | ((static_cast<uint32_t>(datagrams[0]->data[11]) & 0xFF) << 8) | ((static_cast<uint32_t>(datagrams[0]->data[12]) & 0xFF) << 16) | ((static_cast<uint32_t>(datagrams[0]->data[13]) & 0xFF) << 24);
    delete datagrams.back();
    datagrams.pop_back();
    
    return value;
}

/**
 * An utility function to write a simple value to a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @param value the value to write.
 */
void EtherCAT::write64(uint16_t deviceAddress, uint16_t offsetAddress, uint64_t value) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APWR, deviceAddress, offsetAddress, value, 8));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    delete datagrams.back();
    datagrams.pop_back();
}

/**
 * An utility function to read a simple value from a given EtherCAT slave device.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param offsetAddress the address within the EtherCAT slave controller, i.e. the address of a slave controller register.
 * @return the value at the given offset address.
 */
uint64_t EtherCAT::read64(uint16_t deviceAddress, uint16_t offsetAddress) {
    
    vector<Datagram*> datagrams;
    datagrams.push_back(new Datagram(COMMAND_APRD, deviceAddress, offsetAddress, static_cast<uint64_t>(0), 8));
    try {
        sendDatagrams(datagrams);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (datagrams[0]->getWorkingCounter() == 0) throw runtime_error("EtherCAT: datagram was not processed.");
    uint64_t value = (static_cast<uint64_t>(datagrams[0]->data[10]) & 0xFF) | ((static_cast<uint64_t>(datagrams[0]->data[11]) & 0xFF) << 8) | ((static_cast<uint64_t>(datagrams[0]->data[12]) & 0xFF) << 16) | ((static_cast<uint64_t>(datagrams[0]->data[13]) & 0xFF) << 24) | ((static_cast<uint64_t>(datagrams[0]->data[14]) & 0xFF) << 32) | ((static_cast<uint64_t>(datagrams[0]->data[15]) & 0xFF) << 40) | ((static_cast<uint64_t>(datagrams[0]->data[16]) & 0xFF) << 48) | ((static_cast<uint64_t>(datagrams[0]->data[17]) & 0xFF) << 56);
    delete datagrams.back();
    datagrams.pop_back();
    
    return value;
}

/**
 * Sends an EtherCAT frame within a multicast UDP datagram.
 */
void EtherCAT::sendMulticastDatagram(uint8_t data[], uint16_t length) {
    
    sockaddr_in multicastSocket;
    memset((int8_t*)&multicastSocket, 0, sizeof(multicastSocket));
    multicastSocket.sin_family = AF_INET;
    multicastSocket.sin_addr.s_addr = inet_addr(MULTICAST_IP_ADDRESS.c_str());
    multicastSocket.sin_port = htons(PORT_NUMBER);
    
    if (sendto(networkSocket, data, length, 0, (sockaddr*)&multicastSocket, sizeof(multicastSocket)) == -1) {
        throw runtime_error("EtherCAT: couldn't send UDP datagram.");
    }
}

/**
 * Reads a multicast UDP datagram with an EtherCAT frame.
 */
ssize_t EtherCAT::receiveMulticastDatagram(uint8_t data[], uint16_t length) {
    
    // to receive own udp datagrams on linux configure:
    // echo 1 > /proc/sys/net/ipv4/conf/eth0/accept_local
    
    return read(networkSocket, data, length);
}
