/*
 * CoE.h
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 14.07.2017
 *      Author: Marcel Honegger
 */

#ifndef COE_H_
#define COE_H_

#include <cstdlib>
#include <cstdio>
#include <vector>
#include <stdint.h>
#include "EtherCAT.h"
#include "RealtimeThread.h"

using namespace std;

/**
 * This class implements the CANopen over EtherCAT (CoE) protocol.
 * It offers methods to access the CANopen object dictionary entries of an EtherCAT
 * slave device to configure that device. It also offers a method to register
 * EtherCAT datagrams that contain process data (input and output).
 * <br/><br/>
 * This class implements a high priority, periodic realtime thread that handles the
 * EtherCAT communication. The period of this thread can be configured in the constructor
 * of this class. It corresponds to the cycle time of the communication loop on the
 * EtherCAT fieldbus.
 * <br/>
 * Just before and after a communication cycle, this class invokes callback methods of
 * registered slave device drivers. This allows the slave device drivers to update datagrams
 * with output process data and to read datagrams with input process data. Slave device drivers
 * therefore do not need to implement their own handling threads, instead they only need to
 * extend the <code>CoE::SlaveDevice</code> class.
 */
class CoE : public RealtimeThread {
    
    public:
        
        /**
         * The <code>CANopenMailboxDatagram</code> is an extension of the <code>MailboxDatagram</code>
         * for CANopen over EtherCAT mailbox communication.
         */
        class CANopenMailboxDatagram : public EtherCAT::MailboxDatagram {
            
            public:
                
                            CANopenMailboxDatagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t mailboxLength, uint16_t messageType, uint16_t index, uint8_t subindex, uint32_t value, uint16_t length);
                            ~CANopenMailboxDatagram();
                uint16_t    getMessageType();
        };
        
        /**
         * The <code>SlaveDevice</code> class offers callback methods for an EtherCAT master.
         * It needs to be implemented by EtherCAT slave device drivers.
         */
        class SlaveDevice {
            
            public:
                
                                SlaveDevice();
                virtual         ~SlaveDevice();
                virtual void    writeDatagram();
                virtual void    readDatagram();
        };
        
        static const uint16_t   MESSAGE_TYPE_EMERGENCY_MESSAGE = 1;     /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_SDO_REQUEST = 2;           /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_SDO_RESPONSE = 3;          /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_TXPDO = 4;                 /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_RXPDO = 5;                 /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_RTR_OF_TXPDO = 6;          /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_RTR_OF_RXPDO = 7;          /**< CANopen message type. */
        static const uint16_t   MESSAGE_TYPE_SDO_INFORMATION = 8;       /**< CANopen message type. */
        
                                CoE(EtherCAT& etherCAT, double period);
        virtual                 ~CoE();
        void                    addSlaveDevice(SlaveDevice* slaveDevice);
        void                    registerDatagram(EtherCAT::Datagram* datagram);
        void                    writeSDO(uint16_t deviceAddress, uint16_t mailboxOutAddress, uint16_t mailboxOutSize, uint16_t mailboxInAddress, uint16_t mailboxInSize, uint16_t index, uint8_t subindex, uint32_t value, uint16_t length);
        uint32_t                readSDO(uint16_t deviceAddress, uint16_t mailboxOutAddress, uint16_t mailboxOutSize, uint16_t mailboxInAddress, uint16_t mailboxInSize, uint16_t index, uint8_t subindex);
        void                    run();
        
    private:
        
        static const size_t     STACK_SIZE = 32*1024;   // stack size of private thread in [bytes]
        static const int32_t    PRIORITY;               // priority level of private thread
        static const uint16_t   RETRIES = 10;
        
        EtherCAT&                   etherCAT;
        vector<SlaveDevice*>        slaveDevices;
        vector<EtherCAT::Datagram*> datagrams;
};

#endif /* COE_H_ */
