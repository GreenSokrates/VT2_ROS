/*
 * CANopen.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 04.11.2015
 *      Author: Marcel Honegger
 */

#ifndef CAN_OPEN_H_
#define CAN_OPEN_H_

#include <cstdlib>
#include <string>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include <pthread.h>
#include "CANMessage.h"
#include "RealtimeThread.h"

class CAN;

/**
 * The CANopen class is a device driver that offers methods to communicate with
 * CANopen slave devices, like industrial I/O or servo drives. It allows to
 * transmit CANopen objects to given nodes, to receive objects from nodes, and
 * to communicate with the service data object (SDO) server of a CANopen device.
 */
class CANopen : public RealtimeThread {
    
    public:
        
        /**
         * This Delegate is an interface for receivers of CANopen objects.
         * Device drivers that use the CANopen stack should implement this class
         * and register themselves with the CANopen device driver.
         */
        class Delegate {
            
            public:
                
                virtual void    receiveObject(uint32_t functionCode, uint8_t object[]);
        };
        
        static const uint32_t NMT = 0x000;          /**< CANopen function code. */
        static const uint32_t SYNC = 0x080;         /**< CANopen function code. */
        static const uint32_t EMERGENCY = 0x080;    /**< CANopen function code. */
        static const uint32_t TPDO1 = 0x180;        /**< CANopen function code. */
        static const uint32_t RPDO1 = 0x200;        /**< CANopen function code. */
        static const uint32_t TPDO2 = 0x280;        /**< CANopen function code. */
        static const uint32_t RPDO2 = 0x300;        /**< CANopen function code. */
        static const uint32_t TPDO3 = 0x380;        /**< CANopen function code. */
        static const uint32_t RPDO3 = 0x400;        /**< CANopen function code. */
        static const uint32_t TPDO4 = 0x480;        /**< CANopen function code. */
        static const uint32_t RPDO4 = 0x500;        /**< CANopen function code. */
        static const uint32_t TSDO = 0x580;         /**< CANopen function code. */
        static const uint32_t RSDO = 0x600;         /**< CANopen function code. */
        static const uint32_t NODEGUARD = 0x700;    /**< CANopen function code. */
        
        static const uint8_t START_REMOTE_NODE = 0x01;          /**< NMT command specifier. */
        static const uint8_t STOP_REMOTE_NODE = 0x02;           /**< NMT command specifier. */
        static const uint8_t ENTER_PREOPERATIONAL_STATE = 0x80; /**< NMT command specifier. */
        static const uint8_t RESET_NODE = 0x81;                 /**< NMT command specifier. */
        static const uint8_t RESET_COMMUNICATION = 0x82;        /**< NMT command specifier. */
        
                    CANopen(CAN& can);
        virtual     ~CANopen();
        void        registerCANopenSlave(uint32_t nodeID, Delegate* delegate);
        void        transmitObject(uint32_t functionCode, uint32_t nodeID, uint8_t object[], uint8_t length = 8, CANType type = CANData);
        void        transmitNMTObject(uint8_t command, uint32_t nodeID);
        void        transmitSYNCObject();
        void        requestNodeguardObject(uint32_t nodeID);
        bool        receiveObject(uint32_t functionCode, uint32_t nodeID, uint8_t object[]);
        void        resetObject(uint32_t functionCode, uint32_t nodeID);
        void        writeSDO(uint32_t nodeID, uint16_t index, uint8_t subindex, uint32_t value, uint8_t length);
        uint32_t    readSDO(uint32_t nodeID, uint16_t index, uint8_t subindex);
        
    private:
        
        static const size_t     STACK_SIZE = 64*1024;   // stack size of private thread polling for CANopen messages in [bytes]
        static const int32_t    PRIORITY;               // priority level of private thread
        static const double     PERIOD;                 // period of private thread in [s]
        
        static const uint32_t   FUNCTION_CODE_BITMASK = 0x780;  // bitmasks for decoding the CAN identifier
        static const uint32_t   NODE_ID_BITMASK = 0x07F;
        
        static const uint32_t   TIMEOUT = 1000;         // max time to wait for a reply on an SDO request in [ms]
        static const uint32_t   RETRIES = 5;            // max number of retries to get an SDO, within the timeout limit
        
        CAN&                can;
        Delegate*           delegate[128];              // registered CANopen slave device drivers
        pthread_mutex_t     mutex;                      // mutex to lock critical sections
        
        bool                emergencyObjectReceived[128];
        bool                tpdo1Received[128];
        bool                tpdo2Received[128];
        bool                tpdo3Received[128];
        bool                tpdo4Received[128];
        bool                tsdoReceived[128];
        bool                nodeguardObjectReceived[128];
        
        uint8_t             emergencyObject[128][8];
        uint8_t             tpdo1[128][8];
        uint8_t             tpdo2[128][8];
        uint8_t             tpdo3[128][8];
        uint8_t             tpdo4[128][8];
        uint8_t             tsdo[128][8];
        uint8_t             nodeguardObject[128][8];
        
        void                run();
};

#endif /* CAN_OPEN_H_ */
