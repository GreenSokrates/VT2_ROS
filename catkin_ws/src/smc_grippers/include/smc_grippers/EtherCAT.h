/*
 * EtherCAT.h
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.07.2017
 *      Author: Marcel Honegger
 */

#ifndef ETHER_CAT_H_
#define ETHER_CAT_H_

#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <unistd.h>
#include "Mutex.h"

using namespace std;

/**
 * This class implements a simple EtherCAT master stack to communicate with
 * devices on an EtherCAT fieldbus.
 * <br/>
 * This master stack transmits EtherCAT datagrams using multicast UDP packets.
 * In order to receive responses from EtherCAT slave devices, the host system
 * must be able to receive its own UDP packets. On a linux system, this must
 * be enabled as follows:
 * <pre><code>
 * echo 1 > /proc/sys/net/ipv4/conf/eth0/accept_local
 * </code></pre>
 * In this example, 'eth0' is the name of the Ethernet interface to use.
 */
class EtherCAT {
    
    public:
        
        /**
         * The <code>Datagram</code> class represents a simple EtherCAT datagram.
         */
        class Datagram {
            
            public:
                
                uint8_t*    data;
                uint16_t    length;
                
                            Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t length);
                            Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint8_t data[], uint16_t length);
                            Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint32_t value, uint16_t length);
                            Datagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint64_t value, uint16_t length);
                            Datagram(Datagram& datagram);
                            ~Datagram();
                void        setMoreDatagrams(bool moreDatagrams);
                bool        hasMoreDatagrams();
                void        resetWorkingCounter();
                uint16_t    getWorkingCounter();
        };
        
        /**
         * The <code>MailboxDatagram</code> is an extension of the <code>Datagram</code>
         * for EtherCAT mailbox communication.
         */
        class MailboxDatagram : public Datagram {
            
            public:
                
                            MailboxDatagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t mailboxLength, uint8_t type, uint16_t length);
                            MailboxDatagram(uint8_t command, uint16_t deviceAddress, uint16_t offsetAddress, uint16_t mailboxLength, uint8_t type, uint8_t data[], uint16_t length);
                            ~MailboxDatagram();
                
            private:
                
                static uint8_t  counter;
        };
        
        static const uint16_t   ESC_INFORMATION = 0x0000;                           /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_TYPE = 0x0000;                      /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_REVISION = 0x0001;                  /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_BUILD = 0x0002;                     /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_FMMUS_SUPPORTED = 0x0004;           /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_SYNC_MANAGERS_SUPPORTED = 0x0005;   /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_RAM_SIZE = 0x0006;                  /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_PORT_DESCRIPTOR = 0x0007;           /**< ESC register address. */
        static const uint16_t   ESC_INFORMATION_FEATURES_SUPPORTED = 0x0008;        /**< ESC register address. */
        static const uint16_t   STATION_ADDRESS = 0x0010;                           /**< ESC register address. */
        static const uint16_t   STATION_ADDRESS_CONFIGURED_ADDRESS = 0x0010;        /**< ESC register address. */
        static const uint16_t   STATION_ADDRESS_CONFIGURED_ALIAS = 0x0012;          /**< ESC register address. */
        static const uint16_t   DATA_LINK_LAYER = 0x0040;                           /**< ESC register address. */
        static const uint16_t   DATA_LINK_LAYER_STATUS = 0x0110;                    /**< ESC register address. */
        static const uint16_t   APPLICATION_LAYER = 0x0120;                         /**< ESC register address. */
        static const uint16_t   APPLICATION_LAYER_CONTROL = 0x0120;                 /**< ESC register address. */
        static const uint16_t   APPLICATION_LAYER_STATUS = 0x0130;                  /**< ESC register address. */
        static const uint16_t   APPLICATION_LAYER_STATUS_CODE = 0x0134;             /**< ESC register address. */
        static const uint16_t   WATCHDOG_TIME_PDI = 0x0410;                         /**< ESC register address. */
        static const uint16_t   WATCHDOG_TIME_PROCESS_DATA = 0x0420;                /**< ESC register address. */
        static const uint16_t   WATCHDOG_COUNTER_PROCESS_DATA = 0x0442;             /**< ESC register address. */
        static const uint16_t   INTERRUPTS = 0x0200;                                /**< ESC register address. */
        static const uint16_t   SYNC_MANAGER = 0x0800;                              /**< ESC register address. */
        static const uint16_t   SYNC_MANAGER_LENGTH = 0x0802;                       /**< ESC register address. */
        static const uint16_t   SYNC_MANAGER_CONTROL = 0x0804;                      /**< ESC register address. */
        static const uint16_t   SYNC_MANAGER_ACTIVATE = 0x0806;                     /**< ESC register address. */
        static const uint16_t   SYNC_MANAGER_OFFSET = 0x0008;                       /**< ESC register address. */
        static const uint16_t   DC_RECEIVE_TIME_PORT_0 = 0x0900;                    /**< ESC register address. */
        static const uint16_t   DC_RECEIVE_TIME_PORT_1 = 0x0904;                    /**< ESC register address. */
        static const uint16_t   DC_SYSTEM_TIME = 0x0910;                            /**< ESC register address. */
        static const uint16_t   DC_SYSTEM_TIME_OFFSET = 0x0920;                     /**< ESC register address. */
        static const uint16_t   DC_CYCLIC_UNIT_CONTROL = 0x0980;                    /**< ESC register address. */
        static const uint16_t   DC_ACTIVATION_REGISTER = 0x0981;                    /**< ESC register address. */
        static const uint16_t   DC_START_TIME_CYCLIC_OPERATION = 0x0990;            /**< ESC register address. */
        static const uint16_t   DC_SYNC0_CYCLE_TIME = 0x09A0;                       /**< ESC register address. */
        static const uint16_t   DC_SYNC1_CYCLE_TIME = 0x09A4;                       /**< ESC register address. */
        
        static const uint16_t   STATE_MASK = 0x0F;              /**< EtherCAT state machine mask. */
        static const uint16_t   STATE_ERROR_MASK = 0x10;        /**< EtherCAT state machine mask. */
        static const uint16_t   STATE_INIT = 0x01;              /**< EtherCAT state. */
        static const uint16_t   STATE_BOOTSTRAP = 0x03;         /**< EtherCAT state. */
        static const uint16_t   STATE_PRE_OPERATIONAL = 0x02;   /**< EtherCAT state. */
        static const uint16_t   STATE_SAFE_OPERATIONAL = 0x04;  /**< EtherCAT state. */
        static const uint16_t   STATE_OPERATIONAL = 0x08;       /**< EtherCAT state. */
        static const uint16_t   STATE_ERROR = 0x10;             /**< EtherCAT state. */
        
        static const uint8_t    COMMAND_NOP = 0;                /**< EtherCAT datagram command. */
        static const uint8_t    COMMAND_APRD = 1;               /**< EtherCAT datagram command. */
        static const uint8_t    COMMAND_APWR = 2;               /**< EtherCAT datagram command. */
        static const uint8_t    COMMAND_APRW = 3;               /**< EtherCAT datagram command. */
        static const uint8_t    COMMAND_FPRD = 4;               /**< EtherCAT datagram command. */
        static const uint8_t    COMMAND_FPWR = 5;               /**< EtherCAT datagram command. */
        static const uint8_t    COMMAND_FPRW = 6;               /**< EtherCAT datagram command. */
        
        static const uint8_t    MAILBOX_TYPE_MAILBOX_ERROR = 0x0;   /**< EtherCAT mailbox type. */
        static const uint8_t    MAILBOX_TYPE_EOE = 0x2;             /**< EtherCAT mailbox type. */
        static const uint8_t    MAILBOX_TYPE_COE = 0x3;             /**< EtherCAT mailbox type. */
        static const uint8_t    MAILBOX_TYPE_FOE = 0x4;             /**< EtherCAT mailbox type. */
        static const uint8_t    MAILBOX_TYPE_SOE = 0x5;             /**< EtherCAT mailbox type. */
        static const uint8_t    MAILBOX_TYPE_VOE = 0xF;             /**< EtherCAT mailbox type. */
        
                    EtherCAT(string interfaceAddress);
        virtual     ~EtherCAT();
        void        sendDatagrams(vector<Datagram*> datagrams);
        void        write8(uint16_t deviceAddress, uint16_t offsetAddress, uint8_t value);
        uint8_t     read8(uint16_t deviceAddress, uint16_t offsetAddress);
        void        write16(uint16_t deviceAddress, uint16_t offsetAddress, uint16_t value);
        uint16_t    read16(uint16_t deviceAddress, uint16_t offsetAddress);
        void        write32(uint16_t deviceAddress, uint16_t offsetAddress, uint32_t value);
        uint32_t    read32(uint16_t deviceAddress, uint16_t offsetAddress);
        void        write64(uint16_t deviceAddress, uint16_t offsetAddress, uint64_t value);
        uint64_t    read64(uint16_t deviceAddress, uint16_t offsetAddress);
        
    private:
        
        static const string     MULTICAST_IP_ADDRESS;
        static const uint16_t   PORT_NUMBER;
        
        string      interfaceAddress;
        int32_t     networkSocket;
        Mutex       mutex;
        
        void        sendMulticastDatagram(uint8_t data[], uint16_t length);
        ssize_t     receiveMulticastDatagram(uint8_t data[], uint16_t length);
};

#endif /* ETHER_CAT_H_ */
