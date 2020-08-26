/*
 * SMCServoJXCE1.h
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 19.07.2017
 *      Author: Marcel Honegger
 */

#ifndef SMC_SERVO_JXCE1_H_
#define SMC_SERVO_JXCE1_H_

#include <cstdlib>
#include <cstdint>
#include "Thread.h"
#include "EtherCAT.h"
#include "CoE.h"
#include "Mutex.h"
#include "Timer.h"

/**
 * This class implements a device driver for the SMC JXCE1 servo controller.
 * <br/>
 * <img src="smcservojxce1.png" width="400" style="text-align:center"/>
 * <br/>
 * <div style="text-align:center"><b>The SMC JXCE1 servo controller</b></div>
 * <br/>
 * An example that shows how to use this device driver is given below:
 * <pre><code>
 * EtherCAT etherCAT("192.168.1.130");  <span style="color:#008000">// create EtherCAT object with given interface</span>
 * CoE coe(etherCAT, 0.005);            <span style="color:#008000">// create CANopen over EtherCAT driver</span>
 * SMCServoJXCE1 servo(etherCAT, coe, 0x0000);  <span style="color:#008000">// create JXCE1 driver with default motion profile values</span>
 * 
 * servo.enable();  <span style="color:#008000">// enable the servo controller</span>
 * servo.setTargetPosition(500);  <span style="color:#008000">// sets the desired position to 5.00 mm</span>
 * ...
 * servo.disable();
 * </code></pre>
 * Also see the documentation about the EtherCAT and the CoE classes for more information.
 */
class SMCServoJXCE1 : public Thread, CoE::SlaveDevice {
    
    public:
        
        static const uint16_t   PROFILE_ACCELERATION = 1000;    /**< Default acceleration, given in [mm/s2]. */
        static const uint16_t   PROFILE_DECELERATION = 1000;    /**< Default deceleration, given in [mm/s2]. */
        static const uint16_t   PROFILE_VELOCITY = 100;         /**< Default velocity, given in [mm/s]. */
        
                    SMCServoJXCE1(EtherCAT& etherCAT, CoE& coe, uint16_t deviceAddress);
                    SMCServoJXCE1(EtherCAT& etherCAT, CoE& coe, uint16_t deviceAddress, uint16_t profileVelocity, uint16_t profileAcceleration, uint16_t profileDeceleration);
        virtual     ~SMCServoJXCE1();
        void        enable();
        bool        isEnabled();
        void        disable();
        void        setTargetPosition(int32_t targetPosition);
        int32_t     getActualPosition();
        
    private:
        
        static const size_t     STACK_SIZE = 32*1024;   // stack size of private thread in [bytes]
        static const int32_t    PRIORITY;               // priority level of private thread
        static const int32_t    PERIOD = 1;             // period of run loop, given in [ms]
        static const uint32_t   TIMEOUT = 100;          // process command timeout, given in [ms]
        
        static const uint16_t   MAILBOX_OUT_ADDRESS = 0x1000;
        static const uint16_t   MAILBOX_OUT_SIZE = 128;
        static const uint16_t   MAILBOX_IN_ADDRESS = 0x1200;
        static const uint16_t   MAILBOX_IN_SIZE = 128;
        static const uint16_t   BUFFERED_OUT_ADDRESS = 0x1400;
        static const uint16_t   BUFFERED_OUT_SIZE = 36;
        static const uint16_t   BUFFERED_IN_ADDRESS = 0x1600;
        static const uint16_t   BUFFERED_IN_SIZE = 20;
        
        static const int16_t    STATE_OFF = 0;          // states of the state machine
        static const int16_t	STATE_RESET_ALARM = 1;
        static const int16_t	STATE_SERVO_ON = 2;
        static const int16_t	STATE_SETUP = 3;
        static const int16_t	STATE_IDLE = 4;
        static const int16_t	STATE_BUSY = 5;
        
        static const uint16_t   DEVICE_STATUS_BUSY = 0x0100;
        static const uint16_t   DEVICE_STATUS_SVRE = 0x0200;
        static const uint16_t   DEVICE_STATUS_INP = 0x0800;
        static const uint16_t   DEVICE_STATUS_ESTOP = 0x4000;
        static const uint16_t   DEVICE_STATUS_ALARM = 0x8000;
        
        EtherCAT&   etherCAT;               // reference to EtherCAT stack
        CoE&        coe;                    // reference to CANopen over EtherCAT driver
        Mutex       mutex;                  // mutex to lock critical sections
        Timer       timer;
        int16_t     state;
        int16_t     stateDemand;
        int32_t     targetPositionSet;
        bool        targetPositionSetFlag;
        
        uint16_t    outputPort;             // 0x7010/0x00 Output port to which signals are allocated
        uint16_t    numericalDataFlag;      // 0x7011/0x00 Numerical data flag
        uint8_t     startFlag;              // 0x7012/0x00 Start flag
        uint8_t     movementMode;           // 0x7020/0x00 Movement mode
        uint16_t    speed;                  // 0x7021/0x00 Speed
        int32_t     targetPosition;         // 0x7022/0x00 Target position
        uint16_t    acceleration;           // 0x7023/0x00 Acceleration
        uint16_t    deceleration;           // 0x7024/0x00 Deceleration
        uint16_t    pushingForce;           // 0x7025/0x00 Pushing force
        uint16_t    triggerLV;              // 0x7026/0x00 Trigger LV
        uint16_t    pushingSpeed;           // 0x7027/0x00 Pushing speed
        uint16_t    movingForce;            // 0x7028/0x00 Moving force
        int32_t     area1;                  // 0x7029/0x00 Area1
        int32_t     area2;                  // 0x702A/0x00 Area2
        int32_t     inPosition;             // 0x702B/0x00 In Position
        
        uint16_t    inputPort;              // 0x6010/0x00 Signal allocated to the input port
        uint16_t    conrollerInputFlag;     // 0x6011/0x00 Controller information flag
        int32_t     currentPosition;        // 0x6020/0x00 Current position [0.01mm]
        uint16_t    currentSpeed;           // 0x6021/0x00 Current speed [mm/s]
        uint16_t    currentPushingForce;    // 0x6022/0x00 Current pushing force [%]
        int32_t     targetPositionDisplay;  // 0x6023/0x00 Target position [0.01mm]
        uint8_t     alarm1;                 // 0x6030/0x01 Alarm1
        uint8_t     alarm2;                 // 0x6030/0x02 Alarm2
        uint8_t     alarm3;                 // 0x6030/0x03 Alarm3
        uint8_t     alarm4;                 // 0x6030/0x04 Alarm4
        
        EtherCAT::Datagram*     rxPDO;
        EtherCAT::Datagram*     txPDO;
        
        void        initializeEtherCAT(uint16_t deviceAddress);
        void        writeDatagram();
        void        readDatagram();
        void        run();
};

#endif /* SMC_SERVO_JXCE1_H_ */
