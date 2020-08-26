/*
 * SMCServoJXCE1.cpp
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 19.07.2017
 *      Author: Marcel Honegger
 */

#include "smc_grippers/Thread.h"
#include "smc_grippers/SMCServoJXCE1.h"

using namespace std;

const int32_t SMCServoJXCE1::PRIORITY = Thread::MAX_PRIORITY;   // priority level of private thread

/**
 * Creates an <code>SMCServoJXCE1</code> object and starts a handler with a state machine.
 * @param etherCAT a reference to an EtherCAT object this device driver depends on.
 * @param coe a reference to a CoE object this device driver depends on.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 */
SMCServoJXCE1::SMCServoJXCE1(EtherCAT& etherCAT, CoE& coe, uint16_t deviceAddress) : Thread("SMCServoJXCE1", STACK_SIZE, PRIORITY), etherCAT(etherCAT), coe(coe) {
    
    // initialize local values
    
    timer.start();
    state = STATE_OFF;
    stateDemand = STATE_OFF;
    targetPositionSet = 0;
    targetPositionSetFlag = false;
    
    // initialize process data values
    
    outputPort = 0x0000; // i.e. 0x0200 enable servo, 0x0800 reset servo
    numericalDataFlag = 0x0000; // set values
    startFlag = 0; // 1 = transfer set values
    movementMode = 0x01; // absolute position
    speed = PROFILE_VELOCITY; // i.e. 100 mm/s
    targetPosition = 0; // in 0.01 mm
    acceleration = PROFILE_ACCELERATION; // i.e. 100 mm/s2
    deceleration = PROFILE_DECELERATION; // i.e. 100 mm/s2
    pushingForce = 0; // 0%
    triggerLV = 0; // 0%
    pushingSpeed = 10; // 10 mm/s
    movingForce = 100; // 100%
    area1 = 0; // in 0.01 mm
    area2 = 0; // in 0.01 mm
    inPosition = 1; // in 0.01 mm
    
    // initialize EtherCAT communication
    
    initializeEtherCAT(deviceAddress);
    
    // start handler
    
    start();
}

/**
 * Creates an <code>SMCServoJXCE1</code> object and starts a handler with a state machine.
 * @param etherCAT a reference to an EtherCAT object this device driver depends on.
 * @param coe a reference to a CoE object this device driver depends on.
 * @param deviceAddress the relative device address, i.e. 0x0000, 0xFFFF, 0xFFFE, etc.
 * @param profileVelocity the maximum velocity of motions, given in [mm/s].
 * @param profileAcceleration the maximum acceleration of motions, given in [mm/s&sup2;].
 * @param profileDeceleration the maximum deceleration of motions, given in [mm/s&sup2;].
 */
SMCServoJXCE1::SMCServoJXCE1(EtherCAT& etherCAT, CoE& coe, uint16_t deviceAddress, uint16_t profileVelocity, uint16_t profileAcceleration, uint16_t profileDeceleration) : Thread("SMCServoJXCE1", STACK_SIZE, PRIORITY), etherCAT(etherCAT), coe(coe) {
    
    // initialize local values
    
    timer.start();
    state = STATE_OFF;
    stateDemand = STATE_OFF;
    targetPositionSet = 0;
    targetPositionSetFlag = false;
    
    // initialize process data values
    
    outputPort = 0x0000; // i.e. 0x0200 enable servo, 0x0800 reset servo
    numericalDataFlag = 0x0000; // set values
    startFlag = 0; // 1 = transfer set values
    movementMode = 0x01; // absolute position
    speed = profileVelocity; // i.e. 100 mm/s
    targetPosition = 0; // in 0.01 mm
    acceleration = profileAcceleration; // i.e. 100 mm/s2
    deceleration = profileDeceleration; // i.e. 100 mm/s2
    pushingForce = 0; // 0%
    triggerLV = 0; // 0%
    pushingSpeed = 10; // 10 mm/s
    movingForce = 100; // 100%
    area1 = 0; // in 0.01 mm
    area2 = 0; // in 0.01 mm
    inPosition = 1; // in 0.01 mm
    
    // initialize EtherCAT communication
    
    initializeEtherCAT(deviceAddress);
    
    // start handler
    
    start();
}

/**
 * Deletes the <code>SMCServoJXCE1</code> device driver object and releases all allocated resources.
 */
SMCServoJXCE1::~SMCServoJXCE1() {}

/**
 * Enables the servo controller.
 */
void SMCServoJXCE1::enable() {
    
    stateDemand = STATE_IDLE;
}

/**
 * Checks if the servo controller is enabled.
 * @return <code>true</code> if the servo controller is ready, <code>false</code> otherwise.
 */
bool SMCServoJXCE1::isEnabled() {
    
    return (state == STATE_IDLE) || (state == STATE_BUSY);
}

/**
 * Disables the servo controller.
 */
void SMCServoJXCE1::disable() {
    
    stateDemand = STATE_OFF;
}

/**
 * Sets the desired target position of the servo motor.
 * @param targetPosition the desired position, given in [0.01 mm].
 */
void SMCServoJXCE1::setTargetPosition(int32_t targetPosition) {
    
    mutex.lock();
    
    targetPositionSet = targetPosition;
    targetPositionSetFlag = true;
    
    mutex.unlock();
}

/**
 * Gets the actual position of the servo motor.
 * @return the actual position, given in [0.01 mm].
 */
int32_t SMCServoJXCE1::getActualPosition() {
    
    mutex.lock();
    
    int32_t actualPosition = currentPosition;
    
    mutex.unlock();
    
    return actualPosition;
}

/**
 * Initializes the EtherCAT slave controller for process data communication.
 */
void SMCServoJXCE1::initializeEtherCAT(uint16_t deviceAddress) {
    
    // set EtherCAT state machine to state INIT
    
    uint16_t loop = 0;
    uint16_t state = 0;
    while ((state != EtherCAT::STATE_INIT) && (loop++ < 100)) try {
        state = etherCAT.read16(deviceAddress, EtherCAT::APPLICATION_LAYER_STATUS);
        Thread::sleep(10);
        if ((state & EtherCAT::STATE_ERROR_MASK) == EtherCAT::STATE_ERROR) {
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state & EtherCAT::STATE_MASK);
            Thread::sleep(10);
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state);
        } else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_BOOTSTRAP) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_INIT);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_PRE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_INIT);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_SAFE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_PRE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_SAFE_OPERATIONAL);
        Thread::sleep(10);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (state != EtherCAT::STATE_INIT) throw runtime_error("SMCServoJXCE1: couldn't enter state INIT.");
    
    Thread::sleep(10);
    
    // initialize SYNC managers
    
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+0*EtherCAT::SYNC_MANAGER_OFFSET, 0);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER+0*EtherCAT::SYNC_MANAGER_OFFSET, MAILBOX_OUT_ADDRESS);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER_LENGTH+0*EtherCAT::SYNC_MANAGER_OFFSET, MAILBOX_OUT_SIZE);
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_CONTROL+0*EtherCAT::SYNC_MANAGER_OFFSET, 0x26);	// mailbox, write, interrupt in PDI
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+0*EtherCAT::SYNC_MANAGER_OFFSET, 1);
    Thread::sleep(10);
    
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+1*EtherCAT::SYNC_MANAGER_OFFSET, 0);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER+1*EtherCAT::SYNC_MANAGER_OFFSET, 0x1200);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER_LENGTH+1*EtherCAT::SYNC_MANAGER_OFFSET, MAILBOX_IN_SIZE);
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_CONTROL+1*EtherCAT::SYNC_MANAGER_OFFSET, 0x22);	// mailbox, read, interrupt in PDI
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+1*EtherCAT::SYNC_MANAGER_OFFSET, 1);
    Thread::sleep(10);
    
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+2*EtherCAT::SYNC_MANAGER_OFFSET, 0);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER+2*EtherCAT::SYNC_MANAGER_OFFSET, 0x1400);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER_LENGTH+2*EtherCAT::SYNC_MANAGER_OFFSET, BUFFERED_OUT_SIZE);
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_CONTROL+2*EtherCAT::SYNC_MANAGER_OFFSET, 0x24);	// buffered, write, interrupt in PDI
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+2*EtherCAT::SYNC_MANAGER_OFFSET, 1);
    Thread::sleep(10);
    
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+3*EtherCAT::SYNC_MANAGER_OFFSET, 0);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER+3*EtherCAT::SYNC_MANAGER_OFFSET, 0x1600);
    Thread::sleep(10);
    etherCAT.write16(deviceAddress, EtherCAT::SYNC_MANAGER_LENGTH+3*EtherCAT::SYNC_MANAGER_OFFSET, BUFFERED_IN_SIZE);
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_CONTROL+3*EtherCAT::SYNC_MANAGER_OFFSET, 0x20);	// buffered, read, interrupt in PDI
    Thread::sleep(10);
    etherCAT.write8(deviceAddress, EtherCAT::SYNC_MANAGER_ACTIVATE+3*EtherCAT::SYNC_MANAGER_OFFSET, 1);
    Thread::sleep(10);
    
    // set EtherCAT state machine to state PRE OPERATIONAL
    
    loop = 0;
    state = 0;
    while ((state != EtherCAT::STATE_PRE_OPERATIONAL) && (loop++ < 100)) try {
        state = etherCAT.read16(deviceAddress, EtherCAT::APPLICATION_LAYER_STATUS);
        Thread::sleep(10);
        if ((state & EtherCAT::STATE_ERROR_MASK) == EtherCAT::STATE_ERROR) {
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state & EtherCAT::STATE_MASK);
            Thread::sleep(10);
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state);
        } else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_INIT) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_PRE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_BOOTSTRAP) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_INIT);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_SAFE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_PRE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_SAFE_OPERATIONAL);
        Thread::sleep(10);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (state != EtherCAT::STATE_PRE_OPERATIONAL) throw runtime_error("SMCServoJXCE1: couldn't enter state PRE OPERATIONAL.");
    
    // register process datagrams
    
    coe.registerDatagram(rxPDO = new EtherCAT::Datagram(EtherCAT::COMMAND_APWR, deviceAddress, BUFFERED_OUT_ADDRESS, BUFFERED_OUT_SIZE));
    coe.registerDatagram(txPDO = new EtherCAT::Datagram(EtherCAT::COMMAND_APRD, deviceAddress, BUFFERED_IN_ADDRESS, BUFFERED_IN_SIZE));
    
    coe.addSlaveDevice(this);
    
    // set EtherCAT state machine to state SAFE OPERATIONAL
    
    loop = 0;
    state = 0;
    while ((state != EtherCAT::STATE_SAFE_OPERATIONAL) && (loop++ < 100)) try {
        state = etherCAT.read16(deviceAddress, EtherCAT::APPLICATION_LAYER_STATUS);
        Thread::sleep(10);
        if ((state & EtherCAT::STATE_ERROR_MASK) == EtherCAT::STATE_ERROR) {
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state & EtherCAT::STATE_MASK);
            Thread::sleep(10);
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state);
        } else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_INIT) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_PRE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_BOOTSTRAP) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_INIT);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_PRE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_SAFE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_SAFE_OPERATIONAL);
        Thread::sleep(10);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (state != EtherCAT::STATE_SAFE_OPERATIONAL) throw runtime_error("SMCServoJXCE1: couldn't enter state SAFE OPERATIONAL.");
    
    // set EtherCAT state machine to state OPERATIONAL
    
    loop = 0;
    state = 0;
    while ((state != EtherCAT::STATE_OPERATIONAL) && (loop++ < 100)) try {
        state = etherCAT.read16(deviceAddress, EtherCAT::APPLICATION_LAYER_STATUS);
        Thread::sleep(10);
        if ((state & EtherCAT::STATE_ERROR_MASK) == EtherCAT::STATE_ERROR) {
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state & EtherCAT::STATE_MASK);
            Thread::sleep(10);
            etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, state);
        } else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_INIT) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_PRE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_BOOTSTRAP) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_INIT);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_PRE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_SAFE_OPERATIONAL);
        else if ((state & EtherCAT::STATE_MASK) == EtherCAT::STATE_SAFE_OPERATIONAL) etherCAT.write16(deviceAddress, EtherCAT::APPLICATION_LAYER_CONTROL, EtherCAT::STATE_OPERATIONAL);
        Thread::sleep(10);
    } catch (exception& e) {
        cerr << e.what() << endl;
    }
    if (state != EtherCAT::STATE_OPERATIONAL) throw runtime_error("SMCServoJXCE1: couldn't enter state OPERATIONAL.");
}

/**
 * This method is called by the communication handler just before a new
 * EtherCAT frame is transmitted on the fieldbus. It allows this device
 * driver to update its datagram with output process data.
 */
void SMCServoJXCE1::writeDatagram() {
    
    mutex.lock();
    
    rxPDO->data[10] = static_cast<uint8_t>(outputPort & 0xFF);
	rxPDO->data[11] = static_cast<uint8_t>((outputPort >> 8) & 0xFF);
    rxPDO->data[12] = static_cast<uint8_t>(numericalDataFlag & 0xFF);
	rxPDO->data[13] = static_cast<uint8_t>((numericalDataFlag >> 8) & 0xFF);
    rxPDO->data[14] = startFlag;
    rxPDO->data[15] = movementMode;
    rxPDO->data[16] = static_cast<uint8_t>(speed & 0xFF);
	rxPDO->data[17] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    rxPDO->data[18] = static_cast<uint8_t>(targetPosition & 0xFF);
	rxPDO->data[19] = static_cast<uint8_t>((targetPosition >> 8) & 0xFF);
    rxPDO->data[20] = static_cast<uint8_t>((targetPosition >> 16) & 0xFF);
	rxPDO->data[21] = static_cast<uint8_t>((targetPosition >> 24) & 0xFF);
    rxPDO->data[22] = static_cast<uint8_t>(acceleration & 0xFF);
	rxPDO->data[23] = static_cast<uint8_t>((acceleration >> 8) & 0xFF);
    rxPDO->data[24] = static_cast<uint8_t>(deceleration & 0xFF);
	rxPDO->data[25] = static_cast<uint8_t>((deceleration >> 8) & 0xFF);
    rxPDO->data[26] = static_cast<uint8_t>(pushingForce & 0xFF);
	rxPDO->data[27] = static_cast<uint8_t>((pushingForce >> 8) & 0xFF);
    rxPDO->data[28] = static_cast<uint8_t>(triggerLV & 0xFF);
	rxPDO->data[29] = static_cast<uint8_t>((triggerLV >> 8) & 0xFF);
    rxPDO->data[30] = static_cast<uint8_t>(pushingSpeed & 0xFF);
	rxPDO->data[31] = static_cast<uint8_t>((pushingSpeed >> 8) & 0xFF);
    rxPDO->data[32] = static_cast<uint8_t>(movingForce & 0xFF);
	rxPDO->data[33] = static_cast<uint8_t>((movingForce >> 8) & 0xFF);
    rxPDO->data[34] = static_cast<uint8_t>(area1 & 0xFF);
	rxPDO->data[35] = static_cast<uint8_t>((area1 >> 8) & 0xFF);
    rxPDO->data[36] = static_cast<uint8_t>((area1 >> 16) & 0xFF);
	rxPDO->data[37] = static_cast<uint8_t>((area1 >> 24) & 0xFF);
    rxPDO->data[38] = static_cast<uint8_t>(area2 & 0xFF);
	rxPDO->data[39] = static_cast<uint8_t>((area2 >> 8) & 0xFF);
    rxPDO->data[40] = static_cast<uint8_t>((area2 >> 16) & 0xFF);
	rxPDO->data[41] = static_cast<uint8_t>((area2 >> 24) & 0xFF);
    rxPDO->data[42] = static_cast<uint8_t>(inPosition & 0xFF);
	rxPDO->data[43] = static_cast<uint8_t>((inPosition >> 8) & 0xFF);
    rxPDO->data[44] = static_cast<uint8_t>((inPosition >> 16) & 0xFF);
	rxPDO->data[45] = static_cast<uint8_t>((inPosition >> 24) & 0xFF);
    
    mutex.unlock();
}

/**
 * This method is called by the communication handler just after a new
 * EtherCAT frame was received from the fieldbus. It allows this device
 * driver to read its datagram with input process data.
 */
void SMCServoJXCE1::readDatagram() {
    
    mutex.lock();
    
    inputPort = (static_cast<uint16_t>(txPDO->data[10]) & 0xFF) | ((static_cast<uint16_t>(txPDO->data[11]) & 0xFF) << 8);
    conrollerInputFlag = (static_cast<uint16_t>(txPDO->data[12]) & 0xFF) | ((static_cast<uint16_t>(txPDO->data[13]) & 0xFF) << 8);
    currentPosition = (static_cast<int32_t>(txPDO->data[14]) & 0xFF) | ((static_cast<int32_t>(txPDO->data[15]) & 0xFF) << 8) | ((static_cast<int32_t>(txPDO->data[16]) & 0xFF) << 16) | ((static_cast<int32_t>(txPDO->data[17]) & 0xFF) << 24);
    currentSpeed = (static_cast<uint16_t>(txPDO->data[18]) & 0xFF) | ((static_cast<uint16_t>(txPDO->data[19]) & 0xFF) << 8);
    currentPushingForce = (static_cast<uint16_t>(txPDO->data[20]) & 0xFF) | ((static_cast<uint16_t>(txPDO->data[21]) & 0xFF) << 8);
    targetPositionDisplay = (static_cast<int32_t>(txPDO->data[22]) & 0xFF) | ((static_cast<int32_t>(txPDO->data[23]) & 0xFF) << 8) | ((static_cast<int32_t>(txPDO->data[24]) & 0xFF) << 16) | ((static_cast<int32_t>(txPDO->data[25]) & 0xFF) << 24);
    alarm1 = txPDO->data[26];
    alarm2 = txPDO->data[27];
    alarm3 = txPDO->data[28];
    alarm4 = txPDO->data[29];
    
    mutex.unlock();
}

/**
 * This run method implements the logic of a state machine of this device driver.
 */
void SMCServoJXCE1::run() {
    
    while (true) {
        
		sleep(PERIOD);
        
        // get actual device status
        
        mutex.lock();
        
        uint16_t deviceStatus = inputPort;
        
        mutex.unlock();
        
        // handle state machine logic
        
        switch (state) {
            
            case STATE_OFF:
                
                if ((timer > TIMEOUT) && (stateDemand == STATE_IDLE)) {
                    
                    mutex.lock();
                    
                    targetPositionSetFlag = false;
                    targetPosition = 0;
                    
                    outputPort = 0x0800;    // RESET
                    
                    mutex.unlock();
                    
                    timer.reset();
                    
                    state = STATE_RESET_ALARM;
                }
                
                break;
                
            case STATE_RESET_ALARM:
                
                if (timer > TIMEOUT) {
                    
                    mutex.lock();
                    
                    numericalDataFlag = 0xFFF0; // change all values
                    outputPort = 0x0200;        // SVON on
                    
                    mutex.unlock();
                    
                    timer.reset();
                    
                    state = STATE_SERVO_ON;
                }
                
                break;
                
            case STATE_SERVO_ON:
                
                if (timer > TIMEOUT) {
                    
                    mutex.lock();
                    
                    outputPort = 0x1200;    // SETUP & SVON on
                    
                    mutex.unlock();
                    
                    timer.reset();
                    
                    state = STATE_SETUP;
                }
                
                break;
                
            case STATE_SETUP:
                
                if (timer > TIMEOUT) {
                    
                    if (deviceStatus & DEVICE_STATUS_BUSY) {
                        
                        // wait for setup to complete
                        
                    } else {
                        
                        mutex.lock();
                        
                        outputPort = 0x0200;    // SVON on
                        startFlag = 1;
                        
                        mutex.unlock();
                        
                        timer.reset();
                        
                        state = STATE_IDLE;
                    }
                }
                
                break;
                
            case STATE_IDLE:
                
                if (timer > TIMEOUT) {
                    
                    if (stateDemand == STATE_OFF) {
                        
                        mutex.lock();
                        
                        outputPort = 0x0000;
                        startFlag = 0;
                        
                        mutex.unlock();
                        
                        timer.reset();
                        
                        state = STATE_OFF;
                        
                    } else if (targetPositionSetFlag) {
                        
                        mutex.lock();
                        
                        targetPositionSetFlag = false;
                        targetPosition = targetPositionSet;
                        
                        mutex.unlock();
                        
                        timer.reset();
                        
                        state = STATE_BUSY;
                    }
                }
                
                break;
                
            case STATE_BUSY:
                
                if (timer > TIMEOUT) {
                    
                    if (deviceStatus & DEVICE_STATUS_BUSY) {
                        
                        // wait for motion to complete
                        
                    } else {
                        
                        timer.reset();
                        
                        state = STATE_IDLE;
                    }
                }
                
                break;
                
            default:
                
                state = STATE_OFF;
                
                break;
        }
    }
}
