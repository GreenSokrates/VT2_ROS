/*
 * Thread.cpp
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#include "smc_grippers/Thread.h"

using namespace std;

const int32_t Thread::MIN_PRIORITY = sched_get_priority_min(SCHED_RR)+1;
const int32_t Thread::MAX_PRIORITY = (sched_get_priority_min(SCHED_RR)+sched_get_priority_max(SCHED_RR))/2;

/**
 * Creates a thread object with default name, stack size and priority.
 * The default priority is the same as the priority of the
 * thread creating this thread.
 */
Thread::Thread() {
    
    sigset_t signalsToBlock;
    sigemptyset(&signalsToBlock);
    for (int32_t i = SIGRTMIN; i <= SIGRTMAX; i++) sigaddset(&signalsToBlock, i);
    pthread_sigmask(SIG_BLOCK, &signalsToBlock, NULL);
    
    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, DEFAULT_STACK_SIZE);
    pthread_attr_setinheritsched(&threadAttr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setschedpolicy(&threadAttr, SCHED_RR);
    pthread_attr_getschedparam(&threadAttr, &schedParam);
    
    if (schedParam.sched_priority < MIN_PRIORITY) schedParam.sched_priority = MIN_PRIORITY;
    else if (schedParam.sched_priority > MAX_PRIORITY) schedParam.sched_priority = MAX_PRIORITY;
    
    pthread_attr_setschedparam(&threadAttr, &schedParam);
    
    name = "Thread";
    
    alive = false;
}

/**
 * Creates a thread object with given name and stack size.
 * @param name the name of this thread.
 * @param stackSize the desired size of the stack, given in [bytes].
 */
Thread::Thread(std::string name, size_t stackSize) {
    
    sigset_t signalsToBlock;
    sigemptyset(&signalsToBlock);
    for (int32_t i = SIGRTMIN; i <= SIGRTMAX; i++) sigaddset(&signalsToBlock, i);
    pthread_sigmask(SIG_BLOCK, &signalsToBlock, NULL);
    
    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, stackSize);
    pthread_attr_setinheritsched(&threadAttr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setschedpolicy(&threadAttr, SCHED_RR);
    pthread_attr_getschedparam(&threadAttr, &schedParam);
    
    if (schedParam.sched_priority < MIN_PRIORITY) schedParam.sched_priority = MIN_PRIORITY;
    else if (schedParam.sched_priority > MAX_PRIORITY) schedParam.sched_priority = MAX_PRIORITY;
    
    pthread_attr_setschedparam(&threadAttr, &schedParam);
    
    this->name = name;
    
    alive = false;
}

/**
 * Creates a thread object with given name, stack size and priority.
 * @param name the name of this thread.
 * @param stackSize the desired size of the stack, given in [bytes].
 * @param priority the priority of this thread, see <code>setPriority()</code>.
 */
Thread::Thread(std::string name, size_t stackSize, int32_t priority) {
    
    sigset_t signalsToBlock;
    sigemptyset(&signalsToBlock);
    for (int32_t i = SIGRTMIN; i <= SIGRTMAX; i++) sigaddset(&signalsToBlock, i);
    pthread_sigmask(SIG_BLOCK, &signalsToBlock, NULL);
    
    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, stackSize);
    pthread_attr_setinheritsched(&threadAttr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setschedpolicy(&threadAttr, SCHED_RR);
    pthread_attr_getschedparam(&threadAttr, &schedParam);
    
    if (priority < MIN_PRIORITY) priority = MIN_PRIORITY;
    else if (priority > MAX_PRIORITY) priority = MAX_PRIORITY;
    
    schedParam.sched_priority = priority;
    pthread_attr_setschedparam(&threadAttr, &schedParam);
    
    this->name = name;
    
    alive = false;
}

Thread::~Thread() {
    
    pthread_cancel(threadID);
    pthread_attr_destroy(&threadAttr);
}

/**
 * Sets the name of this thread object.
 * @param name the name of this thread.
 */
void Thread::setName(string name) {
    
    this->name = name;
}

/**
 * Gets the name of this thread object.
 * @return the name of this thread as a string.
 */
string Thread::getName() {
    
    return name;
}

/**
 * Sets the size of this thread's stack to a given number of bytes.
 * @param stackSize the desired size of the stack, given in [bytes].
 * @return an error number, or 0 if no error occured.
 */
int32_t Thread::setStackSize(size_t stackSize) {
    
    return pthread_attr_setstacksize(&threadAttr, stackSize);
}

/**
 * Sets the priority level of this thread.
 * This priority has to be at least <code>MIN_PRIORITY</code> and at most
 * <code>MAX_PRIORITY</code>. To set or change the priority level, this
 * method must be called before the thread is started.
 * @param priority the priority of this thread.
 */
void Thread::setPriority(int32_t priority) {
    
    if (priority < MIN_PRIORITY) priority = MIN_PRIORITY;
    else if (priority > MAX_PRIORITY) priority = MAX_PRIORITY;
    
    schedParam.sched_priority = priority;
    pthread_attr_setschedparam(&threadAttr, &schedParam);
}

/**
 * Gets the priority level of this thread.
 * @return the priority level.
 */
int32_t Thread::getPriority() {
    
    return schedParam.sched_priority;
}

/**
 * Starts this thread.
 */
void Thread::start() {
    
    alive = true;
    pthread_create(&threadID, &threadAttr, (void*(*)(void*))Thread::runHandler, this);
}

/**
 * Checks if this thread is running.
 * @return <code>true</code> if this thread is running, <code>false</code>otherwise.
 */
bool Thread::isAlive() {
    
    return alive;
}

/**
 * Waits until this thread is no longer running.
 */
int32_t Thread::join() {
    
    return pthread_join(threadID, NULL);
}

/**
 * Waits until this thread is no longer running.
 * @param millis a timeout given in [ms].
 */
int32_t Thread::join(int32_t millis) {
    
    #if defined __QNX__
    
    timespec timeout;
    timeout.tv_sec = millis/1000;
    timeout.tv_nsec = (millis%1000)*1000000;
    
    return pthread_timedjoin(threadID, NULL, &timeout);
    
    #else
    
    return pthread_join(threadID, NULL);
    
    #endif
}

/**
 * Suspends the execution of this thread for a given duration.
 * @param millis the duration given in [ms].
 */
void Thread::sleep(int32_t millis) {
    
    int32_t error = 0; errno = 0;
    timespec requestedTime, remainingTime;
    
    remainingTime.tv_sec = millis/1000;
    remainingTime.tv_nsec = (millis%1000)*1000000;
    
    do {
        requestedTime.tv_sec = remainingTime.tv_sec;
        requestedTime.tv_nsec = remainingTime.tv_nsec;
        error = nanosleep(&requestedTime, &remainingTime);
    } while ((error != 0) && (errno == EINTR));
}

/**
 * Gets the current time in milliseconds.
 * @return the current time in [ms].
 */
int32_t Thread::currentTimeMillis() {
    
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    
    return static_cast<int32_t>(currentTime.tv_sec*1000+currentTime.tv_usec/1000);
}

/**
 * Gets the current time in microseconds.
 * @return the current time in [us].
 */
int32_t Thread::currentTimeMicros() {
    
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    
    return static_cast<int32_t>(currentTime.tv_sec*1000000+currentTime.tv_usec);
}

void Thread::runHandler(Thread* thread) {
    
    thread->run();
    thread->alive = false;
}
