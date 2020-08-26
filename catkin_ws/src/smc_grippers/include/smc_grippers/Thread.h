/*
 * Thread.h
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#ifndef THREAD_H_
#define THREAD_H_

#include <cstdlib>
#include <ctime>
#include <cerrno>
#include <csignal>
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

using namespace std;

/**
 * The <code>Thread</code> class allows to install background threads running
 * at low priorities and scheduled with a round-robin algorithm. These threads
 * are intended to be used for state machines, network communication and similar
 * tasks.
 * <br/>
 * Periodic threads running at high priority levels, typically used for control
 * tasks should use the <code>RealtimeThread</code> class instead.
 * <br/>
 * An example of a simple user-defined thread class is given below:
 * <pre><code>
 * class MyThread : public Thread {
 *     public:
 *         void run();
 * };
 *
 * void MyThread::run() {
 *     while (true) {
 *     
 *         <span style="color:#008000">// do something</span>
 *     
 *         sleep(10);   <span style="color:#008000">// pause for 10 ms</span>
 *     }
 * }
 * </code></pre>
 * This thread can then be created, configured and started as follows:
 * <pre><code>
 * MyThread myThread("myThread", 8192);        <span style="color:#008000">// thread with given name and stack size</span>
 * myThread.setPriority(Thread::MIN_PRIORITY); <span style="color:#008000">// priority level</span>
 * myThread.start();
 * ...
 *
 * myThread.join(1000);
 * </code></pre>
 * @see RealtimeThread
 */
class Thread {
    
    public:
        
        static const int32_t MIN_PRIORITY;      /**< Lowest priority level for threads. */
        static const int32_t MAX_PRIORITY;      /**< Highest priority level for threads. */
        
                        Thread();
                        Thread(string name, size_t stackSize);
                        Thread(string name, size_t stackSize, int32_t priority);
        virtual         ~Thread();
        void            setName(string name);
        string          getName();
        int32_t         setStackSize(size_t stackSize);
        void            setPriority(int32_t priority);
        int32_t         getPriority();
        virtual void    start();
        virtual void    run() {};
        bool            isAlive();
        int32_t         join();
        int32_t         join(int32_t millis);
        static void     sleep(int32_t millis);
        static int32_t  currentTimeMillis();
        static int32_t  currentTimeMicros();

    private:
    
        static const size_t     DEFAULT_STACK_SIZE = 64*1024;   // default stack size of thread in [bytes]
        
        pthread_attr_t  threadAttr;
        pthread_t       threadID;
        sched_param     schedParam;
        string          name;
        bool            alive;
        
        static void     runHandler(Thread* thread);
};

#endif /* THREAD_H_ */
