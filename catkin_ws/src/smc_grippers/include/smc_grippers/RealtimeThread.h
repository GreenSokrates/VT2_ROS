/*
 * RealtimeThread.h
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#ifndef REALTIME_THREAD_H_
#define REALTIME_THREAD_H_

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cerrno>
#include <csignal>
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#if defined __QNX__

#include <sys/syspage.h>
#include <sys/neutrino.h>

#else

#include <sys/time.h>
#include <sys/timerfd.h>

#endif

using namespace std;

/**
 * The <code>RealtimeThread</code> class allows to install periodic, high-priority tasks.
 * An example of a simple user-defined realtime thread class is given below:
 * <pre><code>
 * class MyThread : public RealtimeThread {
 *     public:
 *         void run();
 * };
 *
 * void MyThread::run() {
 *     while (waitForNextPeriod()) {
 *     
 *         <span style="color:#008000">// code to be executed periodically</span>
 *     }
 * }
 * </code></pre>
 * This realtime thread can then be created, configured and started as follows:
 * <pre><code>
 * MyThread myThread("myThread", 8192);                   <span style="color:#008000">// thread with given name and stack size</span>
 * myThread.setPriority(RealtimeThread::RT_MAX_PRIORITY); <span style="color:#008000">// priority level</span>
 * myThread.setPeriod(0.005);                             <span style="color:#008000">// period in seconds</span>
 * myThread.start();
 * ...
 *
 * myThread.stop();
 * </code></pre>
 * @see Thread
 */
class RealtimeThread {
    
    public:
        
        static const int32_t RT_MIN_PRIORITY;   /**< Lowest priority level for realtime threads. */
        static const int32_t RT_MAX_PRIORITY;   /**< Highest priority level for realtime threads. */
        
                        RealtimeThread();
                        RealtimeThread(string name, size_t stackSize);
                        RealtimeThread(string name, size_t stackSize, int32_t priority, double period);
        virtual         ~RealtimeThread();
        void            setName(string name);
        string          getName();
        int32_t         setStackSize(size_t stackSize);
        void            setPriority(int32_t priority);
        int32_t         getPriority();
        void            setPeriod(double period);
        double          getPeriod();
        bool            isAlive();
        virtual void    start();
        virtual void    stop();
        virtual void    run() {};
        bool            waitForNextPeriod();
        int32_t         join();
        int32_t         join(int32_t millis);
        string          toString();
    
    private:
        
        static const size_t     DEFAULT_STACK_SIZE = 64*1024;   // default stack size of thread in [bytes]
        static const double     DELAY;                          // delay for collecting statistical info in [s]
    
        static pthread_mutex_t  mutex;
        static bool             signalsInitialized;
        static bool*            signalNumbers;
        
        pthread_attr_t  threadAttr;
        pthread_t       threadID;
        sched_param     schedParam;
        string          name;
        int32_t         threadState;
        
        #if defined __QNX__
        
        uint32_t        signalNumber;
        sigevent        signalEvent;
        timer_t         signalTimer;
        sigset_t        signalsToCatch;
        
        #else
        
        int32_t         timerFD;
        
        #endif
        
        uint64_t        runs;
        uint64_t        overruns;
        uint64_t        freeRuns;
        double          releaseTime;
        double          period;
        double          periodActual;
        double          periodMin;
        double          periodMax;
        double          periodMean;
        double          periodDeviation;
        double          durationActual;
        double          durationMin;
        double          durationMax;
        double          durationMean;
        double          durationDeviation;
        
        static uint32_t getSignalNumber();
        static void     runHandler(RealtimeThread* realtimeThread);
};

#endif /* REALTIME_THREAD_H_ */
