/*
 * Timer.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 18.11.2015
 *      Author: Marcel Honegger
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <cstdlib>
#include <stdint.h>
#include "RealtimeThread.h"

/**
 * The <code>Timer</code> class implements a simple timer counting milliseconds.
 */
class Timer : public RealtimeThread {
    
    public:
    
                    Timer();
        virtual     ~Timer();
        void        start();
        void        stop();
        void        reset();
        uint32_t    read();
                    operator uint32_t();
        
    private:
        
        static const size_t STACK_SIZE = 8*1024;    // stack size of thread in [bytes]
        
        uint32_t    time;
        bool        running;
        
        void        run();
};

#endif /* TIMER_H_ */
