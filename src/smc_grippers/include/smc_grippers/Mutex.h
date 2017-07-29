/*
 * Mutex.h
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 08.02.2017
 *      Author: Marcel Honegger
 */

#ifndef MUTEX_H_
#define MUTEX_H_

#include <cstdlib>
#include <pthread.h>

/**
 * The Mutex class is used to synchronise the execution of threads.
 * This is for example used to protect access to a shared resource.
 */
class Mutex {
    
    public:
        
                    Mutex();
        virtual     ~Mutex();
        void        lock();
        void        unlock();
        
    private:
        
        pthread_mutex_t mutex;
};

#endif /* MUTEX_H_ */
