/*
 * Mutex.cpp
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 08.02.2017
 *      Author: Marcel Honegger
 */

#include "smc_grippers/Mutex.h"

using namespace std;

/**
 * Create and initialize a mutex object.
 */
Mutex::Mutex() {
    
    pthread_mutex_init(&mutex, NULL);
}

/**
 * Delete the mutex object.
 */
Mutex::~Mutex() {
    
    pthread_mutex_destroy(&mutex);
}

/**
 * Wait until a mutex becomes available.
 */
void Mutex::lock() {
    
    pthread_mutex_lock(&mutex);
}

/**
 * Unlock the mutex that has previously been locked by the same thread.
 */
void Mutex::unlock() {
    
    pthread_mutex_unlock(&mutex);
}
