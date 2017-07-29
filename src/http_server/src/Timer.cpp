/*
 * Timer.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 18.11.2015
 *      Author: Marcel Honegger
 */

#include <http_server/Timer.h>

using namespace std;

/**
 * Creates a timer object.
 */
Timer::Timer() : RealtimeThread("Timer", STACK_SIZE, RealtimeThread::RT_MIN_PRIORITY, 0.001)
{

    // initialize local values

    time = 0;
    running = false;

    // start timer thread

    RealtimeThread::start();
}

/**
 * Deletes the timer object.
 */
Timer::~Timer()
{

    RealtimeThread::stop();
}

/**
 * Starts the timer.
 */
void Timer::start()
{

    running = true;
}

/**
 * Stops the timer.
 */
void Timer::stop()
{

    running = false;
}

/**
 * Resets the timer to zero.
 */
void Timer::reset()
{

    time = 0;
}

/**
 * Gets the actual time of this timer in [ms].
 */
uint32_t Timer::read()
{

    return time;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
Timer::operator uint32_t()
{

    return read();
}

/**
 * Implements the run logic of the timer.
 */
void Timer::run()
{

    while (waitForNextPeriod())
    {

        if (running && (time < UINT32_MAX))
            time++;
    }
}
