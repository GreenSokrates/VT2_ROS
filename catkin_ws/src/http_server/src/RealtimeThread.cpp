/*
 * RealtimeThread.cpp
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#include <http_server/RealtimeThread.h>

using namespace std;

const int32_t RealtimeThread::RT_MIN_PRIORITY = (sched_get_priority_min(SCHED_FIFO) + sched_get_priority_max(SCHED_FIFO)) / 2 + 1;
const int32_t RealtimeThread::RT_MAX_PRIORITY = sched_get_priority_max(SCHED_FIFO) - 1;

const double RealtimeThread::DELAY = 10.0; // delay for collecting statistical information in [s]

#if defined __QNX__

pthread_mutex_t RealtimeThread::mutex = PTHREAD_MUTEX_INITIALIZER;
bool RealtimeThread::signalsInitialized = false;
bool *RealtimeThread::signalNumbers = new bool[SIGRTMAX - SIGRTMIN + 1];

#endif

/**
 * Creates a realtime thread object with default name, stack size, priority and period.
 * The default priority is <code>RT_MAX_PRIORITY</code> and the default period 1.0 second.
 */
RealtimeThread::RealtimeThread()
{

#if defined __QNX__

    if (!RealtimeThread::signalsInitialized)
    {

        pthread_mutex_init(&RealtimeThread::mutex, NULL);
        for (int32_t i = 0; i < SIGRTMAX - SIGRTMIN + 1; i++)
            RealtimeThread::signalNumbers[i] = false;
        RealtimeThread::signalsInitialized = true;
    }

    sigset_t signalsToBlock;
    sigemptyset(&signalsToBlock);
    for (int32_t i = SIGRTMIN; i <= SIGRTMAX; i++)
        sigaddset(&signalsToBlock, i);
    pthread_sigmask(SIG_BLOCK, &signalsToBlock, NULL);

    signalNumber = getSignalNumber();

#endif

    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, DEFAULT_STACK_SIZE);
    pthread_attr_setinheritsched(&threadAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&threadAttr, SCHED_FIFO);
    pthread_attr_getschedparam(&threadAttr, &schedParam);
    schedParam.sched_priority = RT_MAX_PRIORITY;
    pthread_attr_setschedparam(&threadAttr, &schedParam);

    threadState = 0;
    name = "RealtimeThread";
    period = 1.0;
}

/**
 * Creates a realtime thread object with given name and stack size.
 * @param name the name of this realtime thread.
 * @param stackSize the desired size of the stack, given in [bytes].
 */
RealtimeThread::RealtimeThread(string name, size_t stackSize)
{

#if defined __QNX__

    if (!RealtimeThread::signalsInitialized)
    {

        pthread_mutex_init(&RealtimeThread::mutex, NULL);
        for (int32_t i = 0; i < SIGRTMAX - SIGRTMIN + 1; i++)
            RealtimeThread::signalNumbers[i] = false;
        RealtimeThread::signalsInitialized = true;
    }

    sigset_t signalsToBlock;
    sigemptyset(&signalsToBlock);
    for (int32_t i = SIGRTMIN; i <= SIGRTMAX; i++)
        sigaddset(&signalsToBlock, i);
    pthread_sigmask(SIG_BLOCK, &signalsToBlock, NULL);

    signalNumber = getSignalNumber();

#endif

    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, stackSize);
    pthread_attr_setinheritsched(&threadAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&threadAttr, SCHED_FIFO);
    pthread_attr_getschedparam(&threadAttr, &schedParam);
    schedParam.sched_priority = RT_MAX_PRIORITY;
    pthread_attr_setschedparam(&threadAttr, &schedParam);

    threadState = 0;
    this->name = name;
    period = 1.0;
}

/**
 * Creates a realtime thread object with given name, priority and period.
 * @param name the name of this realtime thread.
 * @param stackSize the desired size of the stack, given in [bytes].
 * @param priority the priority of this thread, see <code>setPriority()</code>.
 * @param period the period of this thread, given in seconds.
 */
RealtimeThread::RealtimeThread(string name, size_t stackSize, int32_t priority, double period)
{

#if defined __QNX__

    if (!RealtimeThread::signalsInitialized)
    {

        pthread_mutex_init(&RealtimeThread::mutex, NULL);
        for (int32_t i = 0; i < SIGRTMAX - SIGRTMIN + 1; i++)
            RealtimeThread::signalNumbers[i] = false;
        RealtimeThread::signalsInitialized = true;
    }

    sigset_t signalsToBlock;
    sigemptyset(&signalsToBlock);
    for (int32_t i = SIGRTMIN; i <= SIGRTMAX; i++)
        sigaddset(&signalsToBlock, i);
    pthread_sigmask(SIG_BLOCK, &signalsToBlock, NULL);

    signalNumber = getSignalNumber();

#endif

    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, stackSize);
    pthread_attr_setinheritsched(&threadAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&threadAttr, SCHED_FIFO);
    pthread_attr_getschedparam(&threadAttr, &schedParam);

    if (priority < RT_MIN_PRIORITY)
        priority = RT_MIN_PRIORITY;
    else if (priority > RT_MAX_PRIORITY)
        priority = RT_MAX_PRIORITY;

    schedParam.sched_priority = priority;
    pthread_attr_setschedparam(&threadAttr, &schedParam);

    threadState = 0;
    this->name = name;
    this->period = period;
}

RealtimeThread::~RealtimeThread()
{

    pthread_cancel(threadID);
    pthread_attr_destroy(&threadAttr);

#if defined __QNX__

    RealtimeThread::signalNumbers[signalNumber - SIGRTMIN] = false;

#endif
}

/**
 * Sets the size of this realtime thread's stack to a given number of bytes.
 * @param stackSize the desired size of the stack, given in [bytes].
 * @return an error number, or 0 if no error occured.
 */
int RealtimeThread::setStackSize(size_t stackSize)
{

    return pthread_attr_setstacksize(&threadAttr, stackSize);
}

/**
 * Sets the priority level of this realtime thread.
 * This priority has to be at least <code>RT_MIN_PRIORITY</code> and at most
 * <code>RT_MAX_PRIORITY</code>. To set or change the priority level, this
 * method must be called before the thread is started.
 * @param priority the priority level of this realtime thread.
 */
void RealtimeThread::setPriority(int32_t priority)
{

    if (priority < RT_MIN_PRIORITY)
        priority = RT_MIN_PRIORITY;
    else if (priority > RT_MAX_PRIORITY)
        priority = RT_MAX_PRIORITY;

    schedParam.sched_priority = priority;
    pthread_attr_setschedparam(&threadAttr, &schedParam);
}

/**
 * Gets the priority level of this realtime thread.
 * @return the priority level.
 */
int32_t RealtimeThread::getPriority()
{

    return schedParam.sched_priority;
}

/**
 * Sets the period of this realtime thread.
 * To set or change the period, this method must be called before the
 * thread is started.
 * @param period the period of this thread, given in seconds.
 */
void RealtimeThread::setPeriod(double period)
{

    this->period = period;
}

/**
 * Gets the period of this realtime thread.
 * @return the period in seconds.
 */
double RealtimeThread::getPeriod()
{

    return period;
}

/**
 * Sets the name of this realtime thread object.
 * @param name the name of this thread.
 */
void RealtimeThread::setName(string name)
{

    this->name = name;
}

/**
 * Gets the name of this realtime thread object.
 * @return the name of this thread as a string.
 */
string RealtimeThread::getName()
{

    return name;
}

/**
 * Tests if this realtime thread is alive.
 * @return <code>true</code> if this realtime thread is alive;
 * <code>false</code> otherwise.
 */
bool RealtimeThread::isAlive()
{

    return (threadState == 1) ? true : false;
}

/**
 * Starts this realtime thread.
 */
void RealtimeThread::start()
{

    if (threadState == 0)
    {

        threadState = 1;

#if defined __QNX__

        signalEvent.sigev_notify = SIGEV_SIGNAL;
        signalEvent.sigev_signo = signalNumber;
        signalEvent.sigev_value.sival_ptr = &signalTimer;

        timer_create(CLOCK_REALTIME, &signalEvent, &signalTimer);

        itimerspec timerSpec;

        uint64_t period = static_cast<uint64_t>(1.0e9 * this->period);
        timerSpec.it_interval.tv_sec = period / 1000000000;
        timerSpec.it_interval.tv_nsec = period % 1000000000;
        timerSpec.it_value = timerSpec.it_interval;

        timer_settime(signalTimer, 0, &timerSpec, NULL);

        sigemptyset(&signalsToCatch);
        sigaddset(&signalsToCatch, signalNumber);

#else

        timerFD = timerfd_create(CLOCK_MONOTONIC, 0);

        itimerspec timerSpec;

        long period = 1.0e9 * this->period;
        timerSpec.it_interval.tv_sec = period / 1000000000;
        timerSpec.it_interval.tv_nsec = period % 1000000000;
        timerSpec.it_value = timerSpec.it_interval;

        timerfd_settime(timerFD, 0, &timerSpec, NULL);

#endif

        runs = 0;
        overruns = 0;
        freeRuns = max(static_cast<uint64_t>(DELAY / period), static_cast<uint64_t>(1));
        releaseTime = 0.0;
        periodActual = period;
        periodMin = 1.0e9;
        periodMax = 0.0;
        periodMean = period;
        periodDeviation = 0.0;
        durationActual = 0.0;
        durationMin = 1.0e9;
        durationMax = 0.0;
        durationMean = 0.0;
        durationDeviation = 0.0;

        pthread_create(&threadID, &threadAttr, (void *(*)(void *))RealtimeThread::runHandler, this);
    }
}

/**
 * Stops this realtime thread.
 */
void RealtimeThread::stop()
{

    if (threadState == 1)
    {

        threadState = 0;

        pthread_join(threadID, NULL);

#if defined __QNX__

        timer_delete(signalTimer);

#else

        close(timerFD);

#endif
    }
}

/**
 * Blocks the thread for the duration of one period.
 * @return <code>false</code> if the <code>stop()</code> method was called,
 * <code>true</code> otherwise.
 */
bool RealtimeThread::waitForNextPeriod()
{

    if (threadState == 0)
        return false;

#if defined __QNX__

    uint64_t cycles = ClockCycles();
    uint64_t cyclesPerSec = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    double time0 = static_cast<double>(cycles) / static_cast<double>(cyclesPerSec);

    //timespec currentTime;
    //clock_gettime(CLOCK_REALTIME, &currentTime);
    //double time0 = static_cast<double>(currentTime.tv_sec+currentTime.tv_nsec/1.0e9);

    sigevent caughtSignal;
    sigwait(&signalsToCatch, (int32_t *)&caughtSignal);

    cycles = ClockCycles();
    double time1 = static_cast<double>(cycles) / static_cast<double>(cyclesPerSec);

//clock_gettime(CLOCK_REALTIME, &currentTime);
//double time1 = static_cast<double>(currentTime.tv_sec+currentTime.tv_nsec/1.0e9);

#else

    timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    double time0 = static_cast<double>(currentTime.tv_sec + currentTime.tv_nsec / 1.0e9);

    uint64_t missed;
    read(timerFD, &missed, sizeof(missed));

    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    double time1 = static_cast<double>(currentTime.tv_sec + currentTime.tv_nsec / 1.0e9);

#endif

    double period = time1 - releaseTime;
    double duration = time0 - releaseTime;
    releaseTime = time1;

    runs++;
    if (runs > freeRuns)
    {
        overruns += (period > 2.0 * this->period) ? 1 : 0;
        periodActual = period;
        periodMin = (period < periodMin) ? period : periodMin;
        periodMax = (period > periodMax) ? period : periodMax;
        periodMean = (((double)runs - (double)freeRuns - 1.0) * periodMean + period) / (double)(runs - freeRuns);
        periodDeviation = sqrt((periodDeviation * periodDeviation * ((double)runs - (double)freeRuns - 1.0) + (period - periodMean) * (period - periodMean)) / (double)(runs - freeRuns));
        durationActual = duration;
        durationMin = (duration < durationMin) ? duration : durationMin;
        durationMax = (duration > durationMax) ? duration : durationMax;
        durationMean = ((runs - freeRuns - 1) * durationMean + duration) / (runs - freeRuns);
        durationDeviation = sqrt((durationDeviation * durationDeviation * ((double)runs - (double)(freeRuns + 1)) + (duration - durationMean) * (duration - durationMean)) / ((double)runs - (double)freeRuns));
    }

    return true;
}

/**
 * Waits until this realtime thread is no longer alive.
 */
int32_t RealtimeThread::join()
{

    return pthread_join(threadID, NULL);
}

/**
 * Waits until this realtime thread is no longer alive.
 * @param millis a timeout given in [ms].
 */
int32_t RealtimeThread::join(int32_t millis)
{

#if defined __QNX__

    timespec timeout;
    timeout.tv_sec = millis / 1000;
    timeout.tv_nsec = (millis % 1000) * 1000000;

    return pthread_timedjoin(threadID, NULL, &timeout);

#else

    return pthread_join(threadID, NULL);

#endif
}

/**
 * Returns a string representation of this realtime thread.
 * @return a string representation.
 */
string RealtimeThread::toString()
{

    stringstream out;

    out << name << ":" << endl;
    out << "  priority: " << getPriority() << endl;

    if (isAlive() && (runs > freeRuns))
    {
        out << "  period: " << static_cast<int32_t>(periodActual * 1.0e6) << " (" << static_cast<int32_t>(period * 1.0e6) << ") us" << endl;
        out << "    min/max: " << static_cast<int32_t>(periodMin * 1.0e6) << "/" << static_cast<int32_t>(periodMax * 1.0e6) << " us" << endl;
        out << "    mean: " << static_cast<int32_t>(periodMean * 1.0e6) << " us" << endl;
        out << "    deviation: " << static_cast<int32_t>(periodDeviation * 1.0e6) << " us" << endl;
        out << "    overruns: " << overruns;
        if (overruns > 0)
            out << " (1:" << static_cast<int32_t>((double)(runs - 100) / (double)overruns) << ")" << endl;
        else
            out << endl;
        out << "  duration: " << static_cast<int32_t>(durationActual * 1.0e6) << " us" << endl;
        out << "    min/max: " << static_cast<int32_t>(durationMin * 1.0e6) << "/" << static_cast<int32_t>(durationMax * 1.0e6) << " us" << endl;
        out << "    mean: " << static_cast<int32_t>(durationMean * 1.0e6) << " us" << endl;
        out << "    deviation: " << static_cast<int32_t>(durationDeviation * 1.0e6) << " us" << endl;
    }
    else
    {
        out << "  period: " << static_cast<int32_t>(period * 1.0e6) << " us" << endl;
        out << "  duration: -" << endl;
    }

    return out.str();
}

uint32_t RealtimeThread::getSignalNumber()
{

#if defined __QNX__

    pthread_mutex_lock(&RealtimeThread::mutex);

    uint32_t currentNumber = 0;

    while (RealtimeThread::signalNumbers[currentNumber] && (currentNumber < SIGRTMAX - SIGRTMIN))
        currentNumber++;
    RealtimeThread::signalNumbers[currentNumber] = true;

    pthread_mutex_unlock(&RealtimeThread::mutex);

    return SIGRTMIN + currentNumber;

#else

    return 0;

#endif
}

void RealtimeThread::runHandler(RealtimeThread *realtimeThread)
{

    realtimeThread->run();
}
