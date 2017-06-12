/*
 * LowpassFilter.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.11.2015
 *      Author: Marcel Honegger
 */

#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#include <cstdlib>
#include <cmath>

/**
 * This class implements a time-discrete 2nd order lowpass filter for a series of data values.
 * This filter can typically be used within a periodic task that takes measurements that need
 * to be filtered, like speed or position values.
 */
class LowpassFilter {
    
    public:
    
                LowpassFilter();
        virtual ~LowpassFilter();
        void    reset();
        void    reset(double value);
        void    setPeriod(double period);
        void    setFrequency(double frequency);
        double  getFrequency();
        double  filter(double value);
        
    private:
        
        double  period;
        double  frequency;
        double  a11, a12, a21, a22, b1, b2;
        double  x1, x2;
};

#endif /* LOWPASS_FILTER_H_ */
