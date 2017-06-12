/*
 * AnalogIn.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 28.09.2015
 *      Author: Marcel Honegger
 */

#ifndef ANALOG_IN_H_
#define ANALOG_IN_H_

#include <cstdlib>
#include "Channel.h"

class Module;

/**
 * The <code>AnalogIn</code> class is used to read the value of an analog input.
 * This class might either be implemented by a specific device driver, or given
 * a reference to a driver module with the constructor, so that it can read an
 * actual analog input from that module.
 */
class AnalogIn : public Channel {

    public:
        
                        AnalogIn(Module& module, uint16_t number);
        virtual         ~AnalogIn();
        virtual void    setGain(float gain);
        virtual float   getGain();
        virtual void    setOffset(float offset);
        virtual float   getOffset();
        virtual float   read();
                        operator float();
        
    protected:
        
        float           gain;
        float           offset;
};

#endif /* ANALOG_IN_H_ */
