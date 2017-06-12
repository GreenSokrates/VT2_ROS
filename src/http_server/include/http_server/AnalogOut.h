/*
 * AnalogOut.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 28.09.2015
 *      Author: Marcel Honegger
 */

#ifndef ANALOG_OUT_H_
#define ANALOG_OUT_H_

#include <cstdlib>
#include "Channel.h"

class Module;

/**
 * The <code>AnalogOut</code> class is used to write a value into an analog output.
 * This class might either be implemented by a specific device driver, or given
 * a reference to a driver module with the constructor, so that it can write an
 * actual analog output of that module.
 */
class AnalogOut : public Channel {

    public:

                        AnalogOut(Module& module, uint16_t number);
        virtual         ~AnalogOut();
        virtual void    setGain(float gain);
        virtual float   getGain();
        virtual void    setOffset(float offset);
        virtual float   getOffset();
        virtual void    write(float value);
        AnalogOut&      operator=(float value);
        virtual float   read();
                        operator float();
        
    protected:
        
        float           gain;
        float           offset;
        float           value;
};

#endif /* ANALOG_OUT_H_ */
