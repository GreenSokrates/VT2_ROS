/*
 * DigitalOut.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 25.09.2015
 *      Author: Marcel Honegger
 */

#ifndef DIGITAL_OUT_H_
#define DIGITAL_OUT_H_

#include <cstdlib>
#include "Channel.h"

class Module;

/**
 * The <code>DigitalOut</code> class is used to configure and control a digital output.
 * This class might either be implemented by a specific device driver, or given
 * a reference to a driver module with the constructor, so that it can write an
 * actual digital output of that module.
 */
class DigitalOut : public Channel {

    public:

                        DigitalOut(Module& module, uint16_t number);
        virtual         ~DigitalOut();
        virtual void    inversePolarity(bool polarity);
        virtual void    write(bool value);
        DigitalOut&     operator=(bool value);
        virtual bool    read();
                        operator bool();
        
    protected:
        
        bool            polarity;
        bool            value;
};

#endif /* DIGITAL_OUT_H_ */
