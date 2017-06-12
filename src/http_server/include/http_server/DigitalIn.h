/*
 * DigitalIn.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 25.09.2015
 *      Author: Marcel Honegger
 */

#ifndef DIGITAL_IN_H_
#define DIGITAL_IN_H_

#include <cstdlib>
#include "Channel.h"

class Module;

/**
 * The <code>DigitalIn</code> class is used to configure and read a digital input.
 * This class might either be implemented by a specific device driver, or given
 * a reference to a driver module with the constructor, so that it can read an
 * actual digital input from that module.
 */
class DigitalIn : public Channel {
    
    public:
        
                        DigitalIn(Module& module, uint16_t number);
        virtual			~DigitalIn();
        virtual void	inversePolarity(bool polarity);
        virtual bool    read();
						operator bool();
        
    protected:
        
        bool            polarity;
};

#endif /* DIGITAL_IN_H_ */
