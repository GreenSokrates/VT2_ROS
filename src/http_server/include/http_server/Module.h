/*
 * Module.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 05.04.2016
 *      Author: Marcel Honegger
 */

#ifndef MODULE_H_
#define MODULE_H_

#include <cstdlib>
#include <stdint.h>

/**
 * This class is the abstract superclass for all periphery modules, like industrial input
 * and output boards offering digital or analog input and output channels. It offers methods
 * that are required by all specific module device drivers.
 */
class Module {
    
    public:
                
                        Module();
        virtual         ~Module();
        virtual void    configureAnalogIn(uint16_t number);
        virtual void    configureAnalogOut(uint16_t number);
        virtual void    configureDigitalIn(uint16_t number);
        virtual void    configureDigitalOut(uint16_t number);
        virtual float   readAnalogIn(uint16_t number);
        virtual void    writeAnalogOut(uint16_t number, float value);
        virtual bool    readDigitalIn(uint16_t number);
        virtual void    writeDigitalOut(uint16_t number, bool value);
};

#endif /* MODULE_H_ */
