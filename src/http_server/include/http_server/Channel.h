/*
 * Channel.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 05.04.2016
 *      Author: Marcel Honegger
 */

#ifndef CHANNEL_H_
#define CHANNEL_H_

#include <cstdlib>
#include <string>
#include <stdint.h>

using namespace std;

class Module;

/**
 * This class is the abstract superclass for all periphery channels, like digital input
 * or analog output channels. It offers methods that are required by all types of channels.
 */
class Channel {
    
    public:
        
                    Channel(Module& module, uint16_t number);
        virtual     ~Channel();
        Module&     getModule();
        uint16_t    getNumber();
        void        setName(string name);
        string      getName();
        
    protected:
        
        Module&     module;     // referece to a module this channel belongs to
        uint16_t    number;     // index number of this channel on the module
        string      name;       // name of this channel
};

#endif /* CHANNEL_H_ */
