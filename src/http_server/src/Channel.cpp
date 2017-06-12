/*
 * Channel.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 05.04.2016
 *      Author: Marcel Honegger
 */

#include <http_server/Module.h>
#include <http_server/Channel.h>

using namespace std;

/**
 * Creates a channel object and defines a module this channel belongs to,
 * as well as the index number of this channel on that module.
 * @param module a reference to the module this channel belongs to.
 * @param number the index number of this channel on the module.
 */
Channel::Channel(Module &module, uint16_t number) : module(module)
{

    this->number = number;
    this->name = "";
}

/**
 * Deletes the channel object.
 */
Channel::~Channel() {}

/**
 * Gets the reference to the module this channel belongs to.
 * @return the module this channel belongs to.
 */
Module &Channel::getModule()
{

    return module;
}

/**
 * Gets the index number of this channel.
 * @return the index number of this channel.
 */
uint16_t Channel::getNumber()
{

    return number;
}

/**
 * This method sets a given name to this channel.
 * @param name the name of this channel.
 */
void Channel::setName(string name)
{

    this->name = name;
}

/**
 * Gets the name of this channel.
 * @return the name of this channel.
 */
string Channel::getName()
{

    return name;
}
