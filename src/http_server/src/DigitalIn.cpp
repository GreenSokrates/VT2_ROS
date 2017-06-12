/*
 * DigitalIn.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 25.09.2015
 *      Author: Marcel Honegger
 */

#include <http_server/Module.h>
#include <http_server/DigitalIn.h>

using namespace std;

/**
 * Creates a digital input object and defines a module this input belongs to,
 * as well as the index number of this input on that module.
 * @param module a reference to the module this digital input belongs to.
 * @param number the index number of this digital input on the module.
 */
DigitalIn::DigitalIn(Module &module, uint16_t number) : Channel(module, number)
{

    polarity = false;

    module.configureDigitalIn(number);
}

/**
 * Deletes the digital input object.
 */
DigitalIn::~DigitalIn() {}

/**
 * Sets the polarity of the input.
 * @param polarity a boolean to define the polarity, <code>true</code> inverts the
 * polarity, <code>false</code> corresponds to the normal polarity of the input.
 */
void DigitalIn::inversePolarity(bool polarity)
{

    this->polarity = polarity;
}

/**
 * Gets the input state, represented as <code>true</code> or <code>false</code>.
 * @return a boolean representing the input state, <code>true</code> for a
 * logical 1 and <code>false</code> for a logical 0.
 */
bool DigitalIn::read()
{

    return polarity != module.readDigitalIn(number);
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
DigitalIn::operator bool()
{

    return read();
}
