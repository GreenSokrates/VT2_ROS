/*
 * DigitalOut.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 25.09.2015
 *      Author: Marcel Honegger
 */

#include <http_server/Module.h>
#include <http_server/DigitalOut.h>

using namespace std;

/**
 * Creates a digital output object and defines a module this output belongs to,
 * as well as the index number of this output on that module.
 * @param module a reference to the module this digital output belongs to.
 * @param number the index number of this digital output on the module.
 */
DigitalOut::DigitalOut(Module &module, uint16_t number) : Channel(module, number)
{

    polarity = false;
    value = false;

    module.configureDigitalOut(number);
}

/**
 * Deletes the digital output object.
 */
DigitalOut::~DigitalOut() {}

/**
 * Sets the polarity of the output.
 * @param polarity a boolean to define the polarity, <code>true</code> inverts the
 * polarity, <code>false</code> corresponds to the normal polarity of the output.
 */
void DigitalOut::inversePolarity(bool polarity)
{

    this->polarity = polarity;
}

/**
 * Writes the output, specified as <code>true</code> or <code>false</code>.
 * @param value a boolean specifying the output value, <code>true</code>
 * for a logical 1 and <code>false</code> for a logical 0.
 */
void DigitalOut::write(bool value)
{

    this->value = value;

    module.writeDigitalOut(number, polarity != value);
}

/**
 * The '=' operator is a shorthand notation of the <code>write()</code> method.
 */
DigitalOut &DigitalOut::operator=(bool value)
{

    write(value);

    return *this;
}

/**
 * Reads the output back, represented as <code>true</code> or <code>false</code>.
 * @return a boolean representing the output, <code>true</code> for a logical 1
 * and <code>false</code> for a logical 0.
 */
bool DigitalOut::read()
{

    return value;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
DigitalOut::operator bool()
{

    return read();
}
