/*
 * AnalogOut.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 28.09.2015
 *      Author: Marcel Honegger
 */

#include <http_server/Module.h>
#include <http_server/AnalogOut.h>

using namespace std;

/**
 * Creates an analog output object and defines a module this output belongs to,
 * as well as the index number of this output on that module.
 * @param module a reference to the module this analog output belongs to.
 * @param number the index number of this analog output on the module.
 */
AnalogOut::AnalogOut(Module &module, uint16_t number) : Channel(module, number)
{

    gain = 1.0f;
    offset = 0.0f;
    value = 0.0f;

    module.configureAnalogOut(number);
}

/**
 * Deletes the analog output object.
 */
AnalogOut::~AnalogOut() {}

/**
 * Sets the gain parameter of this analog output object.
 * @param gain the gain parameter.
 */
void AnalogOut::setGain(float gain)
{

    this->gain = gain;
}

/**
 * Gets the gain parameter of this analog output object.
 * @return the gain parameter.
 */
float AnalogOut::getGain()
{

    return gain;
}

/**
 * Sets the offset parameter of this analog output object.
 * @param offset the offset parameter.
 */
void AnalogOut::setOffset(float offset)
{

    this->offset = offset;
}

/**
 * Gets the offset parameter of this analog output object.
 * @return the offset parameter.
 */
float AnalogOut::getOffset()
{

    return offset;
}

/**
 * Writes the analog output value.
 * @param value a floating point number specifying the analog output value.
 */
void AnalogOut::write(float value)
{

    this->value = value;

    module.writeAnalogOut(number, value * gain + offset);
}

/**
 * The '=' operator is a shorthand notation of the <code>write()</code> method.
 */
AnalogOut &AnalogOut::operator=(float value)
{

    write(value);

    return *this;
}

/**
 * Reads the analog output value back.
 * @return a floating point number representing the analog output value.
 */
float AnalogOut::read()
{

    return value;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
AnalogOut::operator float()
{

    return read();
}
