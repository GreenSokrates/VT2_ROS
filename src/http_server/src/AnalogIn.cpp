/*
 * AnalogIn.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 28.09.2015
 *      Author: Marcel Honegger
 */

#include <http_server/Module.h>
#include <http_server/AnalogIn.h>

using namespace std;

/**
 * Creates an analog input object and defines a module this input belongs to,
 * as well as the index number of this input on that module.
 * @param module a reference to the module this analog input belongs to.
 * @param number the index number of this analog input on the module.
 */
AnalogIn::AnalogIn(Module &module, uint16_t number) : Channel(module, number)
{

    gain = 1.0f;
    offset = 0.0f;

    module.configureAnalogIn(number);
}

/**
 * Deletes the analog input object.
 */
AnalogIn::~AnalogIn() {}

/**
 * Sets the gain parameter of this analog input object.
 * @param gain the gain parameter.
 */
void AnalogIn::setGain(float gain)
{

    this->gain = gain;
}

/**
 * Gets the gain parameter of this analog input object.
 * @return the gain parameter.
 */
float AnalogIn::getGain()
{

    return gain;
}

/**
 * Sets the offset parameter of this analog input object.
 * @param offset the offset parameter.
 */
void AnalogIn::setOffset(float offset)
{

    this->offset = offset;
}

/**
 * Gets the offset parameter of this analog input object.
 * @return the offset parameter.
 */
float AnalogIn::getOffset()
{

    return offset;
}

/**
 * Gets the analog input value.
 * @return a floating point number representing the analog input value.
 */
float AnalogIn::read()
{

    return module.readAnalogIn(number) * gain + offset;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
AnalogIn::operator float()
{

    return read();
}
