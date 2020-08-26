/*
 * Module.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 05.04.2016
 *      Author: Marcel Honegger
 */

#include <http_server/Module.h>

/**
 * Creates a module object.
 */
Module::Module() {}

/**
 * Deletes the module object.
 */
Module::~Module() {}

/**
 * This method is usually called by an AnalogIn object after it was created.
 * It allows to configure the module for that AnalogIn channel.
 * @param number the index number of the analog input.
 */
void Module::configureAnalogIn(uint16_t number) {}

/**
 * This method is usually called by an AnalogOut object after it was created.
 * It allows to configure the module for that AnalogOut channel.
 * @param number the index number of the analog output.
 */
void Module::configureAnalogOut(uint16_t number) {}

/**
 * This method is usually called by a DigitalIn object after it was created.
 * It allows to configure the module for that DigitalIn channel.
 * @param number the index number of the digital input.
 */
void Module::configureDigitalIn(uint16_t number) {}

/**
 * This method is usually called by a DigitalOut object after it was created.
 * It allows to configure the module for that DigitalOut channel.
 * @param number the index number of the digital output.
 */
void Module::configureDigitalOut(uint16_t number) {}

/**
 * This method reads an analog input. It is usually called by an AnalogIn object.
 * @param number the index number of the analog input.
 * @return the value of the analog input.
 */
float Module::readAnalogIn(uint16_t number)
{

    return 0.0f;
}

/**
 * This method writes an analog output. It is usually called by an AnalogOut object.
 * @param number the index number of the analog output.
 * @param value the value of the analog output.
 */
void Module::writeAnalogOut(uint16_t number, float value) {}

/**
 * This method reads a digital input. It is usually called by a DigitalIn object.
 * @param number the index number of the digital input.
 * @return the value of the digital input, given as a bool value (either <code>true</code> or <code>false</code>).
 */
bool Module::readDigitalIn(uint16_t number)
{

    return false;
}

/**
 * This method writes a digital output. It is usually called by a DigitalOut object.
 * @param number the index number of the digital output.
 * @param value the value of the digital output, given as a bool value (either <code>true</code> or <code>false</code>).
 */
void Module::writeDigitalOut(uint16_t number, bool value) {}
