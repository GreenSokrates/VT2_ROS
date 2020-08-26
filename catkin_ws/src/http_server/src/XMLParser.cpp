/*
 * XMLParser.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.07.2015
 *      Author: honegger
 */

#include <http_server/XMLParser.h>

using namespace std;

/**
 * Parses a text string with XML tagged information.
 * @param str the given string to parse.
 * @param tag the tag to parse the string for.
 * @return a substring of the given string within the XML tags.
 */
string XMLParser::parse(string str, string tag)
{

    size_t index = 0;
    if ((index = str.find("<" + tag + ">")) != string::npos)
    {
        str = str.substr(index + tag.length() + 2);
        if ((index = str.find("</" + tag + ">")) != string::npos)
            str = str.substr(0, index);
    }
    else if ((index = str.find("<" + tag + " ")) != string::npos)
    {
        str = str.substr(index + tag.length() + 2);
        if ((index = str.find('>')) != string::npos)
            str = str.substr(index + 1);
        if ((index = str.find("</" + tag + ">")) != string::npos)
            str = str.substr(0, index);
    }
    else
        str = "";

    return str;
}

/**
 * Parses a text string with XML tagged information.
 * @param str the given string to parse.
 * @param tag the tag to parse the string for.
 * @param number the index number of this tag, if this tag appears
 * several times in the string to parse.
 * @return a substring of the given string within the XML tags.
 */
string XMLParser::parse(string str, string tag, uint32_t number)
{

    size_t index = 0;
    for (uint32_t i = 0; i <= number; i++)
    {
        if ((index = str.find("<" + tag + ">")) != string::npos)
        {
            str = str.substr(index + tag.length() + 2);
        }
        else if ((index = str.find("<" + tag + " ")) != string::npos)
        {
            str = str.substr(index + tag.length() + 2);
            if ((index = str.find('>')) != string::npos)
                str = str.substr(index + 1);
        }
        else
            str = "";
    }
    if ((index = str.find("</" + tag + ">")) != string::npos)
        str = str.substr(0, index);

    return str;
}

/**
 * Parses a text string with a <code>string</code> object.
 */
string XMLParser::parseString(string str)
{

    return parse(str, "string");
}

/**
 * Parses a text string with a <code>string</code> object.
 */
string XMLParser::parseString(string str, uint32_t number)
{

    return parse(str, "string", number);
}

/**
 * Parses a text string with a <code>string</code> object.
 */
string XMLParser::parseString(string str, string tag)
{

    return parse(parse(str, tag), "string");
}

/**
 * Parses a text string with a <code>string</code> object.
 */
string XMLParser::parseString(string str, string tag, uint32_t number)
{

    return parse(parse(str, tag, number), "string");
}

/**
 * Parses a text string with a <code>boolean</code> object.
 */
bool XMLParser::parseBoolean(string str)
{

    return (parse(str, "boolean").compare("true") == 0);
}

/**
 * Parses a text string with a <code>boolean</code> object.
 */
bool XMLParser::parseBoolean(string str, uint32_t number)
{

    return (parse(str, "boolean", number).compare("true") == 0);
}

/**
 * Parses a text string with a <code>boolean</code> object.
 */
bool XMLParser::parseBoolean(string str, string tag)
{

    return (parse(parse(str, tag), "boolean").compare("true") == 0);
}

/**
 * Parses a text string with a <code>boolean</code> object.
 */
bool XMLParser::parseBoolean(string str, string tag, uint32_t number)
{

    return (parse(parse(str, tag, number), "boolean").compare("true") == 0);
}

/**
 * Parses a text string with a <code>short</code> object.
 */
int16_t XMLParser::parseShort(string str)
{

    return static_cast<short>(strtol(parse(str, "short").c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>short</code> object.
 */
int16_t XMLParser::parseShort(string str, uint32_t number)
{

    return static_cast<short>(strtol(parse(str, "short", number).c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>short</code> object.
 */
int16_t XMLParser::parseShort(string str, string tag)
{

    return static_cast<short>(strtol(parse(parse(str, tag), "short").c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>short</code> object.
 */
int16_t XMLParser::parseShort(string str, string tag, uint32_t number)
{

    return static_cast<short>(strtol(parse(parse(str, tag, number), "short").c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>int</code> object.
 */
int32_t XMLParser::parseInt(string str)
{

    return static_cast<int>(strtol(parse(str, "int").c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>int</code> object.
 */
int32_t XMLParser::parseInt(string str, uint32_t number)
{

    return static_cast<int>(strtol(parse(str, "int", number).c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>int</code> object.
 */
int32_t XMLParser::parseInt(string str, string tag)
{

    return static_cast<int>(strtol(parse(parse(str, tag), "int").c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>int</code> object.
 */
int32_t XMLParser::parseInt(string str, string tag, uint32_t number)
{

    return static_cast<int>(strtol(parse(parse(str, tag, number), "int").c_str(), NULL, 10));
}

/**
 * Parses a text string with a <code>long</code> object.
 */
int64_t XMLParser::parseLong(string str)
{

    return strtol(parse(str, "long").c_str(), NULL, 10);
}

/**
 * Parses a text string with a <code>long</code> object.
 */
int64_t XMLParser::parseLong(string str, uint32_t number)
{

    return strtol(parse(str, "long", number).c_str(), NULL, 10);
}

/**
 * Parses a text string with a <code>long</code> object.
 */
int64_t XMLParser::parseLong(string str, string tag)
{

    return strtol(parse(parse(str, tag), "long").c_str(), NULL, 10);
}

/**
 * Parses a text string with a <code>long</code> object.
 */
int64_t XMLParser::parseLong(string str, string tag, uint32_t number)
{

    return strtol(parse(parse(str, tag, number), "long").c_str(), NULL, 10);
}

/**
 * Parses a text string with a <code>float</code> object.
 */
float XMLParser::parseFloat(string str)
{

    return static_cast<float>(strtod(parse(str, "float").c_str(), NULL));
}

/**
 * Parses a text string with a <code>float</code> object.
 */
float XMLParser::parseFloat(string str, uint32_t number)
{

    return static_cast<float>(strtod(parse(str, "float", number).c_str(), NULL));
}

/**
 * Parses a text string with a <code>float</code> object.
 */
float XMLParser::parseFloat(string str, string tag)
{

    return static_cast<float>(strtod(parse(parse(str, tag), "float").c_str(), NULL));
}

/**
 * Parses a text string with a <code>float</code> object.
 */
float XMLParser::parseFloat(string str, string tag, uint32_t number)
{

    return static_cast<float>(strtod(parse(parse(str, tag, number), "float").c_str(), NULL));
}

/**
 * Parses a text string with a <code>double</code> object.
 */
double XMLParser::parseDouble(string str)
{

    return strtod(parse(str, "double").c_str(), NULL);
}

/**
 * Parses a text string with a <code>double</code> object.
 */
double XMLParser::parseDouble(string str, uint32_t number)
{

    return strtod(parse(str, "double", number).c_str(), NULL);
}

/**
 * Parses a text string with a <code>double</code> object.
 */
double XMLParser::parseDouble(string str, string tag)
{

    return strtod(parse(parse(str, tag), "double").c_str(), NULL);
}

/**
 * Parses a text string with a <code>double</code> object.
 */
double XMLParser::parseDouble(string str, string tag, uint32_t number)
{

    return strtod(parse(parse(str, tag, number), "double").c_str(), NULL);
}
