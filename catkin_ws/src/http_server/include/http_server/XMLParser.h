/*
 * XMLParser.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.07.2015
 *      Author: honegger
 */

#ifndef XML_PARSER_H_
#define XML_PARSER_H_

#include <string>
#include <cstdlib>
#include <stdint.h>

using namespace std;

class XMLParser {
    
    public:
        
        static string   parse(string str, string tag);
        static string   parse(string str, string tag, uint32_t number);
        static string   parseString(string str);
        static string   parseString(string str, uint32_t number);
        static string   parseString(string str, string tag);
        static string   parseString(string str, string tag, uint32_t number);
        static bool     parseBoolean(string str);
        static bool     parseBoolean(string str, uint32_t number);
        static bool     parseBoolean(string str, string tag);
        static bool     parseBoolean(string str, string tag, uint32_t number);
        static int16_t  parseShort(string str);
        static int16_t  parseShort(string str, uint32_t number);
        static int16_t  parseShort(string str, string tag);
        static int16_t  parseShort(string str, string tag, uint32_t number);
        static int32_t  parseInt(string str);
        static int32_t  parseInt(string str, uint32_t number);
        static int32_t  parseInt(string str, string tag);
        static int32_t  parseInt(string str, string tag, uint32_t number);
        static int64_t  parseLong(string str);
        static int64_t  parseLong(string str, uint32_t number);
        static int64_t  parseLong(string str, string tag);
        static int64_t  parseLong(string str, string tag, uint32_t number);
        static float    parseFloat(string str);
        static float    parseFloat(string str, uint32_t number);
        static float    parseFloat(string str, string tag);
        static float    parseFloat(string str, string tag, uint32_t number);
        static double   parseDouble(string str);
        static double   parseDouble(string str, uint32_t number);
        static double   parseDouble(string str, string tag);
        static double   parseDouble(string str, string tag, uint32_t number);
};

#endif /* XML_PARSER_H_ */
