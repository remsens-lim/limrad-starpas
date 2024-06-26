/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include "def.h"
#include <Wire.h>

class I2Cdev {
    public:
                                        I2Cdev(TwoWire*);
         uint8_t                        readByte(uint8_t address, uint8_t subAddress);
         uint8_t                        M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2);
         void                           readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
         void                           M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t *dest);
 
         void                           writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
         void                           M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data);
         void                           writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
         void                           M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t *dest);
         void                           I2Cscan();                                                                                                                    // Class constructor variable
         TwoWire*                       I2C_Bus;                                                                                                                      // Class constructor variable
    private:
};

#endif //_I2CDEV_H_
