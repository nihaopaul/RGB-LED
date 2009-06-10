/*
 * LEDStrip - Arduino driver for HL1606-based LED strips
 * Thanks to: John M Cohn
 * Copyright (c) 2009, Synoptic Labs
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SYNOPTIC LABS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SYNOPTIC LABS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LEDStrip_h
#define LEDStrip_h

#include <inttypes.h>

enum {
  OFF,  // 0b00
  ON,   // 0b01
  UP,   // 0b10
  DOWN, // 0b11
  NONCMD 
};

#define LATCH (_BV(7))

class LEDStrip
{
  private:
    uint8_t _dPin;
    uint8_t _sPin;       // can only be pin 9 or 10
    uint8_t _latchPin;
    uint8_t _clkPin;
    uint16_t _fadeSpeed;
    static uint8_t attached9;
    static uint8_t attached10;
    static void releaseTimer1();
    static void startTimer1();
    static void stopTimer1();
    static void attachOCR1A();
    static void detachOCR1A();
  public:
    LEDStrip();
    void sPulse();
    void seizeTimer1();
    uint8_t attach(int, int, int, int);
    void detach();
    void setSpeed(uint16_t);
    uint16_t getSpeed();
    void rgbPush(uint8_t, uint8_t, uint8_t);
    void blankPush();
    void latch();
    uint8_t attached();
    void pushCmd(uint8_t);
};

#endif
