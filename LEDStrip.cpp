/*
 * LEDStrip - Arduino driver for HL1606-based LED strips
 * Thanks to: John M Cohn
 * Copyright (c) 2009, Synoptic Labs
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
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

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WConstants.h"
#endif
#include "LEDStrip.h"

LEDStrip::LEDStrip(int dPin, int sPin, int latchPin, int clkPin) 
{
  _dPin = dPin;
  _sPin = sPin;
  _latchPin = latchPin;
  _clkPin = clkPin;
  _faderEnabled = 0;
  
  digitalWrite(_dPin, LOW);
  pinMode(_dPin, OUTPUT);
  digitalWrite(_sPin, LOW);
  pinMode(_sPin, OUTPUT);
  digitalWrite(_latchPin, LOW);
  pinMode(_latchPin, OUTPUT);
  digitalWrite(_clkPin, LOW);
  pinMode(_clkPin, OUTPUT);
}

void LEDStrip::sleep()
{
  digitalWrite(_dPin, LOW);
  pinMode(_dPin, INPUT);
  digitalWrite(_sPin, LOW);
  pinMode(_sPin, INPUT);
  digitalWrite(_latchPin, LOW);
  pinMode(_latchPin, INPUT);
  digitalWrite(_clkPin, LOW);
  pinMode(_clkPin, INPUT);
}


void LEDStrip::wakeup()
{
  digitalWrite(_dPin, LOW);
  pinMode(_dPin, OUTPUT);
  digitalWrite(_sPin, LOW);
  pinMode(_sPin, OUTPUT);
  digitalWrite(_latchPin, LOW);
  pinMode(_latchPin, OUTPUT);
  digitalWrite(_clkPin, LOW);
  pinMode(_clkPin, OUTPUT);
}

void LEDStrip::faderCrank()
{
  unsigned long mymillis;
  
  if (!_faderEnabled) return;

  mymillis = millis();

  // Give us 250ms slop in case we don't exactly catch our edge.
  if (mymillis >= _faderPulseNextEdge && mymillis < _faderPulseNextEdge + 250) {
    if (digitalRead(_sPin) == HIGH) {
      digitalWrite(_sPin, LOW);
    } else {
      // only load new value of _faderPulseHalfWidth on rising edge
      digitalWrite(_sPin, HIGH);
      _faderPulseHalfWidth = _faderPulseNewHalfWidth;
    }

    _faderPulseNextEdge = mymillis + _faderPulseHalfWidth;
  }
}

unsigned int LEDStrip::faderSpeedGet()
{
  return _faderPulseHalfWidth;
}

void LEDStrip::faderSpeedSet(unsigned int halfWidthms)
{
  if (halfWidthms == 0) {
    _faderEnabled = 0;
    _faderPulseHalfWidth = 0;
    _faderPulseNewHalfWidth = 0;
    digitalWrite(_sPin, LOW);
    return;   
  }

  _faderPulseNewHalfWidth = halfWidthms;
  
  // if we're already running, don't re-init _faderPulseNextEdge
  if (_faderEnabled != 1) {  // starting from non-running state, 
    _faderEnabled = 1;

    digitalWrite(_sPin, HIGH);
    _faderPulseHalfWidth = halfWidthms;
    _faderPulseNextEdge = millis() + _faderPulseHalfWidth;
  }
}

/* The HL1606 drives 2 RGB LED's.  Each 3-color LED is controlled with a command
 * word consisting of 8 bits.  Command word is clocked out MSB first (i.e. D8
 * is first bit sent)
 *
 * Format of command word (using conventions in datasheet):
 *   ________________________________________________________________________
 *  |   D1   |   D2   |   D3   |   D4   |   D5   |   D6   |   D7   |    D8   |
 *   ------------------------------------------------------------------------
 *   ________________________________________________________________________
 *  |     LED1 CMD    |    LED2 CMD     |    LED3 CMD     |   2X   | LatchOK |
 *   ------------------------------------------------------------------------
 *
 *   LED{1,2,3} CMD -
 *       00 - LED off
 *       01 - LED on (max bright)
 *       10 - LED fade up   (start at min bright)
 *       11 - LED fade down (start at max bright)
 *
 *   2X - Double fade speed
 *       0 - 1X fade speed, each pulse on SI line steps brightness by 1/128th.
 *       1 - 2X fade speed, each pulse on SI line steps brightness by 1/64th.    
 *
 *   LatchOK - Enable latch.  Set to 0 to insert 'white space' in the serial
 *             chain.  If set to 0, the entire CMD is ignored.
 *       0 - Do not latch this CMD when Latch is thrown.
 *       1 - Latch CMD as normal when Latch is thrown.
 *
*/

// Push a color value down the strip, setting the latch-enable flag.
void LEDStrip::rgbPush(uint8_t redcmd, uint8_t greencmd, uint8_t bluecmd)
{
  uint8_t cmd = 0;
  uint8_t flags = LATCH;

  if (redcmd >= NONCMD || bluecmd >= NONCMD || greencmd >= NONCMD) return;

  cmd |= (greencmd << 4) & (_BV(5) | _BV(4));
  cmd |= (redcmd << 2) & (_BV(3) | _BV(2));
  cmd |= (bluecmd) & (_BV(1) | _BV(0));
  cmd |= flags & (_BV(6) | _BV(7)); 

  pushCmd(cmd);
}

void LEDStrip::rgbPush2X(uint8_t redcmd, uint8_t greencmd, uint8_t bluecmd)
{
  uint8_t cmd = 0;
  uint8_t flags = LATCH | SPEED2X;

  if (redcmd >= NONCMD || bluecmd >= NONCMD || greencmd >= NONCMD) return;

  cmd |= (greencmd << 4) & (_BV(5) | _BV(4));
  cmd |= (redcmd << 2) & (_BV(3) | _BV(2));
  cmd |= (bluecmd) & (_BV(1) | _BV(0));
  cmd |= flags & (_BV(6) | _BV(7)); 

  pushCmd(cmd);
}

void LEDStrip::sPulse()
{
  if (digitalRead(_sPin) == HIGH) {
    //delay(1);
    digitalWrite(_sPin, LOW);
    delayMicroseconds(1000);
    digitalWrite(_sPin, HIGH);
    delayMicroseconds(1000);
  } else {
    //delay(1);
    digitalWrite(_sPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(_sPin, LOW);
    delayMicroseconds(1000);
  }

}

// Push a blank value down the strip, not setting latch-enable flag.  
// Does not affect the status of a particular LED when latched.  It's
// like using whitespace.
void LEDStrip::blankPush()
{
  pushCmd(0); 
}

void LEDStrip::pushCmd(uint8_t cmd)
{
  shiftOut(_dPin, _clkPin, MSBFIRST, cmd);
}

void LEDStrip::latch()
{
  digitalWrite(_latchPin, HIGH);
  delayMicroseconds(1);  // spec sheet specifies minimum latch pulse of 1us
  digitalWrite(_latchPin, LOW);
}

