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

#include <avr/interrupt.h>
#include <wiring.h>
#include <LEDStrip.h>

/* NOTE: the code looks like it currently supports operation of the
 * sPin on either pin 9 or 10.  In actuality, only pin 9 will work.
 */

uint8_t LEDStrip::attached9 = 0;
uint8_t LEDStrip::attached10 = 0;


void LEDStrip::startTimer1()
{
  TCCR1B = (TCCR1B & ~_BV(CS11)) | (_BV(CS10) | _BV(CS12)); /* div 1024 clock prescaler */
}

void LEDStrip::stopTimer1()
{
  TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));
}

void LEDStrip::seizeTimer1()
{
  uint8_t oldSREG = SREG;

  cli();
  TCCR1A = _BV(WGM10) | _BV(WGM11);  /* Fast PWM, OCR1A is top */
  TCCR1B = _BV(WGM13) | _BV(WGM12);  /* Fast PWM, OCR1A is top */

  OCR1A = 65535 - _fadeSpeed;
#if defined(__AVR_ATmega8__)
  TIMSK &= ~(_BV(TICIE1) | _BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) );
#else
  TIMSK1 &=  ~(_BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) );
#endif

  SREG = oldSREG;  // undo cli()    
}

void LEDStrip::releaseTimer1() {}

LEDStrip::LEDStrip() : _dPin(0), _sPin(0), _latchPin(0), _clkPin(0), _fadeSpeed(0) {}

uint8_t LEDStrip::attach(int dPin, int sPin, int latchPin, int clkPin)
{

  if (sPin != 9 /*&& sPin != 10*/) return 0;
  
  _dPin = dPin;
  _sPin = sPin;
  _latchPin = latchPin;
  _clkPin = clkPin;

  digitalWrite(_dPin, LOW);
  pinMode(_dPin, OUTPUT);
  digitalWrite(_sPin, LOW);
  pinMode(_sPin, OUTPUT);
  digitalWrite(_latchPin, LOW);
  pinMode(_latchPin, OUTPUT);
  digitalWrite(_clkPin, LOW);
  pinMode(_clkPin, OUTPUT);

  if (!attached9 && !attached10) {
    seizeTimer1();
    setSpeed(0);
  }
  
  if (_sPin == 9) {
    attached9 = 1;
    //TCCR1A = (TCCR1A & ~_BV(COM1A0)) | _BV(COM1A1);
    attachOCR1A();
  }
  
  if (_sPin == 10) {
    attached10 = 1;
    // not working
    //TCCR1A = (TCCR1A & ~_BV(COM1B0)) | _BV(COM1B1);
  }
  return 1;
}

void LEDStrip::attachOCR1A()
{
  TCCR1A = (TCCR1A & ~_BV(COM1A1)) | _BV(COM1A0);
}

void LEDStrip::detachOCR1A()
{
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
}

void LEDStrip::detach()
{

  stopTimer1();
  // muck with timer flags
  if (_sPin == 9) {
    attached9 = 0;
    detachOCR1A();
    pinMode(_sPin, INPUT);
  } 
  
  if (_sPin == 10) {
    attached10 = 0;
    TCCR1A = TCCR1A & ~_BV(COM1B0) & ~_BV(COM1B1);
    pinMode(_sPin, INPUT);
  }

  if (!attached9 && !attached10) releaseTimer1();
}

void LEDStrip::setSpeed(uint16_t speed)
{

  if (speed == 0) {
    stopTimer1();
  }

  if (_sPin == 9) OCR1A = 65535 - speed;
//  if (_sPin == 10) OCR1B = 65535 - speed;

  if (speed != 0 && _fadeSpeed == 0) {
    startTimer1();
  }

  _fadeSpeed = speed;

}

uint16_t LEDStrip::getSpeed() 
{
  return _fadeSpeed;
}

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

void LEDStrip::sPulse()
{
  detachOCR1A();

  if (digitalRead(_sPin) == HIGH) {
    delayMicroseconds(1000);
    digitalWrite(_sPin, LOW);
    delayMicroseconds(1000);
    digitalWrite(_sPin, HIGH);
    delayMicroseconds(1000);
  } else {
    delayMicroseconds(1000);
    digitalWrite(_sPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(_sPin, LOW);
    delayMicroseconds(1000);
  }
  attachOCR1A();

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

uint8_t LEDStrip::attached()
{
  if (_sPin == 9 && attached9) return 1;
  if (_sPin == 10 && attached10) return 1;
  return 0;
}
