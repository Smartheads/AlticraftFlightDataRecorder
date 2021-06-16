/*
* MIT License
*
* Copyright (c) 2018 Robert Hutter
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "SRF05.h"

SRL::SRF05::SRF05(uint8_t triggerPin, uint8_t echoPin, double maxDistanceCm): Sonar(triggerPin, echoPin, convertUs(maxDistanceCm)), Component(SRF05_COMPONENT_NAME, Component::SONAR)
{

}

unsigned long SRL::SRF05::ping(void)
{
  pingTrigger();
  unsigned long reading = pulseIn(echoPin, HIGH);

  if (reading == 0 || reading > maxDistance)
  {
    return NO_ECHO;
  }

  return reading - PING_OVERHEAD;
}

void SRL::SRF05::pingTrigger(void)
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(4);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
}

double SRL::SRF05::convertCm(unsigned long us)
{
  return LINEAR_FUNCTION_SLOPE * us + LINEAR_FUNCTION_INTERSECT;
}

unsigned long SRL::SRF05::convertUs(double cm)
{
  return (cm - LINEAR_FUNCTION_INTERSECT) / LINEAR_FUNCTION_SLOPE;
}