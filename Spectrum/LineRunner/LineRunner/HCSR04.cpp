/*

MIT License

Copyright (c) [2019] [Orlin Dimitrov]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <stdlib.h>
#include <string.h>
#include "HCSR04.h"

void HCSR04::init(int tp, int ep)
{
	pinMode(tp, OUTPUT);
	pinMode(ep, INPUT);
	_trigPin = tp;
	_echoPin = ep;
	_cmDivisor = 27.6233;
	_inDivisor = 70.1633;
}

long HCSR04::timing()
{
	digitalWrite(_trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(_trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(_trigPin, LOW);
	return pulseIn(_echoPin, HIGH);
}

float HCSR04::convert(long microsec, int metric)
{
	// microsec / 29 / 2;
	if (metric) return microsec / _cmDivisor / 2.0;  // CM
	// microsec / 74 / 2;
	else return microsec / _inDivisor / 2.0;  // IN
}

void HCSR04::setDivisor(float value, int metric)
{
	if (metric) _cmDivisor = value;
	else _inDivisor = value;
}

#ifdef COMPILE_STD_DEV
bool HCSR04::sampleCreate(size_t numBufs, ...)
{
	bool result = false;
	va_list ap;
	_numBufs = numBufs;

	if ((_pBuffers = (BufCtl*)calloc(numBufs, sizeof(BufCtl))) != NULL)
	{
		va_start(ap, numBufs);
		BufCtl* buf;
		size_t smpSize;

		for (size_t i = 0; i < _numBufs; i++)
		{
			buf = &_pBuffers[i];
			smpSize = va_arg(ap, size_t);

			if ((buf->pBegin = (float*)calloc(smpSize, sizeof(float))) != NULL)
			{
				buf->pIndex = buf->pBegin;
				buf->length = smpSize;
				buf->filled = false;
				result = true;
			}
			else
			{
				result = false;
				break;
			}
		}

		va_end(ap);
	}

	if (!result) _freeBuffers();
	return result;
}

void HCSR04::sampleClear()
{
	if (_pBuffers)
	{
		BufCtl* buf;

		for (size_t i = 0; i < _numBufs; i++)
		{
			buf = &_pBuffers[i];
			memset(buf, '\0', sizeof(float) * buf->length);
			buf->pIndex = buf->pBegin;
			buf->filled = false;
		}
	}
}

float HCSR04::unbiasedStdDev(float value, size_t bufNum)
{
	float result = 0.0;

	if (_pBuffers)
	{
		BufCtl* buf = &_pBuffers[bufNum];

		if (buf->length > 1)
		{
			_sampleUpdate(buf, float(value));

			if (buf->filled)
			{
				float sum = 0.0, mean, tmp;

				for (size_t i = 0; i < buf->length; i++)
					sum += buf->pBegin[i];

				mean = sum / buf->length;
				sum = 0.0;

				for (size_t i = 0; i < buf->length; i++)
				{
					tmp = buf->pBegin[i] - mean;
					sum += (tmp * tmp);
				}

				result = sqrt(sum / (buf->length - 1));
				//Serial.print(bufNum);
				//Serial.print(" : ");
				//Serial.println(result);
			}
		}
	}

	return result;
}

void HCSR04::_sampleUpdate(BufCtl* buf, float msec)
{
	if (buf->pIndex >= (buf->pBegin + buf->length))
	{
		buf->pIndex = buf->pBegin;
		buf->filled = true;
	}

	*(buf->pIndex++) = msec;
}

void HCSR04::_freeBuffers()
{
	if (_pBuffers)
	{
		BufCtl* buf;

		for (size_t i = 0; i < _numBufs; i++)
		{
			buf = &_pBuffers[i];
			free(buf->pBegin);
		}

		free(_pBuffers);
	}
}
#endif // COMPILE_STD_DEV
