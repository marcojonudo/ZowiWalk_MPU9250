//--------------------------------------------------------------
//-- Oscillator.pde
//-- Generate sinusoidal oscillations in the servos
//--------------------------------------------------------------
//-- (c) Juan Gonzalez-Gomez (Obijuan), Dec 2011
//-- GPL license
//--------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif
#include "Oscillator.h"
#include <Servo.h>

int hipDegrees = 60;
int goOnCounter = 0;

//-- This function returns true if another sample
//-- should be taken (i.e. the TS time has passed since
//-- the last sample was taken
bool Oscillator::next_sample()
{

  //-- Read current time
  _currentMillis = millis();

  //-- Check if the timeout has passed
  if(_currentMillis - _previousMillis > _TS) {
    _previousMillis = _currentMillis;

    return true;
  }

  return false;
}

//-- Attach an oscillator to a servo
//-- Input: pin is the arduino pin were the servo
//-- is connected
void Oscillator::attach(int pin, bool rev)
{
  //-- If the oscillator is detached, attach it.
  if(!_servo.attached()){

    //-- Attach the servo and move it to the home position
      _servo.attach(pin);
      //_servo.write(90);

      _pin = pin;

      //-- Default parameters
      //_A=45;

      //_phase0=0;
      //_O=0;
      _stop=false;

      //-- Reverse mode
      _rev = rev;

      _goOn = true;
  }

}

//-- Detach an oscillator from his servo
void Oscillator::detach()
{
   //-- If the oscillator is attached, detach it.
  if(_servo.attached())
        _servo.detach();

}

/*************************************/
/* Set the oscillator period, in ms  */
/*************************************/
void Oscillator::SetT(unsigned int T)
{
	_phase=0;
	_cycleStarted = false;
    _turnHips = false;
    _pos = round(_A * sin(_phase + _phase0) + _O);
    _initialPos = _pos+90;

    if ((_pin==4)||(_pin==5)) {
      _servo.write(_pos+90+_trim);
    }

    delay(5);
};

void Oscillator::SetParameters(unsigned int A, unsigned int O, double Ph) {
    _A = A;
    _O = O;
    _phase0 = Ph;

    _phase = 0;
    _cycleStarted = false;
    _turnHips = false;
    _firstTime = true;
    _pos = round(_A * sin(_phase + _phase0) + _O);
    _initialPos = _pos + 90;
    _turnThreshold = false;
    _turnThresholdCounter = 0;
    _overFootThreshold = false;

    delay(5);
}

void Oscillator::RefreshVariables() {
    _cycleStarted = false;
    _goOn = true;
    goOnCounter = 0;
    _turnThresholdCounter = 0;
    _overFootThreshold = false;
}

/*******************************/
/* Manual set of the position  */
/******************************/

void Oscillator::SetPosition(int position)
{
    _servo.write(position+_trim);
};


/*******************************************************************/
/* This function should be periodically called                     */
/* in order to maintain the oscillations. It calculates            */
/* if another sample should be taken and position the servo if so  */
/*******************************************************************/
void Oscillator::refresh()
{
  //-- Only When TS milliseconds have passed, the new sample is obtained
  if (next_sample()) {

      //-- If the oscillator is not stopped, calculate the servo position
      if (!_stop) {

        //-- Sample the sine function and set the servo pos
         _pos = round(_A * sin(_phase + _phase0) + _O);
	       if (_rev) _pos=-_pos;
         _servo.write(_pos+90+_trim);
      }

      //recalculateInc();

      //-- Increment the phase
      //-- It is always increased, even when the oscillator is stop
      //-- so that the coordination is always kept
      _phase = _phase + _inc;

  }
}

void Oscillator::oscillateServosDegrees(int degreeDiff)
{
	if (!_stop) {
		checkPin(degreeDiff);
	}
}

void Oscillator::checkPin(int degreeDiff) {
	if ((_pin==2)||(_pin==3)) {
		moveHip(degreeDiff);
	}
	else {
		moveServo();
	}

}

void Oscillator::moveHip(int degreeDiff) {
	/* Con un inc de 0.05 la precisión es los suficientemente alta como para */
	/* comparar con _initialPos a la hora de parar los servos */
	double inc = 0.13;

	if (!_turnHips) {
		_phase = _phase + inc;
		_pos = round(_A * sin(_phase + _phase0) + _O);
        delay(5);

        /* Se mueve la cadera cuando _pos+90>=90 para que el movimiento sea siempre */
        /* hacia delante. Si comenzara inmediatamente (60º), primero el giro sería */
        /* hacia atras (de 90º en reposo a 60º) para despues ser hacia delante */
        /* (hasta los 120º que se persiguen) */
		if ((_pos+90)>=90) {
			_turnHips = true;
		}
	}
	else {
		moveHipServo(degreeDiff);
	}
}

void Oscillator::moveServo() {
	double inc = 0.13;

    _phase = _phase + inc;
    _pos = round(_A * sin(_phase + _phase0) + _O);
    Serial.print(_pin); Serial.print(":  "); Serial.println(_pos+90);

	if (!_cycleStarted) {
		_servo.write(_pos+90+_trim);
		delay(5);
        Serial.print(_pin); Serial.print(":    "); Serial.println(_pos+90);

        /* Turn cycle starts when servo's position is different to the
        initial one (pin 4: 94, pin 5: 86) */
        /* El ciclo de giro comienza cuando la posición del servo sea distita */
        /* a la inicial (pin 4: 94, pin 5: 86) */
		if ((_servo.read()-_trim)!=_initialPos) {
			_cycleStarted = true;
		}
	}
    /* _overFootThreshold is used, after reaching _initialPos, for avoiding going on
    writing positions in the servos */
	else if (((_pos+90) != _initialPos)&&(!_overFootThreshold)) {
		_servo.write(_pos+90+_trim);

		delay(5);
	}
    /* This last 'else if' allows the program to write the last degree in the feet servos,
    and not do it again */
    else if (((_pos+90) == _initialPos)&&(!_overFootThreshold)) {
        _servo.write(_pos+90+_trim);

        /* If degreeDiff >= 0, goOnCounter reach value 4 when increased by feet
        (because of the loop inside Zowi::feetMovement) */
        goOnCounter += 1;
        Serial.print(_pin); Serial.print(" - "); Serial.println(goOnCounter);
        if (goOnCounter == 4) {
            _goOn = false;
        }
        Serial.print(_pin); Serial.print(" - "); Serial.println(_goOn);

        _overFootThreshold = true;
    }
}

void Oscillator::moveHipServo(int degreeDiff) {
	double inc = 0.13;
    int prevDegreeDiff;

    if (!_turnThreshold) {
        _phase = _phase + inc;
        _pos = round(_A * sin(_phase + _phase0) + _O);
        prevDegreeDiff = degreeDiff;
        if (degreeDiff < 0) {
            degreeDiff = 0;
        }
    }

    /* The +30 is used to make the position be between 0 and 60, instead of
    between -30 and +30 (values given by the sin function) */
    if (((_pos+30) < (hipDegrees-degreeDiff))&&((_pos+30) > degreeDiff)&&(_turnThreshold)) {
        /* If sin(_phase)>0, the angle is increasing. So the _pos also has to increase to reached
        120+*/
        if (sin(_phase) > 0) {
            _pos += 1;
        }
        else {
            _pos -= 1;
        }

        _servo.write(_pos+90+_trim);

        _turnThresholdCounter -= 1;
        if (_turnThresholdCounter == degreeDiff) {
            _turnThreshold = false;

            /* If degreeDiff < 0, goOnCounter reach value 4 when increased by hip
            (because hip servos move over the threshold, so it takes longer) */
            goOnCounter += 1;
            if (goOnCounter == 4) {
                _goOn = false;
            }
        }
    }
    else if (((_pos+30) < (hipDegrees-degreeDiff))&&((_pos+30) > degreeDiff)) {
        _servo.write(_pos+90+_trim);
        _firstTime = true;
    }
    else if (_firstTime) {
        _servo.write(_pos+90+_trim);
        _firstTime = false;

        if (prevDegreeDiff < 0) {
            _turnThreshold = true;
        }
        else {
            /* It is not necessary to check if goOnCounter=4 or not. If degreeDiff>=0,
            as hips correspond to servo[0] and [1] and they are executed before in the
            for loop, this value is reached in servo[3] (foot) */
            goOnCounter += 1;
        }
    }

	delay(5);
}

bool Oscillator::goOn() {
	return _goOn;
}

void Oscillator::setPhase(double phase)
{
	_phase = phase;
}
