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

  Serial.print(_pin);
  Serial.print(':');
  Serial.print(' ');
  Serial.println(_initialPos);

  if ((_pin==4)||(_pin==5)) {
	  _servo.write(_pos+90+_trim);
  }

  delay(5);
};

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

void Oscillator::oscillateServosDegrees()
{
	if (!_stop) {
		checkPin();
	}
}

void Oscillator::checkPin() {
	if ((_pin==2)||(_pin==3)) {
		moveHip();
	}
	else {
		moveServo();
	}

}

void Oscillator::moveHip() {
	//Con un inc de 0.05 la precisión es los suficientemente alta como para
	//comparar con _initialPos a la hora de parar los servos
	double inc = 0.05;

	if (!_turnHips) {
		_phase = _phase + inc;
		_pos = round(_A * sin(_phase + _phase0) + _O);
		delay(5);
		if ((_pos+90)>=90) {
			_turnHips = true;
		}
	}
	else {
		moveHipServo();
	}
}

void Oscillator::moveServo() {
	double inc = 0.05;

	if (!_cycleStarted) {
		_phase = _phase + inc;
		_pos = round(_A * sin(_phase + _phase0) + _O);
		_servo.write(_pos+90+_trim);
		delay(5);

		if ((_servo.read()-_trim)!=_initialPos) {
			_cycleStarted = true;
		}
	}
	else {
		_phase = _phase + inc;
		_pos = round(_A * sin(_phase + _phase0) + _O);
		_servo.write(_pos+90+_trim);
		delay(5);

//		if (_pin == 4) {
//			Serial.print(_pin);
//			Serial.print(';');
//			Serial.print(' ');
//			Serial.println(_servo.read()-_trim);
//		}

		if ((_servo.read()-_trim)==_initialPos) {
			_goOn = false;
			_cycleStarted = false;
		}
	}
}

void Oscillator::moveHipServo() {
	double inc = 0.05;

	_phase = _phase + inc;
	_pos = round(_A * sin(_phase + _phase0) + _O);
	_servo.write(_pos+90+_trim);
	delay(5);

//	if (abs(_servo.read()-_trim-90)==30) {
//		_goOn = false;
//	}

}

bool Oscillator::goOn() {
	return _goOn;
}

void Oscillator::setGoOn(bool goOn) {
	_goOn = goOn;
}

void Oscillator::setPhase(double phase)
{
	_phase = phase;
}
