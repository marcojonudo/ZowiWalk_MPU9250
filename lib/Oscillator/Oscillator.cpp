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
int goOnCounter = 0, turnThresholdCounter = 0;

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

    delay(5);
}

void Oscillator::RefreshVariables() {
    _cycleStarted = false;
    _goOn = true;
    goOnCounter = 0;
    turnThresholdCounter = 0;
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
	double inc = 0.05;

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
	double inc = 0.05;

	if (!_cycleStarted) {
		_phase = _phase + inc;
		_pos = round(_A * sin(_phase + _phase0) + _O);
		_servo.write(_pos+90+_trim);
		delay(5);

        /* El ciclo de giro comienza cuando la posición del servo sea distita */
        /* a la inicial (pin 4: 92, pin 5: 88) */
		if ((_servo.read()-_trim)!=_initialPos) {
			_cycleStarted = true;
		}
	}
	else {
		_phase = _phase + inc;
		_pos = round(_A * sin(_phase + _phase0) + _O);
		_servo.write(_pos+90+_trim);
		delay(5);

        /* Cuando se vuelve a la posición inicial, significa que ya se completó */
        /* un ciclo completo (un paso) */
		if ((_servo.read()-_trim)==_initialPos) {
            goOnCounter += 1;
            Serial.print("Foot: "); Serial.println(goOnCounter);
            if (goOnCounter == 4) {
                _goOn = false;
                Serial.print("FootGoOn: "); Serial.println(_goOn);
            }
		}
	}
}

void Oscillator::moveHipServo(int degreeDiff) {
    /* En el caso de establecer inc = 0.04, probar con hipDegrees = 61.
    Con este valor no se llega a los 120º (debido a la aproximación) al no
    superar los 119.5 */
	double inc = 0.05;
    int prevDegreeDiff;

    /* If compensation needed, we need to move the servos over the 60-120º limits.
    This variable is used to check that condition */

    if (_turnThreshold) {
        _pos += 1;
        _servo.write(_pos+90+_trim);
        Serial.print(_pin); Serial.print(" --- "); Serial.println(_servo.read()-_trim);

        turnThresholdCounter -= 1;
        if (turnThresholdCounter == degreeDiff) {
            _turnThreshold = false;
            goOnCounter += 1;
            Serial.print("Hip: "); Serial.println(goOnCounter);
            if (goOnCounter == 4) {
                _goOn = false;
                Serial.print("HipGoOn: "); Serial.println(_goOn);
            }
        }
    }
    else {
        _phase = _phase + inc;
        _pos = round(_A * sin(_phase + _phase0) + _O);
        prevDegreeDiff = degreeDiff;
        degreeDiff = 0;
        // Que degreediff deje de ser 0 para entrar en el sig bucle
    }

    /* El +30 es para situar la posición entre 0 y 60, en lugar de entre */
    /* -30 y +30 (valores devueltos por la función seno) */
    if (((_pos+30) < (hipDegrees-degreeDiff))&&((_pos+30) > degreeDiff)) {
        _servo.write(_pos+90+_trim);
        Serial.print(_pin); Serial.print(" - "); Serial.println(_servo.read()-_trim);
        _firstTime = true;
    }
    else if (_firstTime) {
        _servo.write(_pos+90+_trim);
        Serial.print(_pin); Serial.print(" -- "); Serial.println(_servo.read()-_trim);
        _firstTime = false;

        if (prevDegreeDiff < 0) {
            _turnThreshold = true;
            Serial.println("_turnThreshold to true");
        }
        else {
            goOnCounter += 1;
            Serial.print("Hip2: "); Serial.println(goOnCounter);
        }
    }

	delay(5);
}

bool Oscillator::goOn() {
	return _goOn;
}

void Oscillator::setGoOn(bool goOn) {
	_goOn = goOn;
}

bool Oscillator::getGoOnCounter() {
    return goOnCounter;
}

void Oscillator::setPhase(double phase)
{
	_phase = phase;
}
