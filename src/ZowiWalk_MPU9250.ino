//----------------------------------------------------------------
//-- Zowi basic firmware v2
//-- (c) BQ. Released under a GPL licencse
//-- 04 December 2015
//-- Authors:  Anita de Prado: ana.deprado@bq.com
//--           Jose Alberca:   jose.alberca@bq.com
//--           Javier Isabel:  javier.isabel@bq.com
//--           Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--           Irene Sanz : irene.sanz@bq.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Zowi has!
//-----------------------------------------------------------------
#include "Wire.h" // 207 bytes

#include <Servo.h> // 42 bytes
#include <Oscillator.h> // 173 bytes
// #include <EEPROM.h> // 0 bytes
#include <BatReader.h> // 0 bytes
#include <US.h> // 0 bytes
#include <LedMatrix.h> // 0 bytes

//-- Library to manage external interruptions
#include <EnableInterrupt.h> // 53 bytes

//-- Library to manage serial commands
#include <ZowiSerialCommand.h> // 0 bytes
/* Commented in order to reduce SRAM usage. Descomment when needed */
//ZowiSerialCommand SCmd;  // 569 bytes

//-- Zowi Library
#include <Zowi.h> // 9 bytes
Zowi zowi;  // 337 bytes

//---------------------------------------------------------
//-- Configuration of pins where the servos are attached
/*
		  ---------------
		 |               |
		 |     O   O     |
		 |               |
 YR ==>  |               | <== YL
		 ---------------
			 ||     ||
			 ||     ||
			 ||     ||
 RR ==>   -----   ------  <== RL
 		  -----   ------
 */

#define PIN_YL 2 //servo[0]
#define PIN_YR 3 //servo[1]
#define PIN_RL 4 //servo[2]
#define PIN_RR 5 //servo[3]
//---------------------------------------------------------

//---Zowi Buttons
#define PIN_SecondButton 6
#define PIN_ThirdButton 7

///////////////////////////////////////////////////////////////////
//-- Global Zowi Variables --------------------------------------//
///////////////////////////////////////////////////////////////////

const char programID[] = "ZOWI"; //Each program will have a ID

//-- Movement parameters
int T = 1000;              //Initial duration of movement
int moveId = 0;            //Number of movement
int moveSize = 15;         //Asociated with the height of some movements

//---------------------------------------------------------
//-- Zowi has 5 modes:
//--    * MODE = 0: Zowi is awaiting
//--    * MODE = 1: Dancing mode!
//--    * MODE = 2: Obstacle detector mode
//--    * MODE = 3: Noise detector mode
//--    * MODE = 4: ZowiPAD or any Teleoperation mode (listening SerialPort).
//---------------------------------------------------------
volatile int MODE = 0; //State of zowi in the principal state machine.

volatile bool buttonPushed = false; //Variable to remember when a button has been pushed
volatile bool buttonAPushed = false; //Variable to remember when A button has been pushed

unsigned long previousMillis = 0;

int randomDance = 0;
int randomSteps = 0;

bool obstacleDetected = false;

///////////////////////////////////////////////////////////////////
//-- Global MPU9250 Variables -----------------------------------//
///////////////////////////////////////////////////////////////////

#define accelBias0 -77.16f
#define accelBias1 22.60f
#define accelBias2 -1036.47f
#define gyroBias0 -264.60f
#define gyroBias1 142.94f
#define gyroBias2 22.22f

float magCalibration[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};

#define aRes 0.000061037015914f
#define gRes 0.007629627227783f
#define mRes 1.464888477325439f

//-- Yaw Calculation Variables ----------------------------------//

#define nYaw 200
float globalTime;
int yawCounter = 0, initialDirection = 361, meanYaw;
bool yawCalculated = false, startMoving = false;;
float yaw, totalYaw;
float deltat = 0.0f;
uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
#define beta 0.604599857330322f

///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup() {

	//Serial communication initialization
	Serial.begin(38400);

	setupZowi();
	setupMPU9250();

	globalTime = millis();
}

void setupZowi() {
	pinMode(PIN_SecondButton, INPUT);

	//Set the servo pins
	zowi.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true);
	//Set a random seed
	randomSeed(analogRead(A6));

	//Interrumptions
	enableInterrupt(PIN_SecondButton, secondButtonPushed, RISING);

	//Zowi wake up!
	zowi.sing(S_connection);
	zowi.home();

	if (!buttonPushed) {
		zowi.putMouth(happyOpen);
	}

	previousMillis = millis();
}

void setupMPU9250() {
	zowi.initMPU();
}

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
	float actualTime = millis();

	if ((actualTime-globalTime)>15000) {
		float yaw = zowi.getYaw();
		Serial.print("  "); Serial.println(round(yaw));

	}
	else {
		float yaw = zowi.getYaw();
		Serial.println(round(yaw));
	}

	/* After 10s values have stabilized */
	// if ((actualTime-globalTime)>10000) {
	// 	getMeanYaw(yaw);
	// 	if (initialDirection != 361) {
	// 		startMoving = true;
	// 	}
	//
	// 	if (startMoving) {
	// 		zowi.prepareWalking();
	// 		zowi.feetMovement(1, 0);
	// 		for (int i=0; i<200; i++) {
	// 			float yaw = calculateYaw();
	// 			getMeanYaw(yaw);
	// 		}
	// 		int directionDiff = abs(initialDirection-meanYaw);
	// 		int angleCompensation = 30 - directionDiff;
	//
	// 		zowi.feetMovement(1, angleCompensation);
	// 	}
	// }
	//
    // Serial.println(round(yaw));

	//First attemp to initial software
	if (buttonPushed) {

		zowi.home();

		delay(100); //Wait for all buttons
		zowi.sing(S_buttonPushed);
		delay(200); //Wait for all buttons

		if (buttonAPushed) {
			MODE = 1;
			zowi.sing(S_mode1);
		} //else

		zowi.putMouth(MODE);

		int showTime = 2000;
		while ((showTime > 0)) { //Wait to show the MODE number

			showTime -= 10;
			delay(10);
		}

		zowi.putMouth(happyOpen);

		buttonPushed = false;
		buttonAPushed = false;

	}
	else {
		switch (MODE) {
			case 0:

				break;
			case 1:
				zowi.prepareWalking();
				int degreeDiff;
				Serial.println("Introduce dD:");
				while (!Serial.available()) {
					degreeDiff = Serial.parseInt();
				}
				zowi.feetMovement(4, degreeDiff);

				delay(10000);

				break;
			default:
				MODE = 0;
				break;
		}
	}
}

///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
///////////////////////////////////////////////////////////////////

void getMeanYaw(float yaw) {
	yawCounter++;
	totalYaw += yaw;

	/* 200 is a reliable number of times to get the mean value of the yaw angle */
	if (yawCounter == 200) {
		totalYaw /= 200;
		Serial.print("totalYaw: "); Serial.println(totalYaw);
		Serial.print("roundtotalYaw: "); Serial.println(round(totalYaw));
		meanYaw = round(totalYaw);

		/* Initial value is 361 because it can't be reached (yaw angle
		stays between 0 and 360). It is a way to identify if it is the
		first time this function is called. */
		if (initialDirection == 361) {
			initialDirection = round(totalYaw);
		}
		yawCounter = 0;
		totalYaw = 0;
	}
}

//-- Function executed when second button is pushed
void secondButtonPushed() {

	buttonAPushed = true;

	if (!buttonPushed) {
		buttonPushed = true;
		zowi.putMouth(smallSurprise);
	}
}
