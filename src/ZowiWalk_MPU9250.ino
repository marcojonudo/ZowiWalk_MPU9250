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
#include "Wire.h"

#include <Servo.h>
#include <Oscillator.h>
#include <EEPROM.h>
#include <BatReader.h>
#include <US.h>
#include <LedMatrix.h>

//-- Library to manage external interruptions
#include <EnableInterrupt.h>

//-- Library to manage serial commands
#include <ZowiSerialCommand.h>
ZowiSerialCommand SCmd;  //The SerialCommand object

//-- Zowi Library
#include <Zowi.h>
Zowi zowi;  //This is Zowi!!

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

const char programID[] = "ZOWI_BASE_v2"; //Each program will have a ID

const char name_fac = '$'; //Factory name
const char name_fir = '#'; //First name

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
volatile bool buttonBPushed = false; //Variable to remember when B button has been pushed

unsigned long previousMillis = 0;

int randomDance = 0;
int randomSteps = 0;

bool obstacleDetected = false;

///////////////////////////////////////////////////////////////////
//-- Global MPU9250 Variables -----------------------------------//
///////////////////////////////////////////////////////////////////

#define MPU9250_ADDRESS 0x68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define FIFO_COUNTH      0x72
#define FIFO_R_W         0x74
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value

uint8_t Gscale = 0;
uint8_t Ascale = 0;
uint8_t Mscale = 1; // 16-bit magnetometer resolution
uint8_t Mmode = 0x06;

int16_t accelCount[3], gyroCount[3], magCount[3];

float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float mRes = 10.*4800.0/32767.0; // Proper scale to return milliGauss
float aRes = 2.0/32767.0;
float gRes = 250.0/32767.0;

float ax, ay, az, gx, gy, gz, mx, my, mz;

//-- Yaw Calculation Variables ----------------------------------//

float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float a12, a22, a31, a32, a33;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta


///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup() {

	//Serial communication initialization
	Serial.begin(38400);

	setupZowi();
	setupMPU9250();

}

void setupZowi() {
	pinMode(PIN_SecondButton, INPUT);
	pinMode(PIN_ThirdButton, INPUT);

	//Set the servo pins
	zowi.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true);

	//Uncomment this to set the servo trims manually and save on EEPROM
	//zowi.setTrims(TRIM_YL, TRIM_YR, TRIM_RL, TRIM_RR);
	//zowi.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.

	//Set a random seed
	randomSeed(analogRead(A6));

	//Interrumptions
	enableInterrupt(PIN_SecondButton, secondButtonPushed, RISING);

	//Setup callbacks for SerialCommand commands
	SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
	SCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
	SCmd.addCommand("T", recieveBuzzer);    //  sendAck & sendFinalAck
	SCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
	SCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
	SCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
	SCmd.addCommand("C", receiveTrims);     //  sendAck & sendFinalAck
	SCmd.addCommand("G", receiveServo);     //  sendAck & sendFinalAck
	SCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck
	SCmd.addCommand("E", requestName);
	SCmd.addCommand("D", requestDistance);
	SCmd.addCommand("N", requestNoise);
	SCmd.addCommand("B", requestBattery);
	SCmd.addCommand("I", requestProgramId);
	SCmd.addDefaultHandler(receiveStop);

	//Zowi wake up!
	zowi.sing(S_connection);
	zowi.home();

	//If Zowi's name is '&' (factory name) means that is the first time this program is executed.
	//This first time, Zowi mustn't do anything. Just born at the factory!
	//5 = EEPROM address that contains first name character
	if (EEPROM.read(5) == name_fac) {

		EEPROM.put(5, name_fir); //From now, the name is '#'
		EEPROM.put(6, '\0');
		zowi.putMouth(culito);

		while (true) {
			delay(1000);
		}
	}

	//Send Zowi name, programID & battery level.
	requestName();
	delay(50);
	requestProgramId();
	delay(50);
	requestBattery();

	//Checking battery
	//ZowiLowBatteryAlarm();

	// Animation Uuuuuh - A little moment of initial surprise
	//-----
	for (int i = 0; i < 2; i++) {
		for (int i = 0; i < 8; i++) {
			if (buttonPushed) {
				break;
			}
			zowi.putAnimationMouth(littleUuh, i);
			delay(150);
		}
	}
	//-----

	//Smile for a happy Zowi :)
	if (!buttonPushed) {
		zowi.putMouth(smile);
		zowi.sing(S_happy);
		delay(200);
	}

	//If Zowi's name is '#' means that Zowi hasn't been baptized
	//In this case, Zowi does a longer greeting
	//5 = EEPROM address that contains first name character
	if (EEPROM.read(5) == name_fir) {

		if (!buttonPushed) {
			zowi.jump(1, 700);
			delay(200);
		}

		if (!buttonPushed) {
			zowi.shakeLeg(1, T, 1);
		}

		if (!buttonPushed) {
			zowi.putMouth(smallSurprise);
			zowi.swing(2, 800, 20);
			zowi.home();
		}
	}

	if (!buttonPushed) {
		zowi.putMouth(happyOpen);
	}

	previousMillis = millis();
}

void setupMPU9250() {
	Wire.begin();

	Serial.println("MPU9250 9-axis motion sensor...");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

    if (c == 0x71) {
        Serial.println("MPU9250 is online...");

        // calibrateAccelGyro(accelBias, gyroBias);
		/* Acceleromter and gyroscope offsets. Could be advisable to calculate them on the fly */
        accelBias[0] = 66.00;
        accelBias[1] = 264.08;
        accelBias[2] = -1295.76;
        gyroBias[0] = -280.54;
        gyroBias[1] = 131.13;
        gyroBias[2] = 8.31;

        // Serial.println("MPU9250 accel biases (g)"); Serial.println(accelBias[0]); Serial.println(accelBias[1]); Serial.println(accelBias[2]);
        // Serial.println("MPU9250 gyro biases (deg/s)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
        // delay(1000);

        initMPU9250();
        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
        Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);
        Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

		// magcalMPU9250(magBias, magScale);
        /* Valores con el MPU1, correspondientes al archivo 'MPU_YawAngle_Uncalibrated_Calibracion_Zowi' */
        /* Los 3 planos salen casi perfectos en esa figura */
        magBias[0] = 180.35;
        magBias[1] = -104.66;
        magBias[2] = 71.38;
        magScale[0] = 1.04;
        magScale[1] = 1.01;
        magScale[2] = 0.95;

        // Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
        // Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
        // delay(1000);
    }
}

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
	Serial.println("bien");

	if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the accleration value into actual g's
        ax = ((float)accelCount[0] - accelBias[0])*aRes;  // get actual g value, this depends on scale being set
        ay = ((float)accelCount[1] - accelBias[1])*aRes;
        az = ((float)accelCount[2] - accelBias[2])*aRes;
       Serial.print("ax = "); Serial.print(ax);
       Serial.print(" ay = "); Serial.print(ay);
       Serial.print(" az = "); Serial.print(az); Serial.println(" g");

        readGyroData(gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        gx = ((float)gyroCount[0] - gyroBias[0])*gRes;  // get actual gyro value, this depends on scale being set
        gy = ((float)gyroCount[1] - gyroBias[1])*gRes;
        gz = ((float)gyroCount[2] - gyroBias[2])*gRes;
    //    Serial.print("gx = "); Serial.print(gx);
    //    Serial.print(" gy = "); Serial.print(gy);
    //    Serial.print(" gz = "); Serial.print(gz); Serial.println(" deg/s");

        readMagData(magCount);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];
//        Serial.print(mx); Serial.print(",");
//        Serial.print(my); Serial.print(",");
//        Serial.println(mz); //Serial.print("\t");
    }

	if (Serial.available() > 0 && MODE != 4) {

		MODE = 4;
		zowi.putMouth(happyOpen);

		//Disable Pin Interruptions
		disableInterrupt(PIN_SecondButton);
		disableInterrupt(PIN_ThirdButton);

		buttonPushed = false;
	}

	//First attemp to initial software
	if (buttonPushed) {

		zowi.home();

		delay(100); //Wait for all buttons
		zowi.sing(S_buttonPushed);
		delay(200); //Wait for all buttons

		if (buttonAPushed && !buttonBPushed) {
			MODE = 1;
			zowi.sing(S_mode1);
		} else if (!buttonAPushed && buttonBPushed) {
			MODE = 2;
			zowi.sing(S_mode2);
		} else if (buttonAPushed && buttonBPushed) {
			MODE = 3;
			zowi.sing(S_mode3);
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
		buttonBPushed = false;

	} else {

		switch (MODE) {

		//-- MODE 0 - Zowi is awaiting
		//---------------------------------------------------------
		case 0:

			//Every 80 seconds in this mode, Zowi falls asleep

			break;

			//-- MODE 1 - Dance Mode!
			//---------------------------------------------------------
		case 1:

			/*zowi.moveFeet(90,120);
			 zowi.moveHip(90,70);
			 zowi.moveHip(90,90);*/

			//zowi.walk(1,1000,1);
			//zowi.turnWithoutMoving(1000,1);
			zowi.feetMovement();

			break;

			//-- MODE 2 - Obstacle detector mode
			//---------------------------------------------------------
		case 2:

			buttonPushed = false;
			buttonAPushed = false;
			buttonBPushed = false;
			MODE = 4;

			break;
			//-- MODE 3 - Noise detector mode
			//---------------------------------------------------------
		case 3:

			break;
			//-- MODE 4 - ZowiPAD or any Teleoperation mode (listening SerialPort)
			//---------------------------------------------------------
		case 4:

			SCmd.readSerial();

			break;

		default:
			MODE = 4;
			break;
		}

	}

}

///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
///////////////////////////////////////////////////////////////////

void initMPU9250() {
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    delay(100); // Wait for all registers to reset

    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                  // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    //   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}

void initAK8963(float * destination) {
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    delay(10);
}

void readAccelData(int16_t * destination) {
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readGyroData(int16_t * destination) {
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination) {
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
        }
    }
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;                            // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }         // Put read results in the Rx buffer
}


//-- Function executed when second button is pushed
void secondButtonPushed() {

	buttonAPushed = true;

	if (!buttonPushed) {
		buttonPushed = true;
		zowi.putMouth(smallSurprise);
	}
}

//-- Function to receive Stop command.
void receiveStop() {

	sendAck();
	zowi.home();
	sendFinalAck();

}

//-- Function to receive LED commands
void receiveLED() {

	//sendAck & stop if necessary
	sendAck();
	zowi.home();

	//Examples of receiveLED Bluetooth commands
	//L 000000001000010100100011000000000
	//L 001111111111111111111111111111111 (todos los LED encendidos)
	unsigned long int matrix;
	char *arg;
	char *endstr;
	arg = SCmd.next();
	//Serial.println (arg);
	if (arg != NULL) {
		matrix = strtoul(arg, &endstr, 2); // Converts a char string to unsigned long integer
		zowi.putMouth(matrix, false);
	} else {
		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();
	}

	sendFinalAck();

}

//-- Function to receive buzzer commands
void recieveBuzzer() {

	//sendAck & stop if necessary
	sendAck();
	zowi.home();

	bool error = false;
	int frec;
	int duration;
	char *arg;

	arg = SCmd.next();
	if (arg != NULL) {
		frec = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		duration = atoi(arg);
	} // Converts a char string to an integer
	else {
		error = true;
	}

	if (error == true) {

		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();

	} else {

		zowi._tone(frec, duration, 1);
	}

	sendFinalAck();

}

//-- Function to receive TRims commands
void receiveTrims() {

	//sendAck & stop if necessary
	sendAck();
	zowi.home();

	int trim_YL, trim_YR, trim_RL, trim_RR;

	//Definition of Servo Bluetooth command
	//C trim_YL trim_YR trim_RL trim_RR
	//Examples of receiveTrims Bluetooth commands
	//C 20 0 -8 3
	bool error = false;
	char *arg;
	arg = SCmd.next();
	if (arg != NULL) {
		trim_YL = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		trim_YR = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		trim_RL = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		trim_RR = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	if (error == true) {

		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();

	} else { //Save it on EEPROM
		zowi.setTrims(trim_YL, trim_YR, trim_RL, trim_RR);
		zowi.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.
	}

	sendFinalAck();

}

//-- Function to receive Servo commands
void receiveServo() {

	sendAck();
	moveId = 30;

	//Definition of Servo Bluetooth command
	//G  servo_YL servo_YR servo_RL servo_RR
	//Example of receiveServo Bluetooth commands
	//G 90 85 96 78
	bool error = false;
	char *arg;
	int servo_YL, servo_YR, servo_RL, servo_RR;

	arg = SCmd.next();
	if (arg != NULL) {
		servo_YL = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		servo_YR = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		servo_RL = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		servo_RR = atoi(arg);
	}    // Converts a char string to an integer
	else {
		error = true;
	}

	if (error == true) {

		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();

	} else { //Update Servo:

		int servoPos[4] = { servo_YL, servo_YR, servo_RL, servo_RR };
		zowi._moveServos(200, servoPos);   //Move 200ms

	}

	sendFinalAck();

}

//-- Function to receive movement commands
void receiveMovement() {

	sendAck();

	if (zowi.getRestState() == true) {
		zowi.setRestState(false);
	}

	//Definition of Movement Bluetooth commands
	//M  MoveID  T   MoveSize
	char *arg;
	arg = SCmd.next();
	if (arg != NULL) {
		moveId = atoi(arg);
	} else {
		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();
		moveId = 0; //stop
	}

	arg = SCmd.next();
	if (arg != NULL) {
		T = atoi(arg);
	} else {
		T = 1000;
	}

	arg = SCmd.next();
	if (arg != NULL) {
		moveSize = atoi(arg);
	} else {
		moveSize = 15;
	}
}


//-- Function to receive gesture commands
void receiveGesture() {

	//sendAck & stop if necessary
	sendAck();
	zowi.home();

	//Definition of Gesture Bluetooth commands
	//H  GestureID
	int gesture = 0;
	char *arg;
	arg = SCmd.next();
	if (arg != NULL) {
		gesture = atoi(arg);
	} else {
		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();
	}

	switch (gesture) {
	case 1: //H 1
		zowi.playGesture(ZowiHappy);
		break;
	case 2: //H 2
		zowi.playGesture(ZowiSuperHappy);
		break;
	case 3: //H 3
		zowi.playGesture(ZowiSad);
		break;
	case 4: //H 4
		zowi.playGesture(ZowiSleeping);
		break;
	case 5: //H 5
		zowi.playGesture(ZowiFart);
		break;
	case 6: //H 6
		zowi.playGesture(ZowiConfused);
		break;
	case 7: //H 7
		zowi.playGesture(ZowiLove);
		break;
	case 8: //H 8
		zowi.playGesture(ZowiAngry);
		break;
	case 9: //H 9
		zowi.playGesture(ZowiFretful);
		break;
	case 10: //H 10
		zowi.playGesture(ZowiMagic);
		break;
	case 11: //H 11
		zowi.playGesture(ZowiWave);
		break;
	case 12: //H 12
		zowi.playGesture(ZowiVictory);
		break;
	case 13: //H 13
		zowi.playGesture(ZowiFail);
		break;
	default:
		break;
	}

	sendFinalAck();
}

//-- Function to receive sing commands
void receiveSing() {

	//sendAck & stop if necessary
	sendAck();
	zowi.home();

	//Definition of Sing Bluetooth commands
	//K  SingID
	int sing = 0;
	char *arg;
	arg = SCmd.next();
	if (arg != NULL) {
		sing = atoi(arg);
	} else {
		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();
	}

	switch (sing) {
	case 1: //K 1
		zowi.sing(S_connection);
		break;
	case 2: //K 2
		zowi.sing(S_disconnection);
		break;
	case 3: //K 3
		zowi.sing(S_surprise);
		break;
	case 4: //K 4
		zowi.sing(S_OhOoh);
		break;
	case 5: //K 5
		zowi.sing(S_OhOoh2);
		break;
	case 6: //K 6
		zowi.sing(S_cuddly);
		break;
	case 7: //K 7
		zowi.sing(S_sleeping);
		break;
	case 8: //K 8
		zowi.sing(S_happy);
		break;
	case 9: //K 9
		zowi.sing(S_superHappy);
		break;
	case 10: //K 10
		zowi.sing(S_happy_short);
		break;
	case 11: //K 11
		zowi.sing(S_sad);
		break;
	case 12: //K 12
		zowi.sing(S_confused);
		break;
	case 13: //K 13
		zowi.sing(S_fart1);
		break;
	case 14: //K 14
		zowi.sing(S_fart2);
		break;
	case 15: //K 15
		zowi.sing(S_fart3);
		break;
	case 16: //K 16
		zowi.sing(S_mode1);
		break;
	case 17: //K 17
		zowi.sing(S_mode2);
		break;
	case 18: //K 18
		zowi.sing(S_mode3);
		break;
	case 19: //K 19
		zowi.sing(S_buttonPushed);
		break;
	default:
		break;
	}

	sendFinalAck();
}

//-- Function to receive Name command
void receiveName() {

	//sendAck & stop if necessary
	sendAck();
	zowi.home();

	char newZowiName[11] = "";  //Variable to store data read from Serial.
	int eeAddress = 5;          //Location we want the data to be in EEPROM.
	char *arg;
	arg = SCmd.next();

	if (arg != NULL) {

		//Complete newZowiName char string
		int k = 0;
		while ((*arg) && (k < 11)) {
			newZowiName[k] = *arg++;
			k++;
		}

		EEPROM.put(eeAddress, newZowiName);
	} else {
		zowi.putMouth(xMouth);
		delay(2000);
		zowi.clearMouth();
	}

	sendFinalAck();

}

//-- Function to send Zowi's name
void requestName() {

	zowi.home(); //stop if necessary

	char actualZowiName[11] = "";  //Variable to store data read from EEPROM.
	int eeAddress = 5;            //EEPROM address to start reading from

	//Get the float data from the EEPROM at position 'eeAddress'
	EEPROM.get(eeAddress, actualZowiName);

	Serial.print(F("&&"));
	Serial.print(F("E "));
	Serial.print(actualZowiName);
	Serial.println(F("%%"));
	Serial.flush();
}

//-- Function to send ultrasonic sensor measure (distance in "cm")
void requestDistance() {

	zowi.home();  //stop if necessary

	int distance = zowi.getDistance();
	Serial.print(F("&&"));
	Serial.print(F("D "));
	Serial.print(distance);
	Serial.println(F("%%"));
	Serial.flush();
}

//-- Function to send noise sensor measure
void requestNoise() {

	zowi.home();  //stop if necessary

	int microphone = zowi.getNoise(); //analogRead(PIN_NoiseSensor);
	Serial.print(F("&&"));
	Serial.print(F("N "));
	Serial.print(microphone);
	Serial.println(F("%%"));
	Serial.flush();
}

//-- Function to send battery voltage percent
void requestBattery() {

	zowi.home();  //stop if necessary

	//The first read of the batery is often a wrong reading, so we will discard this value.
	double batteryLevel = zowi.getBatteryLevel();

	Serial.print(F("&&"));
	Serial.print(F("B "));
	Serial.print(batteryLevel);
	Serial.println(F("%%"));
	Serial.flush();
}

//-- Function to send program ID
void requestProgramId() {

	zowi.home();   //stop if necessary

	Serial.print(F("&&"));
	Serial.print(F("I "));
	Serial.print(programID);
	Serial.println(F("%%"));
	Serial.flush();
}

//-- Function to send Ack comand (A)
void sendAck() {

	delay(30);

	Serial.print(F("&&"));
	Serial.print(F("A"));
	Serial.println(F("%%"));
	Serial.flush();
}

//-- Function to send final Ack comand (F)
void sendFinalAck() {

	delay(30);

	Serial.print(F("&&"));
	Serial.print(F("F"));
	Serial.println(F("%%"));
	Serial.flush();
}
