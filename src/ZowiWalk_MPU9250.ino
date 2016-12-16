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

#define MPU9250_ADDRESS 0x68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
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

#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value

#define accelBias0 -77.16f
#define accelBias1 22.60f
#define accelBias2 -1036.47f
#define gyroBias0 -264.60f
#define gyroBias1 142.94f
#define gyroBias2 22.22f
#define magBias0 180.35f
#define magBias1 -104.66f
#define magBias2 71.38f
#define magScale0 1.04f
#define magScale1 1.01f
#define magScale2 0.95f

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
	Wire.begin();

    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    if (c == 0x71) {
		// float accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0};
        // calibrateAccelGyro(accelBias, gyroBias);
		// Serial.print(accelBias[0]); Serial.print(","); Serial.print(accelBias[1]); Serial.print(","); Serial.println(accelBias[2]);
		// Serial.print(gyroBias[0]); Serial.print(","); Serial.print(gyroBias[1]); Serial.print(","); Serial.println(gyroBias[2]);
		// delay(10000);

        initMPU9250();

        // byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
        // Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);

		// magcalMPU9250(magBias, magScale);
        /* Valores con el MPU1, correspondientes al archivo 'MPU_YawAngle_Uncalibrated_Calibracion_Zowi' */
        /* Los 3 planos salen casi perfectos en esa figura */


        // Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
        // Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
        // delay(1000);
    }
}

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
	float yaw = calculateYaw();

	float actualTime = millis();

	/* After 10s values have stabilized */
	if ((actualTime-globalTime)>10000) {
		getMeanYaw(yaw);
		if (initialDirection != 361) {
			startMoving = true;
		}

		Serial.println(startMoving);
		Serial.println(initialDirection);

		if (startMoving) {
			zowi.prepareWalking();
			zowi.feetMovement(1, 0);
			/* 25 meausres of margin to stabilise values */
			for (int i=0; i<25; i++) {
				float yaw = calculateYaw();
			}
			for (int i=0; i<200; i++) {
				float yaw = calculateYaw();
				getMeanYaw(yaw);
			}
			int directionDiff = abs(initialDirection-meanYaw);
			int angleCompensation = 30 - directionDiff;

			zowi.feetMovement(1, angleCompensation);
		}
	}

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
				// int dD;
				// while (Serial.available()<=0) {
				// 	dD = Serial.read();
				// }
				zowi.feetMovement(3, -4);

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

float calculateYaw() {
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
		int16_t accelCount[3], gyroCount[3], magCount[3];

		readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the accleration value into actual g's
        ax = ((float)accelCount[0] - accelBias[0])*aRes;  // get actual g value, this depends on scale being set
        ay = ((float)accelCount[1] - accelBias[1])*aRes;
        az = ((float)accelCount[2] - accelBias[2])*aRes;
		// Serial.print("ax = "); Serial.print(ax);
		// Serial.print(" ay = "); Serial.print(ay);
		// Serial.print(" az = "); Serial.print(az); Serial.println(" g");

        readGyroData(gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        gx = ((float)gyroCount[0] - gyroBias[0])*gRes;  // get actual gyro value, this depends on scale being set
        gy = ((float)gyroCount[1] - gyroBias[1])*gRes;
        gz = ((float)gyroCount[2] - gyroBias[2])*gRes;
		// Serial.print("gx = "); Serial.print(gx);
		// Serial.print(" gy = "); Serial.print(gy);
		// Serial.print(" gz = "); Serial.print(gz); Serial.println(" deg/s");

        readMagData(magCount);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias0;  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magBias1;
        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias2;
        mx *= magScale0;
        my *= magScale1;
        mz *= magScale2;
		// Serial.print(mx); Serial.print(",");
		// Serial.print(my); Serial.print(",");
		// Serial.println(mz); //Serial.print("\t");
    }

	Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    /* Al darse la vuelta el sensor, cambia la convención de ejes */
	// MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz);
    MadgwickQuaternionUpdate(-ax, -ay, -az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, -mz);

    // pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
    // roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    // pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    if (yaw < 0) yaw += 360.0f; // Ensure yaw stays between 0 and 360
    // roll  *= 180.0f / PI;

	return yaw;
}

void calibrateAccelGyro(float * dest1, float * dest2) {
    int                   num_readings = 100;
    float                 x_accel = 0;
    float                 y_accel = 0;
    float                 z_accel = 0;
    float                 x_gyro = 0;
    float                 y_gyro = 0;
    float                 z_gyro = 0;
	int16_t accelCount[3], gyroCount[3], magCount[3];
	delay(3000);

    readAccelData(accelCount);
    readGyroData(gyroCount);

    // Read and average the raw values from the IMU
    for (int i = 0; i < num_readings; i++) {
        readAccelData(accelCount);
        readGyroData(gyroCount);
        x_accel += accelCount[0];
        y_accel += accelCount[1];
        z_accel += accelCount[2];
        x_gyro += gyroCount[0];
        y_gyro += gyroCount[1];
        z_gyro += gyroCount[2];
        delay(100);
    }
    x_accel /= num_readings;
    y_accel /= num_readings;
    z_accel /= num_readings;
    x_gyro /= num_readings;
    y_gyro /= num_readings;
    z_gyro /= num_readings;

    dest1[0] = x_accel;
    dest1[1] = y_accel;
    dest1[2] = (z_accel+16384.0);
    dest2[0] = x_gyro;
    dest2[1] = y_gyro;
    dest2[2] = z_gyro;
}

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
    c = c | 0 << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | 0 << 3; // Set full scale range for the accelerometer
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
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x16); // Set magnetometer data resolution and sample ODR
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
