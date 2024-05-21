#include "gd32vf103.h"
//Temperature camera driver
//GD32VF103 implementation of the following driver provoded by the manufacturer.
//No modifications have been done outside MLX90640_I2C_Driver.c/.h
//https://github.com/melexis/mlx90640-library
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "delay.h" //Useful delay function
#include "math.h" //If math functions don't work such as sqrt() or absf()
#include "pwm.h"

#define OPENAIR_TA_SHIFT 8 ///< Default 8 degree offset from ambient air

void get_min_max_float(float* buffer, uint32_t size, float* ret_min, float* ret_max);// Sets minimum and maximum temperature
int getMultiplier(int targetTmp, int currentTmp, int currentMode);					// Outputs the % that the fan pwm will run at
int getHottestPixel(float tmpImage[]);												// Finds the index of the hottest pixel
int getServoAngle(float tmpImage[], float maxTmp, int aOrB);						// Uses the hottest pixel to calculate x and y coords for servo pwm
void switchMode(int *pMode);														// Switches what array for getMultiplier() will use

typedef struct{
	int intervals[8];
}State;

int main(){
	float tr = 23.15; //Reflected temperature
	int cyclecount = 0;
	float emissivity = 0.95;

	int multiplier = 0;
	int runOnce = 1;
	int tragetHeat = 33; // The temperature that the fan is aiming for, max temperature below it stops the fan.
	uint32_t button;
	int mode = 0;
	int btnPress = 0;

	//Parameters involving IR camera
	paramsMLX90640 params;
	uint16_t eeMLX90640[MLX90640_EEPROM_SIZE];
	uint16_t mlx90640Frame[MLX90640_FRAME_SIZE];
	float image[MLX90640_IMAGE_SIZE];
	float min, max;

	//Startup pins
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

	//Initialize I2C bus
	MLX90640_I2CInit();
	//Temperature sensor configuration
	MLX90640_SetResolution(MLX90640_ADDR,MLX90640_RESOLUTION_19BIT);
	MLX90640_SetRefreshRate(MLX90640_ADDR,MLX90640_REFRESHRATE_32HZ);
	MLX90640_SetChessMode(MLX90640_ADDR); //Sensor only updates half of the pixels per frame, this sets the 
	MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
	MLX90640_ExtractParameters(eeMLX90640, &params);

	// Initilize Fan and Servo PWM
	InitPWM();
	InitServo();
	
	while(1){
		//Extract sensor data for current frame
		MLX90640_GetFrameData(MLX90640_ADDR, mlx90640Frame);
		//Get ambient temperature from sensor	
		tr = MLX90640_GetTa(mlx90640Frame, &params) - OPENAIR_TA_SHIFT; 
		//Calculate temperatures from raw sensor data
		MLX90640_CalculateTo(mlx90640Frame, &params, emissivity, tr, image);
		//Correct broken pixels and outlier values
		MLX90640_BadPixelsCorrection((&params)->brokenPixels, image, 1, &params);
		MLX90640_BadPixelsCorrection((&params)->outlierPixels, image, 1, &params);

		//At this point "image" contains the measured temperatures for each pixel. 
		//A pixels adress can be calculated by multiplying it's x and y position.
		//So a pixel with coordinate x=10, y=12 would be found in image[10*12]
		
		//Extract maximum and minimum temperature in image
		get_min_max_float(image, (24*32)-1, &min, &max);

		if (runOnce == 1){
			runOnce = 0;
			MoveServoA(1500);
			MoveServoB(1500);
		}

		delay_1ms(1000);

		// Multiplier based on temperature difference
		multiplier = getMultiplier(tragetHeat, max, mode);	
		FanPWMch0(160 * multiplier); // max: input is 16000,  ex: 160 * 100 = 16000

		// Get servo PWM based on hottest pixel position
		int servoA_angle = getServoAngle(image, max, 1);
		int servoB_angle = getServoAngle(image, max, 0);

		// Move servo with PWM
		MoveServoA(servoA_angle);
		MoveServoB(servoB_angle);

		// Button input from user
		button = gpio_input_bit_get(GPIOB, GPIO_PIN_0);
		if(button == 1){
			if (!btnPress){
				switchMode(&mode);
				btnPress = 1;
			}
		} else {
			btnPress = 0;
		}
	}
}

// ------------------- Camera Functions -------------------
//Some helper functions
void get_min_max_float(float* buffer, uint32_t size, float* ret_min, float* ret_max){
	*ret_min = __FLT_MAX__;
	*ret_max = -__FLT_MAX__;
	for(int i = 0; i < size; i++){
		if(*ret_max < buffer[i]) *ret_max = buffer[i]; 
		if(*ret_min > buffer[i]) *ret_min = buffer[i]; 
	}
}

// ------------------- Fan Functions -------------------

void switchMode(int *pMode){
	*pMode++;
	if (*pMode >= 3){
		*pMode = 0;
	}
}

int getMultiplier(int targetTmp, int currentTmp, int currentMode){
	State states[3] = {{10,10,25,30,50,75,100,100}, {5,10,20,25,30,40,45,50}, {20,30,50,80,100,100,100,100}};

	int tmpDiff = currentTmp - targetTmp;

	if (tmpDiff <= 0){
		return 0;
	}

	if (tmpDiff > 8){
		return states[currentMode].intervals[7];
	}
	return states[currentMode].intervals[tmpDiff];
}

// ------------------- Servo Functions -------------------

int getHottestPixel(float tmpImage[]){
    float maxTemp = tmpImage[0];
    int hottestPixelIndex = 0;

	// Searching for hottest pixel
    for (int i = 1; i < MLX90640_IMAGE_SIZE; i++) {
        if (tmpImage[i] > maxTemp) {
            maxTemp = tmpImage[i];
            hottestPixelIndex = i;
        }
    }

    return hottestPixelIndex;
}

int getServoAngle(float tmpImage[], float maxTmp, int aOrB){
	if (tmpImage[16*12] < maxTmp){
		int hottestPixelIndex = getHottestPixel(tmpImage);

		// Hottest pixel position in the image
		int hottestPixelX = hottestPixelIndex % 32;
		int hottestPixelY = hottestPixelIndex / 32;

		// Preset values for the servo pwm, 2000 = 180 degrees and 1000 = 0 degrees
		int diffY[24] = {2000, 1950, 1900, 1850, 1800, 1750, 1700, 1650, 1600, 1590, 1550, 
						1500, 1490, 1450, 1400, 1390, 1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000};

		int diffX[32] = {1000, 1030, 1050, 1095, 1100, 1150, 1190, 1200, 1250, 1290,
						1300, 1350, 1400, 1450, 1500,  1550, 1590, 1600, 1650, 1690, 1700, 1750, 1790, 
						1800, 1850, 1890, 1900, 1930, 1950, 1990, 1000};

		// Conditions on picking out the the pwm for respective servo
		if(aOrB == 1){
			if (hottestPixelX <= 0) {
				return diffX[0];
			} else if (hottestPixelX >= 32){
				return diffX[31];
			} else{
				return diffX[hottestPixelX];
			}
		} else{
			if (hottestPixelY <= 0) {
				return diffY[0];
			} else if (hottestPixelY >= 24){
				return diffY[23];
			} else{
				return diffY[hottestPixelY];
			}
		}

	} else {
		// Center servo if the center pixel is the hottest spot
		return 1500;
	}
}