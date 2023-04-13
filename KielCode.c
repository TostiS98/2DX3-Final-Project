// Stefan Tosti - 400367761
// Meausurement LED - PN1
// Additional Status LED - PF4
// Assigned Bus Speed - 60Hz
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortM0M1M2M3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};         
	GPIO_PORTM_DIR_R = 0b00000000;      								       
  GPIO_PORTM_DEN_R = 0b00001111;  
	return;
}

void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};         
	GPIO_PORTH_DIR_R = 0b00001111;       
  GPIO_PORTH_DEN_R = 0b00001111;  
	return;
}

// THIS IS MY ADDITIONAL STATUS LED 

void PortE0E1E2E3_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // activate the clock for Port E
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; // allow time for clock to stabilize
    GPIO_PORTE_DIR_R = 0b00001111;
    GPIO_PORTE_DEN_R = 0b00001111; // Enable PE0:PE3 
return;
}

void rotationControl(int direction, int step, int delay){

	//counter-clockwise
	if(direction == -1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
		}

	}
	//clockwise
	else if(direction == 1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);

		}
	}

}

// THIS IS MY PRIMARY BLINKY THINGY LED 


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}




uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

// MAIN STARTS HERE
int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	// INITIALIZE ALL PORTS AND PLL AND I2C AND UART AND WHAT NOT 
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM0M1M2M3_Init();
	PortH0H1H2H3_Init();
	PortE0E1E2E3_Init();

	
	// BASICALLY JUST LOAD UP THE TOF SENSOR HERE
	// ********************************************************************************************
	// ********************************************************************************************
	// ********************************************************************************************
	// ********************************************************************************************
	int mynumber = 1;

/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	//sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	//UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	// ********************************************************************************************
	// ********************************************************************************************
	// ********************************************************************************************
	// ********************************************************************************************
	// THIS IS THE END OF THE TIME OF FLIGHT SENSOR INTITIALIZATION


// SET A BUNCH OF CONSTANT VARIABLES 
// DIR -> DIRECTION OF MOTOR TRANSLATION 
// STEPS -> NUMBER OF STEPS IN ONE ROTATION 
// DELAY -> CONSTANT FOR DELAY BETWEEN STEPS IN ROTATION CONTROL
// ROTATE -> TELLS US WHETHER TO ROTATE OR NOT
// COUNT -> INCREMENETED EVERYTIME WE GO THROUGH ANOTHER STEP
// DEPTH -> INCREMENTED AFTER EVERY FULL ROTATION
// SCAN -> TELLS US WHETHER WE SHOULD BE TAKING DATA 
// SCANENABLE -> BUTTON 2, ADDITIONAL STATUS LED, TRANSMIT DATA OR NOT 
	int dir = 1;
	int steps = 512;
	int delay = 2000;
	int rotate = 0;
	int count = 0;
	int depth = 0;
	int scan = 0;
	int scanEnable = 0;
	
	// MAIN LOOP 
	while (1){
		
	// IF BUTTON 1 IS PRESSED, THEN TOGGLE ROTATE
	if((GPIO_PORTM_DATA_R&0x01)==0){
		// BLOCKING STATEMENT TO MITIGATE BOUNCING OF EXTERNAL BUTTONS
		while((GPIO_PORTM_DATA_R&0x01)==0){SysTick_Wait10ms(10);} 
		rotate = !rotate;
		}
	
		// CHECK BUTTON 2, TOGGLE ADDITIONAL STATUS LED AND TOGGLE SCAN ENABLE
	if ((GPIO_PORTM_DATA_R&0b00000010) == 0){
		// NECESSARY BLOCKING STATEMENT
		while((GPIO_PORTM_DATA_R&0b00000010)==0){SysTick_Wait10ms(100);}
		GPIO_PORTF_DATA_R ^= 0b00010000;
		scanEnable = !scanEnable;
	}
		
	// ************************************************************************************************
	// ************************************************************************************************
	// ************************************************************************************************
	// scanEnable coorepnds to additional status LED and tells us whether or not to send data
	// scan = 1 is set later in the main loop everytime we get to 16 steps (every 11.25 degrees)
	if(scan == 1 && scanEnable == 1) {
		
		// check for port 1 button click
		while((GPIO_PORTM_DATA_R&0x01)==0){SysTick_Wait10ms(100);}
		FlashLED3(1);
		
		for(int i = 0; i < 1; i++) {
			
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED3(1);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			//read the data values from ToF sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			status = VL53L1X_GetSignalRate(dev, &SignalRate);
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			status = VL53L1X_GetSpadNb(dev, &SpadNum);
			//status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA)
      //status = VL53L1_RdByte(dev, 0x0110, &byteData); //for module type (0xCC)
      //status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and type

			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			// print the resulted readings to UART
			int step = count;
			if (depth%2 == 1) { // Invert when spinning other direction
				step = 512-count;
			}
			
			// TRANSMIT RANGESTATUS, DISTANCE, STEP, DEPTH, SPADNUM
			sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, step, depth, SpadNum);
			/*sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);*/
			UART_printf(printf_buffer);
			SysTick_Wait10ms(50);
			}
		// DATA TRANSMISSION ENDS HERE LOL 
			
		// SET SCAN TO 0 BECAUSE WE JUST SCANNED 
		scan = 0;
   }
	// NO MO DISTANCE ACQUISITION
	 
	 // IF ROTATE IS 1 THEN MOVE 1 STEP
	if (rotate == 1){
		rotationControl(dir, 1, delay);
		count++;
	}
	
	// EVERY 11.25 DEGREES FLASH THE DATA ACQUISITION LED, WAIT, AND SET SCAN TO 1
	if (count % 16 == 0 && rotate == 1 && count != 0){
		FlashLED1(1);
		SysTick_Wait(10000000); 
		scan = 1; 
	}

	// SWITCH DIRECTIONS AND INCREASE DEPTH
	if (count > 512){
			rotate = 0;
			count = 0;
			dir *= -1;
			depth += 1;
	}
	
	// toggle PE0 for showing our bus speed
	// by probing the amount of time the pin is high and low
	// we can determine the value of one tick based on SysTick_Wait
	// the time of one tick (high -> low) should be 1/bus speed
	// in my case, this should be 1/60MHz
	// this should ideally be 1.667e-8 s = 0.0167 microseconds
	// I want to test this with the scope in the lab 
	// because these values are a little outside of the range of the AD2 
	
	// scope gives us 166.5ms with 10 000 000 in brackets 
	// therefore we get a 0.1665 second delay for 10 000 000 times one frequency 
	// delay for one tick * 10 000 000 = 1.667e-8 * 10 000 000 = 0.1667 seconds 
	
	// UNCOMMENT THE TWO LINES BELOW FOR THE AD2 FREQUENCY DEMO
	//SysTick_Wait(10000000);
	//GPIO_PORTE_DATA_R ^= 0b00000001;

	
	
	
}

	VL53L1X_StopRanging(dev);
  while(1) {}
}