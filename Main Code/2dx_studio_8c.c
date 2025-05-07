#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

// === Constants ===
#define I2C_ADDRESS     0x29
#define TOTAL_STEPS     512
#define DEGREE_INTERVAL 16
#define STEP_WAIT       1

// === Globals ===
int tofStatus = 0;
uint32_t stepDelay = STEP_WAIT;
uint16_t rangeVal = 0;

// === I2C Setup for VL53L1X Sensor ===
void ConfigureI2C(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
  while((SYSCTL_PRGPIO_R & 0x02) == 0){};

  GPIO_PORTB_AFSEL_R |= 0x0C;
  GPIO_PORTB_ODR_R |= 0x08;
  GPIO_PORTB_DEN_R |= 0x0C;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) | 0x00002200;

  I2C0_MCR_R = 0x10;
  I2C0_MTPR_R = 0x3B;  // ~100kHz I2C Clock
}

// === Port M – Controls Stepper Motor Pins (PM0–PM3) ===
void InitStepperMotorPort(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
  GPIO_PORTM_DIR_R |= 0x0F;
  GPIO_PORTM_DEN_R |= 0x0F;
  GPIO_PORTM_AFSEL_R &= ~0x0F;
  GPIO_PORTM_AMSEL_R &= ~0x0F;
}

// === Forward Rotation Step Pattern ===
void RotateMotorClockwise(void){
  GPIO_PORTM_DATA_R = 0x03; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x06; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x0C; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x09; SysTick_Wait10ms(stepDelay);
}

// === Reverse Rotation Step Pattern ===
void RotateMotorCounterClockwise(void){
  GPIO_PORTM_DATA_R = 0x09; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x0C; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x06; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x03; SysTick_Wait10ms(stepDelay);
}

// === Button Port PJ1 Setup ===
void SetupButtonPort(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
  while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {};
  GPIO_PORTJ_DIR_R &= ~0x02;
  GPIO_PORTJ_DEN_R |= 0x02;
  GPIO_PORTJ_PCTL_R &= ~0x000000F0;
  GPIO_PORTJ_AMSEL_R &= ~0x02;
  GPIO_PORTJ_PUR_R |= 0x02;
}

// === Sensor Reset Pin Setup (XSHUT on PG0) ===
void SetupSensorResetPort(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){};
  GPIO_PORTG_DIR_R &= ~0x01;
  GPIO_PORTG_AFSEL_R &= ~0x01;
  GPIO_PORTG_DEN_R |= 0x01;
  GPIO_PORTG_AMSEL_R &= ~0x01;
}

// === Reset ToF Sensor Using XSHUT ===
void ResetToFSensor(void){
  GPIO_PORTG_DIR_R |= 0x01;
  GPIO_PORTG_DATA_R &= ~0x01;
  FlashAllLEDs();
  SysTick_Wait10ms(10);
  GPIO_PORTG_DIR_R &= ~0x01;
}

// === Main Program Execution ===
int main(void){
  uint8_t isReady = 0, dataAvailable = 0;

  PLL_Init();                  // Set 24 MHz bus speed
  SysTick_Init();              // Delay timer
  onboardLEDs_Init();          // Initialize status LEDs
  ConfigureI2C();              // I2C peripheral init
  UART_Init();                 // UART for PC communication
  InitStepperMotorPort();      // Setup PM0–PM3
  SetupButtonPort();           // Setup PJ1
  SetupSensorResetPort();      // Optional: if XSHUT needed

  UART_printf("Program Begins\r\n");

  tofStatus = VL53L1X_GetSensorId(I2C_ADDRESS, &rangeVal);
  UART_printf("Sensor ID read\r\n");

  while(isReady == 0){
    tofStatus = VL53L1X_BootState(I2C_ADDRESS, &isReady);
    SysTick_Wait10ms(10);
  }

  FlashAllLEDs();
  UART_printf("ToF Ready\r\n");

  VL53L1X_ClearInterrupt(I2C_ADDRESS);
  VL53L1X_SensorInit(I2C_ADDRESS);
  VL53L1X_StartRanging(I2C_ADDRESS);

  while(1){
    if((GPIO_PORTJ_DATA_R & 0x02) == 0){
      for(int step = 0; step < TOTAL_STEPS; step++){
        RotateMotorClockwise();

        if(step % DEGREE_INTERVAL == 0){
          while(dataAvailable == 0){
            VL53L1X_CheckForDataReady(I2C_ADDRESS, &dataAvailable);
            VL53L1_WaitMs(I2C_ADDRESS, 5);
          }

          dataAvailable = 0;
          VL53L1X_GetDistance(I2C_ADDRESS, &rangeVal);
          FlashLED4(1); // Measurement LED
          FlashLED2(1); // UART transmission indicator

          sprintf(printf_buffer, "%u\r\n", rangeVal);
          UART_printf(printf_buffer);
          VL53L1X_ClearInterrupt(I2C_ADDRESS);
        }
      }

      // Return without measuring
      for(int step = 0; step < TOTAL_STEPS; step++){
        RotateMotorCounterClockwise();
      }

      FlashLED1(1);  // Scan complete indicator
    }
  }

  VL53L1X_StopRanging(I2C_ADDRESS);
}