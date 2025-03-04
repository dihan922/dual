// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "uart.h"
#include "spi.h"
#include "interrupt.h"
#include "pin_mux_config.h"
#include "utils.h"
#include "prcm.h"
#include "hw_apps_rcm.h"
#include "gpio.h"
#include "hw_nvic.h"
#include "systick.h"

// Common interface includes
#include "uart_if.h"
#include "gpio_if.h"
#include "i2c_if.h"

#include "pin_mux_config.h"
#include "Adafruit_GFX.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.4.0"
#define APP_NAME                "Sliding Ball"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define SPI_IF_BIT_RATE         100000
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

// Color definitions
#define BLACK           0x0000
#define WHITE           0xFFFF
#define RED             0xF800


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned char switch_intflag;     // flag set when switch is pressed

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************

//*****************************************************************************
//
//! Display the buffer contents over I2C
//!
//! \param  pucDataBuf is the pointer to the data store to be displayed
//! \param  ucLen is the length of the data to be displayed
//!
//! \return none
//! 
//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//****************************************************************************
//
//! Reads registers 0x2 - 0x5 corresponding to X/Y acceleration data
//! 
//! This function  
//!    1. Invokes the corresponding I2C APIs
//!
//! \return array containing X and Y byte data
//
//****************************************************************************
int8_t*
ReadAccData()
{
    unsigned char ucRegOffset = 2;
    unsigned char aucRdDataBuf[256];

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(24,&ucRegOffset,1,0));
    
    //
    // Read 4 bytes of successive data
    //
    RET_IF_ERR(I2C_IF_Read(24, &aucRdDataBuf[0], 4));

    //
    // Return the X and Y acceleration data
    //
    static int8_t data[2];
    data[0] = (int8_t)aucRdDataBuf[3];
    data[1] = (int8_t)aucRdDataBuf[1];

    return data;
}

//****************************************************************************
//
//! Checks if switch is pressed
//!
//!
//! \return none
//
//****************************************************************************
static void GPIOA2IntHandler(void) {
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA2_BASE, true);
    MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);        // clear interrupts on GPIOA2

    uint32_t pin_state = MAP_GPIOPinRead(GPIOA2_BASE, 0x40);
    if ((pin_state & 0x40) == 0) // Read falling edge
    {
//        HWREG(NVIC_ST_CURRENT) = 1;
//        systick_cnt = 0;

        switch_intflag = 1;
    }
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function handling the I2C example
//!
//! \param  None
//!
//! \return None
//! 
//*****************************************************************************
void main(){
    int iRetVal;
    char acCmdStore[512];
    unsigned long ulStatus;
    
    //
    // Initialize board configurations
    //
    BoardInit();
    
    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    
    //
    // Configuring UART
    //
    InitTerm();
    
    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    
    //
    // Display the banner followed by the usage description
    //
    DisplayBanner(APP_NAME);

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    // Initialize ship
    int SHIP_SIZE = 16;
    int SCREEN = 128;

    Adafruit_Init();
    fillScreen(BLACK);
    fillRoundRect(56, 56, SHIP_SIZE, SHIP_SIZE, 2, RED);

    int shipPosition[2] = { 56, 56 };
    int8_t shipVelocity[2] = { 0, 0 };

    //
    // Register the interrupt handler
    //
    MAP_GPIOIntRegister(GPIOA2_BASE, GPIOA2IntHandler);

    //
    // Configure falling edge interrupt on switch
    //
    MAP_GPIOIntTypeSet(GPIOA2_BASE, 0x40, GPIO_FALLING_EDGE);
    ulStatus = MAP_GPIOIntStatus(GPIOA2_BASE, false);
    MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);

    // Enable switch interrupt
    MAP_GPIOIntEnable(GPIOA2_BASE, 0x40);
    switch_intflag = 0;

    while (FOREVER)
    {
        if (switch_intflag) {
            switch_intflag = 0;
            Report("Shooting projectile\n\r");
        }
        // Erase ship from the old position
        fillRoundRect(shipPosition[0], shipPosition[1], SHIP_SIZE, SHIP_SIZE, 2, BLACK);

        // Get acceleration data and scale with max acceleration
        int8_t* accData = ReadAccData();
        int8_t xAcc = (int8_t)(((double)accData[0] / 64) * 13);
        int8_t yAcc = (int8_t)(((double)accData[1] / 64) * 13);
//        Report("X Acc: %d, Y Acc: %d\n\r", accData[0], accData[1]);

        // Update velocity based on acceleration and apply friction
        shipVelocity[0] = (shipVelocity[0] + xAcc) * 0.99;
        shipVelocity[1] = (shipVelocity[1] + yAcc) * 0.99;

        // Update position based on velocity
        shipPosition[0] += shipVelocity[0];
        shipPosition[1] += shipVelocity[1];

        // Keep ship position within screen bounds
        if (shipPosition[0] <= 0) {
            shipPosition[0] = 0;
            shipVelocity[0] = 0;
        } else if (shipPosition[0] > SCREEN - SHIP_SIZE - 1) {
            shipPosition[0] = SCREEN - SHIP_SIZE - 1;
            shipVelocity[0] = 0;
        }

        if (shipPosition[1] <= 0) {
            shipPosition[1] = 0;
            shipVelocity[1] = 0;
        } else if (shipPosition[1] > SCREEN - SHIP_SIZE - 1) {
            shipPosition[1] = SCREEN - SHIP_SIZE - 1;
            shipVelocity[1] = 0;
        }

        // Draw ship at the new position
        fillRoundRect(shipPosition[0], shipPosition[1], SHIP_SIZE, SHIP_SIZE, 2, RED);
    }

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @
//
//*****************************************************************************
