// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

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
#define APPLICATION_VERSION     "SQ25"
#define APP_NAME                "DUAL"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define UART1                   UARTA1_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define SPI_IF_BIT_RATE         100000
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
//
// Player mode selector macro
//
// MASTER_MODE = 1 : Player is RED
// MASTER_MODE = 0 : Player is BLUE
//
//*****************************************************************************
#define PLAYER_MODE 1

// Color definitions
#define BLACK           0x0000
#define WHITE           0xFFFF
#define RED             0xF800
#define BLUE            0x001F

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

volatile int timer = 0;
volatile int systick_cnt = 0;
volatile int projectile_size = 2;
volatile int projectile_scale = 1;
volatile int ammo_cnt = 6;

// Initialize projectile
#define MAX_PROJECTILES 6

typedef struct {
    volatile int  x_position;
    volatile int  y_position;
    volatile int  x_velocity;
    volatile int  y_velocity;
    volatile bool state;
    volatile int  size;
} Projectile;

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
static void GPIOA1IntHandler(void) {
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus(GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);        // clear interrupts on GPIOA1

    uint32_t pin_state = MAP_GPIOPinRead(GPIOA1_BASE, 0x20);
    if ((pin_state & 0x20) == 0) { // Read falling edge
        switch_intflag = 1;
    }
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    uint32_t pin_state = MAP_GPIOPinRead(GPIOA1_BASE, 0x20);
    static bool charging = false;
    static int starting_ammo = 6;  // Stores ammo count when button is first pressed

    if ((pin_state & 0x20) != 0) {  // Button is held down
        systick_cnt++;

        if (!charging) {
            // Reset projectile_scale only when first pressed
            projectile_scale = 1;
            starting_ammo = ammo_cnt;  // Capture ammo count at start
            charging = true;
        }

        // Scale projectile size based on ammo spent
        if (ammo_cnt > 0 && (systick_cnt % 15) == 0) {
            ammo_cnt--;

            // Ensure projectile_scale correctly follows the ammo depletion
            projectile_scale = (starting_ammo - ammo_cnt);
            if (projectile_scale > 6) {
                projectile_scale = 6;
            }
        }
        projectile_size = projectile_scale * 2;
    }
    else {  // Button is released
        charging = false;
    }

    timer++;
}


void drawShipWithAmmo(int x, int y, int size, int color, int ammo) {
    int ammo_size = size / 6;
    int padding = size / 8;  // Space between ammo squares

    // Draw the ship
    fillRect(x, y, size, size, color);

    // Draw ammo squares based on ammo count
    if (ammo >= 1) fillRect(x + size + padding, y + size - ammo_size, ammo_size, ammo_size, color);
    if (ammo >= 2) fillRect(x - ammo_size - padding, y + size - ammo_size, ammo_size, ammo_size, color);
    if (ammo >= 3) fillRect(x + size + padding, y + size - (2 * ammo_size) - padding, ammo_size, ammo_size, color);
    if (ammo >= 4) fillRect(x - ammo_size - padding, y + size - (2 * ammo_size) - padding, ammo_size, ammo_size, color);
    if (ammo >= 5) fillRect(x + size + padding, y + size - (3 * ammo_size) - (2 * padding), ammo_size, ammo_size, color);
    if (ammo >= 6) fillRect(x - ammo_size - padding, y + size - (3 * ammo_size) - (2 * padding), ammo_size, ammo_size, color);
}


void InitUART1(void)
{
    // Configure UART1 for 115200 baud, 8 data bits, 1 stop bit, no parity
    MAP_UARTConfigSetExpClk(UART1,
                            MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                            115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    MAP_UARTFIFOEnable(UART1);
    MAP_UARTEnable(UART1);
}

void UART1SendProjectile(Projectile *proj)
{
    char buffer[11];

    // Pack the projectile data into the buffer
    buffer[0] = (proj->x_position >> 8) & 0xFF;  // High byte of x_position
    buffer[1] = proj->x_position & 0xFF;         // Low byte of x_position

    buffer[2]  = (proj->x_velocity >> 24) & 0xFF;
    buffer[3]  = (proj->x_velocity >> 16) & 0xFF;
    buffer[4]  = (proj->x_velocity >> 8) & 0xFF;
    buffer[5]  = proj->x_velocity & 0xFF;

    buffer[6]  = (proj->y_velocity >> 24) & 0xFF;
    buffer[7]  = (proj->y_velocity >> 16) & 0xFF;
    buffer[8]  = (proj->y_velocity >> 8) & 0xFF;
    buffer[9]  = proj->y_velocity & 0xFF;

    buffer[10] = proj->size;  // Projectile size (1 byte)

    // Send each byte through UART1
    int i;
    for (i = 0; i < 11; i++)
    {
        while (MAP_UARTBusy(UART1)); // Wait until UART is ready
        MAP_UARTCharPut(UART1, buffer[i]);
    }
}

void UART1ReceiveProjectile(Projectile *proj)
{
    char buffer[11];  // Buffer to store received data
    int index = 0;

    // Wait until we have received exactly 12 bytes
    while (index < 11)
    {
        if (MAP_UARTCharsAvail(UART1))
        {
            buffer[index++] = MAP_UARTCharGetNonBlocking(UART1);
        }
    }

    // Reconstruct projectile data from received bytes
    proj->x_position = 128 - ((buffer[0] << 8) | buffer[1]);
    proj->y_position = 128 - buffer[10];
    proj->x_velocity = ((buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5]) * -1;
    proj->y_velocity = ((buffer[6] << 24) | (buffer[7] << 16) | (buffer[8] << 8) | buffer[9]) * -1;
    proj->size = buffer[10];
    proj->state = true;
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

/**
 * Initializes SysTick Module
 */
 static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
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

    InitUART1();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Enable SysTick
    SysTickInit();

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
    int SHIP_SIZE = 24;
    int PLAYER_COLOR = (PLAYER_MODE) ? RED : BLUE;
    int SCREEN = 128;

    // Create an array to hold up to MAX_PROJECTILES projectiles.
    Projectile projectiles[MAX_PROJECTILES];
    Projectile incoming_proj[MAX_PROJECTILES];

    Adafruit_Init();
    fillScreen(BLACK);
    drawShipWithAmmo( ((SCREEN / 2) - (SHIP_SIZE / 2)), ((SCREEN / 2) - (SHIP_SIZE / 2)), SHIP_SIZE, PLAYER_COLOR, 6);

    int shipPosition[2] = { ( (SCREEN / 2) - (SHIP_SIZE / 2) ), ( (SCREEN / 2) - (SHIP_SIZE / 2) ) };
    int8_t shipVelocity[2] = { 0, 0 };

    //
    // Register the interrupt handler
    //
    MAP_GPIOIntRegister(GPIOA1_BASE, GPIOA1IntHandler);

    //
    // Configure falling edge interrupt on switch
    //
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x20, GPIO_FALLING_EDGE);
    ulStatus = MAP_GPIOIntStatus(GPIOA1_BASE, false);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);

    // Enable switch interrupt
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x20);
    switch_intflag = 0;

    int i;
    for (i = 0; i < MAX_PROJECTILES; i++)
    {
        projectiles[i].state = false;
        projectiles[i].x_position = 0;
        projectiles[i].y_position = 0;
        projectiles[i].x_velocity = 0;
        projectiles[i].y_velocity = 0;
        projectiles[i].size = 0;

        incoming_proj[i].state = false;
        incoming_proj[i].x_position = 0;
        incoming_proj[i].y_position = 0;
        incoming_proj[i].x_velocity = 0;
        incoming_proj[i].y_velocity = 0;
        incoming_proj[i].size = 0;
    }


    while (FOREVER)
    {
        if (timer > 25) {
            if (ammo_cnt < 6 && (MAP_GPIOPinRead(GPIOA1_BASE, 0x20) & 0x20) == 0) {
                ammo_cnt++;
            }
            timer = 0;
        }

        if (MAP_UARTCharsAvail(UART1))
        {
            Projectile new_proj;
            UART1ReceiveProjectile(&new_proj);

            // Add the received projectile to an available slot
            int i;
            for (i = 0; i < MAX_PROJECTILES; i++)
            {
                if (!incoming_proj[i].state)  // Find an inactive slot
                {
                    incoming_proj[i] = new_proj;
                    break;
                }
            }
        }

        // -------------------------
        // Update Ship Position
        // -------------------------
        // Erase ship from the old position
        drawShipWithAmmo(shipPosition[0], shipPosition[1], SHIP_SIZE, BLACK, 6);

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
        drawShipWithAmmo(shipPosition[0], shipPosition[1], SHIP_SIZE, PLAYER_COLOR, ammo_cnt);
        Report("Ammo: %d\n\r", ammo_cnt);
        Report("Projectile Scale: %d\n\r", projectile_scale);


        // -------------------------
        // Handle New Projectile
        // -------------------------
        if (switch_intflag)
        {
            switch_intflag = 0;
            HWREG(NVIC_ST_CURRENT) = 1;  // Reset SysTick counter
            systick_cnt = 0;

            // Fire projectile only if ammo is available
            if (ammo_cnt > 0 || projectile_scale > 1) {
                int i;
                for (i = 0; i < MAX_PROJECTILES; i++)
                {
                    if (!projectiles[i].state)  // Find an inactive slot
                    {
                        projectiles[i].state = true;
                        projectiles[i].x_position = shipPosition[0] + (SHIP_SIZE / 2);
                        projectiles[i].y_position = shipPosition[1] + SHIP_SIZE + projectile_size;
                        projectiles[i].x_velocity = xAcc;
                        projectiles[i].y_velocity = 10 + (projectile_scale * 4);
                        projectiles[i].size = projectile_size;

                        // Reset scale to minimum after firing
                        projectile_scale = 1;
                        if (ammo_cnt > 0) {
                            ammo_cnt--;   // Decrease ammo immediately on shot
                        }
                        break;
                    }
                }
            }
        }


        // -------------------------
        // Update Projectiles
        // -------------------------
        int j;
        for (j = 0; j < MAX_PROJECTILES; j++)
        {
            if (projectiles[j].state)
            {
                // Erase the projectile at its old position
                fillCircle(projectiles[j].x_position,
                           projectiles[j].y_position,
                           projectiles[j].size,
                           BLACK);

                // Update projectile position based on its velocity
                projectiles[j].x_position += projectiles[j].x_velocity;
                projectiles[j].y_position += projectiles[j].y_velocity;

                // Check if projectile is off-screen
                if (projectiles[j].x_position < 0 || projectiles[j].x_position > (SCREEN - projectiles[j].size - 1) ||
                    projectiles[j].y_position > (SCREEN - projectiles[j].size - 1))
                {
                    projectiles[j].state = false;  // Deactivate projectile
                    if (projectiles[j].y_position > (SCREEN - projectiles[j].size - 1)) {
                        UART1SendProjectile(&projectiles[j]);
                    }
                }
                else
                {
                    // Draw projectile at its new position
                    fillCircle(projectiles[j].x_position,
                               projectiles[j].y_position,
                               projectiles[j].size,
                               PLAYER_COLOR);
                }
            }
        }

        // -------------------------
        // Update Incoming Projectiles
        // -------------------------
        for (j = 0; j < MAX_PROJECTILES; j++)
        {
            if (incoming_proj[j].state)
            {
                // Erase the projectile at its old position
                fillCircle(incoming_proj[j].x_position,
                           incoming_proj[j].y_position,
                           incoming_proj[j].size,
                           BLACK);

                // Update projectile position based on its velocity
                incoming_proj[j].x_position += incoming_proj[j].x_velocity;
                incoming_proj[j].y_position += incoming_proj[j].y_velocity;

                // Check if projectile is off-screen
                if (incoming_proj[j].x_position < 0 || incoming_proj[j].x_position > (SCREEN - incoming_proj[j].size - 1) ||
                        incoming_proj[j].y_position < (incoming_proj[j].size + 1))
                {
                    incoming_proj[j].state = false;  // Deactivate projectile
                }
                else
                {
                    // Draw projectile at its new position
                    fillCircle(incoming_proj[j].x_position,
                               incoming_proj[j].y_position,
                               incoming_proj[j].size,
                               (PLAYER_MODE) ? BLUE : RED);
                }
            }
        }
    }

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @
//
//*****************************************************************************
