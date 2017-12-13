/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "periodic_scheduler/periodic_callback.h"
#include "uart2.hpp"
#include "uart3.hpp"
#include "utilities.h"

#include "nrf_stream.hpp"

/* BEGIN RJ's Includes */
#include "FreeRTOS.h"   // FreeRTOS constructs
#include "LPC17xx.h"    // LPC_GPIOx, PINSELX
#include "stdint.h"     // uintX_t
#include "printf_lib.h" // u0_dbg_printf()
#include "adc0.h"       // ADC0 api
#include "keypad/keypad.h"
#include <string.h>     // strncpy()
/* END RJ's Includes */

//Delwin's includes
#include "ssp0.h"       //sends codec song data
#include "ssp1.h"       //maybe to speak to lcd?
#include "ff.h"         //file access fatfs
#include <stdio.h>      //printf

#include "io.hpp"

/* BEGIN RJ's Defines */
// VS1053 Audio Codec-Specific Defines
#define VS_READ 0x03
#define VS_WRITE 0X02

// Volume Control Defines
#define VOL_STEP_SIZE 10   // the amount of volume points to increase/decrease by (every 1 step = 0.5db)
#define VOL_LESS 0
#define VOL_MORE 1

// Control Unit Defines
#define CU_MIN_RPT_COUNT 3  // the minimum value of repeatCnt for the CU to accept last_key as a valid input character
#define MAX_FILE_NAME_SIZE 100  // limit size of song filenames
#define FILE_STATUS_OPEN 0x01   // file is open
#define FILE_STATUS_CLOSED 0x00 // file is closed/unopened/unitialized

// Misc Debug Controls
#define DBG_KEYPAD 0        // enables keypadRead() debug messages
#define DBG_CU 1            // enables controlUnit() debug messages
#define DBG_SONGCHANGE 1    // enables changeSong() debug messages
/* END RJ's Defines */

FIL file;

TaskHandle_t xinitialize = NULL;
TaskHandle_t xplay_music = NULL;

/* BEGIN RJ's Globals */
TaskHandle_t xHandleKeypadRead = NULL;
TaskHandle_t xHandleControlUnit = NULL;
char last_key = '\0';
uint8_t repeatCnt = 0;
uint8_t currentVolume = 0x2F;    // the volume val for both channels; half-volume = 0x7F = 127; max = 0x00 = 0; min = 0xFE = 254
uint8_t fileStatus = FILE_STATUS_CLOSED;  // 0x00 = closed/unloaded/uninitialized, 0x01 = streaming

typedef enum {
    PLAYING,    // playing a song
    PAUSED,     // a song is paused (somewhere in the middle)
    HALTED,     // the VS is busy, or we need to do some temporary manipulation to it for a short while
    IDLE        // doing nothing (i.e. a song was either completed or stopped & must be restarted from 0:00, or a song has yet to be loaded)
} mp3_state;
mp3_state currentState = IDLE;
mp3_state prevState = IDLE;
/* END RJ's Globals */

char pFilename[MAX_FILE_NAME_SIZE] = "1:Wet Dreamz.mp3";

/* BEGIN RJ's Mutexes */
SemaphoreHandle_t xKeypadValueMutex = NULL; // for protecting "last_key" and "repeatCnt"
SemaphoreHandle_t xStateMutex = NULL;       // for protecting "currentState" and "prevState"
SemaphoreHandle_t xVolumeMutex = NULL;      // for protecting "currentVolume"
SemaphoreHandle_t xSpiMutex = NULL;         // for protecting SSP0 bus use
SemaphoreHandle_t xFileMutex = NULL;        // for protecting "pFilename", "fileStatus", and "file"
/* END RJ's Mutexes */

/* BEGIN RJ's Function Declarations */
bool volumeSet (uint8_t val);
void changeState (mp3_state new_state);
/* END RJ's Function Declarations */

void initialize(void *p)
{
    // Create mutexes for specific parameters
    xKeypadValueMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xVolumeMutex = xSemaphoreCreateMutex();
    xSpiMutex = xSemaphoreCreateMutex();
    xFileMutex = xSemaphoreCreateMutex();

    //XDCS
    LPC_GPIO1->FIODIR |= (1 << 19);

    //XCS
    LPC_GPIO1->FIODIR |= (1 << 20);
    LPC_GPIO1->FIOSET = (1 << 20);

    //RST
    LPC_GPIO1->FIODIR |= (1 << 22);

    //DREQ
    LPC_GPIO1->FIODIR &= ~(1 << 23);

    LPC_GPIO1->FIOCLR = (1 << 22);
    LPC_GPIO1->FIOSET = (1 << 22);

    ssp0_init(0);

    static bool stop_run = false;

    unsigned int bytesRead = 0;

    if(!stop_run)
    {
        if(LPC_GPIO1->FIOPIN & (1 << 20))
        {
            // printf("XCS is high.\n");
        }
        else 
        {
            // printf("XCS is low.\n");
        }

        if(LPC_GPIO1->FIOPIN & (1 << 23))
        {
            LPC_GPIO1->FIOCLR = (1 << 20);

            // printf("Writing to CLOCKF register");

            ssp0_exchange_byte(0x02);
            ssp0_exchange_byte(0x03);
            // minimum music accesibility is with SC_MULT = XTALI x 2.0 (bits 15-13 are 3'b001)
            // max flexibility acheived with SC_ADD = 3 (bits 12-11 are 2'b11)

            // running at XTALI = 3MHz? 21.192MHz (bits 10-0 are 11'b100 1110 0010) <-- a bit too fast
            // ssp0_exchange_byte(0x3C);
            // ssp0_exchange_byte(0xE2);

            // running at XTALI = ?MHz <-- a tiny bit too fast
            // ssp0_exchange_byte(0x3D);
            // ssp0_exchange_byte(0x1F);

            // running at XTALI = ?MHz <-- the closest I've got to the original
            ssp0_exchange_byte(0x3D);
            ssp0_exchange_byte(0x25);

            // running at XTALI = ?MHz <-- a tiny bit too slow
            // ssp0_exchange_byte(0x3D);
            // ssp0_exchange_byte(0x27);

            // running at XTALI = ?MHz <-- a bit too slow
            // ssp0_exchange_byte(0x3D);
            // ssp0_exchange_byte(0x2F);

            // printf("%X%X\n", b2, b3);
            LPC_GPIO1->FIOSET = (1 << 20);
        }


        if(LPC_GPIO1->FIOPIN & (1 << 23))
        {
            LPC_GPIO1->FIOCLR = (1 << 20);
            // printf("Writing to MODE register\n");
            
            ssp0_exchange_byte(0x02);
            ssp0_exchange_byte(0x00);
            ssp0_exchange_byte(0x48);
            ssp0_exchange_byte(0x40);

            LPC_GPIO1->FIOSET = (1 << 20);
        }

        // RJ: Set an initial volume
        while (!volumeSet(currentVolume)) {
            // try again
        }

        stop_run = true;

        //SD read
        

        f_open(&file, pFilename, FA_OPEN_EXISTING | FA_READ);
        fileStatus = FILE_STATUS_OPEN;
    }

    vTaskSuspend(NULL);

}

void play_music(void *p)
{
    while(1)
    {
        if (xStateMutex != NULL) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                if (currentState != PLAYING) {
                    xSemaphoreGive(xStateMutex);
                    vTaskSuspend(NULL); // have this task suspend itself (in an effort to prevent a case where the CU pauses this task while p1.19 is still LOW)
                }

                if(LPC_GPIO1->FIOPIN & (1 << 23))
                {
                    static uint32_t offset = 0;
                    char buffer[32] = {0};
                    unsigned int bytesRead = 0;
            
                    if (xFileMutex != NULL) {
                        if (xSemaphoreTake(xFileMutex, portMAX_DELAY) == pdTRUE) {
                            f_read(&file, &buffer, 32, &bytesRead);
                            if(bytesRead < 32)
                            {
                                f_close(&file);
                                fileStatus = FILE_STATUS_CLOSED;
                            }
                            xSemaphoreGive(xFileMutex);
                        }
                    }
                    
                    offset = offset + 32;

                    LPC_GPIO1->FIOCLR = (1 << 19);

                    if (xSpiMutex != NULL) {
                        if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
                            for(int i = 0; i < 32; i++)
                            {
                                ssp0_exchange_byte(buffer[i]);
                            }
                            xSemaphoreGive(xSpiMutex);
                        }
                    }
                    LPC_GPIO1->FIOSET = (1 << 19);

                    xSemaphoreGive(xStateMutex);    // this is necessary to avoid deadlock (changeState REQUIRES xStateMutex)

                    // After sending last group of bytes (i.e. end of song), mark that this song is over
                    if (bytesRead < 32) {
                        offset = 0;
                        changeState(IDLE);
                    }
                } else {
                    xSemaphoreGive(xStateMutex);
                }
            }
        }
    }
}

void menu_toc(void *p)
{

    char *pFilename = "1:table_of_contents.txt";
    f_open(&file, pFilename, FA_OPEN_EXISTING | FA_READ); 
    unsigned int bytesRead = 0;


    int offset = 0;

    for(int j = 0; j < 5; j++)
    {
        f_lseek(&file, offset);
        printf("offset: %d\n", offset);
        char songname[32] = {0};
        char extract[32] = {0};
        f_read(&file, &songname, 32, &bytesRead);
        sscanf(songname, "%s", extract);

        //extracting the song name
        for(int i = 0; i < 32; i++)
        {
            if(extract[i] == '\0')
            {
                printf("\n");
                printf("line length: %d\n", i);
                offset += (i+1);
                break;
            }
            printf("%c", extract[i]);
        }


    }

    while(1)
    {

    }
}

void dir_test(void *p)
{
    DIR dp;
    FRESULT status = FR_INT_ERR;
    FILINFO info;
    char Lfname[_MAX_LFN];

    if (FR_OK == (status = f_opendir(&dp, "1:")))
    {
        // printf("%d opened successfully.\n", status);
    }
    else
    {
        // printf("%d failed.\n", status);
    }

    while(1)
    {
        //lock this inside a blocking queue. wait for button press
        info.lfname = Lfname;
        info.lfsize = sizeof(Lfname);

        status = f_readdir(&dp, &info);
        if(FR_OK != status || !info.fname[0]) 
        {
            f_closedir(&dp);
            f_opendir(&dp, "1:");
        }
        printf("%s", Lfname);            
        printf("\n-----------\n");            
    }
}

/* BEGIN RJ's Code */
// @function    changeState
// @parameter   new_state - the state to change to
// @returns     n/a
// @details     This function updates the current state with the value of new_state, while also updating prevState with the last known state from currentState
void changeState (mp3_state new_state) {
    // Use mutex to acquire access to guarded resources
    if (xStateMutex != NULL) {
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            prevState = currentState;
            currentState = new_state;
            xSemaphoreGive(xStateMutex);
        }
    }
    return;
}

// @function    keypadRead
// @parameter   n/a
// @returns     n/a
// @details     This function comprises the keypad reader task, whose purpose is to determine the pressed key using the ADC value taken from the keypad circuit
void keypadRead (void* p) {
    // Init P1.30 as ADC0.4
    LPC_PINCON->PINSEL3 |= (3 << 28);   // sets PINSEL3[29:28] bits to 11
    uint8_t cnt = 0;
    uint16_t avg = 0;
    uint16_t running_total = 0;
    keypad kp;

    // Main task loop
    while (1) {
        vTaskDelay(5);
        switch (cnt) {
            case 5: {  // every 5 measurements, print the avg and use that to determine which key is pressed
                avg = running_total/cnt;
                char temp = kp.key(avg);

                // Use mutex to acquire access to guarded resources (i.e. last_key and repeatCnt)
                if (xKeypadValueMutex != NULL) {
                    if (xSemaphoreTake(xKeypadValueMutex, portMAX_DELAY) == pdTRUE) {
                        // Key Noise Resolution: If current read matches the previous key, increment the key counter (a larger key count means this character is most likely the intended button being pressed). A sufficiently high repeatCnt will invoke the Control Unit to process the current key
                        switch (temp != '\0' && temp == last_key) {
                            case 1: {
                                repeatCnt++;
                                break;
                            }
                            default: {  // either too much button noise, or we are getting the '\0' character
                                repeatCnt = 0;
                                break;
                            }
                        }
                        last_key = temp;
                        #if DBG_KEYPAD
                        u0_dbg_printf("key:%d %c\n", avg, (last_key == '\0') ? 'x' : last_key);
                        #endif
                        avg = 0;
                        cnt = 0;
                        running_total = 0;
                        xSemaphoreGive(xKeypadValueMutex);
                    }
                }
                break;
            }
            default: {
                running_total += adc0_get_reading(4);  // reads from adc0 channel 3 (i.e. P0.26)
                cnt++;
            }
        }
    }
}

// @function    volumeControl
// @parameter   dir - a uin8_t acting as a bool to control whether to increase (i.e. 1), or decrease (i.e. 0) volume by a preset amount
// @returns     False, if the operation couldn't be completed. True, otherwise
// @details     This function comprises the task that increases or decreases the volume (depending on the value dir) by steps the size of VOL_STEP_SIZE
bool volumeControl (uint8_t dir) {
    bool success = false;

    // Take volume mutex
    if (xVolumeMutex != NULL) {
        if (xSemaphoreTake(xVolumeMutex, portMAX_DELAY) == pdTRUE) {
            // Only assert control commands to SCI if DREQ is high
            if (LPC_GPIO1->FIOPIN & (1 << 23)) {
                LPC_GPIO1->FIOCLR = (1 << 20);  // drive XCS LOW

                if (xSpiMutex != NULL) {
                    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
                        ssp0_exchange_byte(VS_WRITE);
                        ssp0_exchange_byte(0x0B);   // VOL register
                        switch (dir) {
                            // Increase volume
                            case 1: {
                                currentVolume = (currentVolume - VOL_STEP_SIZE < 0) ? 0 : (currentVolume - VOL_STEP_SIZE);
                                break;
                            }

                            // Decrease volume
                            default: {
                                currentVolume = (currentVolume + VOL_STEP_SIZE > 0xFE) ? 0xFE : (currentVolume + VOL_STEP_SIZE);
                                break;
                            }
                        }
                        ssp0_exchange_byte(currentVolume);  // control volume of first speaker (L or R? Which is modified first?)
                        ssp0_exchange_byte(currentVolume);  // control volume of other speaker
                        xSemaphoreGive(xSpiMutex);
                    }
                }
                success = true;
                
                LPC_GPIO1->FIOSET = (1 << 20);  // pull XCS HIGH
            }
            xSemaphoreGive(xVolumeMutex);
        }
    }

    return success;
}

// @function    volumeSet
// @parameter   val - the uint8_t value to set the volume register to
// @returns     False, if the operation couldn't be completed. True, otherwise
// @details     This function allows the volume to be set manually to a specific value in contrast to volumeControl(), which only allows relative volume changes (i.e. either louder or quieter)
// @note        EXERCISE CAUTION, since this function DOES NOT CHECK if val is within bounds (i.e. 0 <= val <= 254)
//              Also, this function updates currentVolume with the new value
bool volumeSet (uint8_t val) {
    bool success = false;

    // Take volume mutex
    if (xVolumeMutex != NULL) {
        if (xSemaphoreTake(xVolumeMutex, portMAX_DELAY) == pdTRUE) {
            // Only assert control commands to SCI if DREQ is high
            if (LPC_GPIO1->FIOPIN & (1 << 23)) {
                LPC_GPIO1->FIOCLR = (1 << 20);  // drive XCS LOW

                if (xSpiMutex != NULL) {
                    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
                        ssp0_exchange_byte(VS_WRITE);
                        ssp0_exchange_byte(0x0B);   // VOL register
                        ssp0_exchange_byte(val);    // control volume of first speaker (L or R? Which is modified first?)
                        ssp0_exchange_byte(val);    // control volume of other speaker
                        xSemaphoreGive(xSpiMutex);
                    }
                }
                currentVolume = val;        // update current volume level
                success = true;
                
                LPC_GPIO1->FIOSET = (1 << 20);  // pull XCS HIGH
            }
            xSemaphoreGive(xVolumeMutex);
        }
    }

    return success;
}

// @function    changeSong
// @parameter   newSong - a c-string of the file name to search for (i.e. "1:mySong.mp3")
// @returns     n/a
// @details     This function allows the user to change the current song to the one listed in pFilename
// @note        This function assumes you've appropriately updated "pFilename" before calling it
void changeSong () {
    if (xFileMutex != NULL) {
        if (xSemaphoreTake(xFileMutex, portMAX_DELAY) == pdTRUE) {
            // If file is still open, close it to avoid corruption
            if (fileStatus != FILE_STATUS_CLOSED) {
                #if DBG_SONGCHANGE
                u0_dbg_printf("CHGSNG: closing\n");
                #endif
                f_close(&file);
                fileStatus = FILE_STATUS_CLOSED;
            }

            #if DBG_SONGCHANGE
            u0_dbg_printf("CHGSNG: opening new file\n");
            #endif
            f_open(&file, pFilename, FA_OPEN_EXISTING | FA_READ);
            fileStatus = FILE_STATUS_OPEN;
            xSemaphoreGive(xFileMutex);
        }
    }
}

// @function    controlUnit
// @parameter   n/a
// @returns     n/a
// @details     This function manages state changes to control the operation of the mp3 player
void controlUnit (void* p) {
    // CU Settings Flags Bit Description:
    // The 2 least significant bits determine the mode to run in,
    //  00 - playthrough mode (the DEFAULT mode; play all songs alphabetically, then stop when you get to the end of the list)
    //  01 - shuffle mode (play all songs at random, without end)
    //  10 - repeat mode (repeat the current song over and over)
    static uint8_t cuSettings = 0x00;
    bool invokeSongChange = false;

    // Main control loop
    while (1) {
        vTaskDelay(50);

        // (Contextual) Continuous Playback
        if (xStateMutex != NULL) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                switch (currentState) {
                    case IDLE: {    // If the song has finished/stopped, or if nothing has happened yet
                        if (prevState == IDLE) {    // here, the mp3 has just initialized
                            // Do nothing; the user has to select a song and hit play, first
                        } else if (prevState == PLAYING || prevState == PAUSED) {  // here, the mp3 has paused or has just finished a song
                            // Determine what to do based off of "cuSettings[1:0]"
                            switch (cuSettings & 0x03) {
                                // Playthrough Mode (default)
                                case 0x00: {
                                    // Add some one-time, alphabetical-order song-playing logic here...
                                    // ...
                                    // ...

                                    // Then change the song name like this...
                                    strncpy(pFilename, "1:Cant_Feel_My_Face.mp3", MAX_FILE_NAME_SIZE);
                                    pFilename[MAX_FILE_NAME_SIZE - 1] = '\0';   // ensure last char is a null terminator

                                    // Request a song change to the new song
                                    invokeSongChange = true;
                                    break;
                                }

                                // Shuffle Mode
                                case 0x01: {
                                    // Add some random-selection, never-ending song-playing logic here...
                                    // ...
                                    // ...

                                    // Then change the song name like this...
                                    strncpy(pFilename, "1:Wet Dreamz.mp3", MAX_FILE_NAME_SIZE);
                                    pFilename[MAX_FILE_NAME_SIZE - 1] = '\0';   // ensure last char is a null terminator

                                    // Request a song change to the new song
                                    invokeSongChange = true;
                                    break;
                                }

                                // Repeat Mode
                                case 0x02: {
                                    // Simply request a song change without changing the song name
                                    invokeSongChange = true;
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
                xSemaphoreGive(xStateMutex);

                // After song change, only autoplay the song if the prev song wasn't paused or stopped
                if (invokeSongChange == true) {
                    invokeSongChange = false;
                    changeSong();   // switch stream source

                    // Set Codec's Cancel Bit
                    if (xSpiMutex != NULL) {
                        if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
                            while (LPC_GPIO1->FIOPIN & (1 << 23) == 0) {
                                // Wait for DREQ to go high before sending SCI commands
                                #if DBG_CU
                                u0_dbg_printf("CU: DREQ low\n");
                                #endif
                            }

                            // Drive XCS LOW
                            LPC_GPIO1->FIOCLR = (1 << 20);

                            // Set cancel bit here
                            ssp0_exchange_byte(VS_WRITE);
                            ssp0_exchange_byte(0x00);   // mode register addr
                            ssp0_exchange_byte(0x48);   // keep high byte same as before
                            ssp0_exchange_byte(0x48);   // preserve previous settings, except bit 3, which we set

                            // Pull XCS HIGH
                            LPC_GPIO1->FIOSET = (1 << 20);

                            xSemaphoreGive(xSpiMutex);

                            while (LPC_GPIO1->FIOPIN & (1 << 23) == 0) {
                                // Wait for DREQ to go high before resuming normal data streaming (see 7.4.2 [pg.21] of datasheet [SCI Write])
                                #if DBG_CU
                                u0_dbg_printf("CU: DREQ low\n");
                                #endif
                            }
                        }
                    }

                    // Since music task suspends itself when currentState != PLAYING,
                    // we must restore the PLAYING state and resume it ourselves
                    changeState(PLAYING);
                    vTaskResume(xplay_music);
                }
            }
        }

        // Keypad Input Response
        if (xKeypadValueMutex != NULL) {
            if (xSemaphoreTake(xKeypadValueMutex, portMAX_DELAY) == pdTRUE) {
                // Only do something if key value wasn't noisy
                if (repeatCnt >= CU_MIN_RPT_COUNT) {
                    // Determine action based on accepted key value
                    switch (last_key) {
                        case '1': { // Play
                            // Do something
                            #if DBG_CU
                            u0_dbg_printf("CU: Playing\n");
                            #endif
                            changeState(PLAYING);
                            vTaskResume(xplay_music);
                            break;
                        }
                        case '2': { // Pause
                            // Do something
                            #if DBG_CU
                            u0_dbg_printf("CU: Pausing\n");
                            #endif
                            changeState(PAUSED);
                            break;
                        }
                        case '3': { // Decrease Volume
                            #if DBG_CU
                            u0_dbg_printf("CU: V-\n");
                            #endif
                            changeState(HALTED);
                            while (!volumeControl(VOL_LESS)) {
                                // Redo volume control
                                #if DBG_CU
                                u0_dbg_printf("...\n");
                                #endif
                            }
                            #if DBG_CU
                            u0_dbg_printf("vol:-%5fdB\n", ((float) currentVolume) * 0.5);
                            #endif
                            changeState(PLAYING);
                            break;
                        }
                        case 'A': { // Increase Volume
                            #if DBG_CU
                            u0_dbg_printf("CU: V+\n");
                            #endif
                            changeState(HALTED);
                            while (!volumeControl(VOL_MORE)) {
                                // Redo volume control
                                #if DBG_CU
                                u0_dbg_printf("...\n");
                                #endif
                            }
                            #if DBG_CU
                            u0_dbg_printf("vol:-%5fdB\n", ((float) currentVolume) * 0.5);
                            #endif
                            changeState(PLAYING);
                            break;
                        }
                        case 'B': { // Next Song
                            #if DBG_CU
                            u0_dbg_printf("CU: NXT\n");
                            #endif
                            changeState(IDLE);
                            break;
                        }
                        case 'D': { // Switch mode
                            // Change modes in a cyclic manner
                            switch ((cuSettings & 0x03) % 3) {
                                // From Playthrough Mode, switch to Shuffle Mode
                                case 0x00: {
                                    #if DBG_CU
                                    u0_dbg_printf("CU: shuffle mode\n");
                                    #endif
                                    cuSettings &= 0xFC; // preserve bits 7-2
                                    cuSettings |= 0x01; // set bit 0
                                    break;
                                }

                                // From Shuffle Mode, switch to Repeat Mode
                                case 0x01: {
                                    #if DBG_CU
                                    u0_dbg_printf("CU: repeat mode\n");
                                    #endif
                                    cuSettings &= 0xFC; // preserve bits 7-2
                                    cuSettings |= 0x02; // set bit 1
                                    break;
                                }

                                // From Repeat Mode, switch to Playthrough Mode
                                case 0x02: {
                                    #if DBG_CU
                                    u0_dbg_printf("CU: playthrough mode\n");
                                    #endif
                                    cuSettings &= 0xFC; // preserve bits 7-2, clear bits 1-0
                                    break;
                                }
                            }
                        }
                        default: {
                            // If last_key == '\0' or is unknown, do nothing
                            break;
                        }
                    }

                    // Reset key repeatCnt
                    repeatCnt = 0;
                }
                xSemaphoreGive(xKeypadValueMutex);
            }
        }
    }
}
/* END RJ's Code */
int offset = 0;
unsigned int bytesRead = 0;

void display_lcd(char *row1, char *row2)
{
    printf("%s\n", row1);
    printf("%s\n", row2);
}

//if button pressed
void update_screen(char *row1, char *row2, int button)
{
    f_lseek(&file, offset);
    printf("offset: %d\n", offset);
    char songname[32] = {0};
    char extract[32] = {0};
    f_read(&file, &songname, 32, &bytesRead);
    sscanf(songname, "%s", extract);

    *row1 = {0};

    //sets row1 to row2
    for(int i = 0; i < 32; i++)
    {
        row1[i] = row2[i];
    }

    //sets row2 to the next line
    for(int i = 0; i < 32; i++)
    {
        if(extract[i] == '\0')
        {
            row2[i] = extract[i];
            offset += (i+1);
            break;
        }
        row2[i] = extract[i];
    }

    if(bytesRead < 2)
    {
        offset = 0;
    }

}    

void lcd_menu(void *p)
{
    char lcd_top_row[32] = {0};
    char lcd_bottom_row[32] = {0};

    char *pFilename = "1:table_of_contents.txt";
    f_open(&file, pFilename, FA_OPEN_EXISTING | FA_READ); 

    //fill in top row
    f_lseek(&file, offset);
    printf("offset: %d\n", offset);
    char songname[32] = {0};
    f_read(&file, &songname, 32, &bytesRead);
    sscanf(songname, "%s", lcd_top_row);

    //extracting the song name
    for(int i = 0; i < 32; i++)
    {
        if(lcd_top_row[i] == '\0')
        {
            // printf("\n");
            // printf("line length: %d\n", i);
            offset += (i+1);
            break;
        }
        // printf("%c", lcd_top_row[i]);
    }

    //fills in bottom row
    f_lseek(&file, offset);
    printf("offset: %d\n", offset);
    *songname = {0};
    f_read(&file, &songname, 32, &bytesRead);
    sscanf(songname, "%s", lcd_bottom_row);

    //extracting the song name
    for(int i = 0; i < 32; i++)
    {
        if(lcd_bottom_row[i] == '\0')
        {
            // printf("\n");
            // printf("line length: %d\n", i);
            offset += (i+1);
            break;
        }
        // printf("%c", lcd_bottom_row[i]);
    }

    display_lcd(lcd_top_row, lcd_bottom_row);

    while(1)
    {
        if(SW.getSwitch(1))
        {
            f_open(&file, pFilename, FA_OPEN_EXISTING | FA_READ); 
            vTaskDelay(1000);
            update_screen(lcd_top_row, lcd_bottom_row, 1);
            display_lcd(lcd_top_row, lcd_bottom_row);
            f_close(&file);
        }
        else if(SW.getSwitch(2))
        {

        }
    }
}




/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    // xTaskCreate(initialize, "initialize", STACK_BYTES(2048), NULL, PRIORITY_HIGH, &xinitialize);
    // xTaskCreate(play_music, "play_music", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, &xplay_music);

    // xTaskCreate(menu_toc, "menu_toc", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, NULL);
    // xTaskCreate(dir_test, "dir_test", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, NULL);

    xTaskCreate(lcd_menu, "lcd_menu", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, NULL);

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    const bool run_1Khz = false;
    scheduler_add_task(new periodicSchedulerTask(run_1Khz));
    #endif

    /* The task for the IR receiver to "learn" IR codes */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif


    /* BEGIN RJ's Code */
    // xTaskCreate(keypadRead, "keypad", STACK_BYTES(1024), 0, PRIORITY_MEDIUM, &xHandleKeypadRead);
    // xTaskCreate(controlUnit, "mp3_cu", STACK_BYTES(1024), 0, PRIORITY_HIGH, &xHandleControlUnit);
    /* END RJ's Code */

    scheduler_start(); ///< This shouldn't return
    return -1;
}
