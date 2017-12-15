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
#include <string.h>     // strncpy(), strncmp()
/* END RJ's Includes */

//Delwin's includes
#include "ssp0.h"       //sends codec song data
#include "ssp1.h"       //maybe to speak to lcd?
#include "ff.h"         //file access fatfs
#include <stdio.h>      //printf

#include "io.hpp"
#include "storage.hpp"
#include "LCD.hpp"

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
#define DBG_SONGCHANGE 0    // enables changeSong() debug messages
/* END RJ's Defines */

FIL file;
FIL file2;

TaskHandle_t xinitialize = NULL;
TaskHandle_t xplay_music = NULL;

/* BEGIN RJ's Globals */
TaskHandle_t xHandleKeypadRead = NULL;
TaskHandle_t xHandleControlUnit = NULL;
TaskHandle_t xHandleLcdMenu = NULL;
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
typedef enum{
    INITIAL,                 //initial menu display
    PLAYSONG,               //displays song list
    PLAYPLAYLIST,           //displays playlists to choose from
    CREATEPLAYLIST          //create a new playlist
} menu_state_t;

typedef struct{
    menu_state_t state;
    char filename[MAX_FILE_NAME_SIZE];
} menu_packet_t;

char pFilename[MAX_FILE_NAME_SIZE] = "\0";  // start with no song

/* BEGIN RJ's Mutexes */
SemaphoreHandle_t xKeypadValueMutex = NULL; // for protecting "last_key" and "repeatCnt"
SemaphoreHandle_t xStateMutex = NULL;       // for protecting "currentState" and "prevState"
SemaphoreHandle_t xVolumeMutex = NULL;      // for protecting "currentVolume"
SemaphoreHandle_t xSpiMutex = NULL;         // for protecting SSP0 bus use
SemaphoreHandle_t xFileMutex = NULL;        // for protecting "pFilename", "fileStatus", and "file"
/* END RJ's Mutexes */

//Queues
QueueHandle_t button_pushed = xQueueCreate(1, sizeof(uint8_t));     //'1' will be down '2' will be back '3' will be enter
QueueHandle_t menu_block = xQueueCreate(1, sizeof(menu_packet_t));

/* BEGIN RJ's Function Declarations */
bool volumeSet (uint8_t val);
void changeState (mp3_state new_state);
/* END RJ's Function Declarations */

void initialize(void *p)
{
    initLCD();
    setCursor(1,1);
    // cursorBlinkOn();
    clearLCD();
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
        
        //create songlist of song directory

        //Directory stuff

        //deletes the old directory song list
        f_unlink("1:songlist.txt");

        DIR dp;
        FRESULT status = FR_INT_ERR;
        FILINFO info;
        char Lfname[_MAX_LFN];
        char songname[_MAX_LFN];
        //end of directory stuff
        f_opendir(&dp, "1:/Songs");

        while(1)
        {
            info.lfname = Lfname;
            info.lfsize = sizeof(Lfname);

            status = f_readdir(&dp, &info);
            if(FR_OK != status || !info.fname[0]) 
            {
                f_closedir(&dp);
                break;
            }

            int length = sprintf(songname, "1:/Songs/%s\n", Lfname);
            Storage::append("1:songlist.txt", &songname, length);
        }
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
// @parameter   n/a
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

// @function    findNextSongInList
// @parameter   fname - the c-string name of the songlist/playlist file (i.e. "1:file.txt")
// @parameter   sname - the c-string to store the name of the next song
// @returns     n/a
// @details     This function automatically acquires the next song name from a file (list of songs) "fname" and stores it in "sname". It does this by searching for the next newline after the character, since RD mp3 player playlist/songlist files separate song names by newline
// @note        Both "fname" and "sname" are EXPECTED to be of size MAX_FILE_NAME_SIZE
void findNextSongInList (char* fname, char* sname) {
    static char currentSongList[MAX_FILE_NAME_SIZE] = "1:songlist.txt";  // initially track
    static uint32_t fileOffset = 0;

    // If fname is null, use the current song list
    if (fname == NULL) {
        // Do nothing
    }

    // Else, see if the passed file name is DIFFERENT from the current song list. If it is, close the old one and open the new one.
    else if (strncmp(fname, currentSongList, MAX_FILE_NAME_SIZE) != 0) {
        strncpy(currentSongList, fname, MAX_FILE_NAME_SIZE);
        currentSongList[MAX_FILE_NAME_SIZE - 1] = '\0'; // ensure last char is a null terminator
        fileOffset = 0;
    }

    // Read for next newline or null terminator
    int i;
    for (i = 0; i < MAX_FILE_NAME_SIZE; i++) {
        char temp = '\0';
        Storage::read(currentSongList, &temp, 1, fileOffset);

        switch ((int) temp) {
            case 10: {  // "\n"
                sname[i] = '\0';    // end new song name here...
                i = MAX_FILE_NAME_SIZE; // simply leave the for loop
                break;
            }
            case 0: {   // "\0"
                strncpy(currentSongList, "1:songlist.txt", MAX_FILE_NAME_SIZE); // If end of file, return to song list and start over
                fileOffset = 0;
                sname[i] = '\0';    // end new song name here...
                i = MAX_FILE_NAME_SIZE; // simply leave the for loop
                break;
            }
            default: {
                sname[i] = temp;
            }
        }

        fileOffset++;
    }
    sname[i] = '\0'; // ensure last char is a null terminator

    return;
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

    // Other Settings
    bool invokeSongChange = false;  // prompts the control unit to execute a song change
    bool lcdActionRequest = false;  // prompts the control unit to service an lcd action request instead of its own song decision protocol
    menu_packet_t lcd_state;        // holds the response packet from the lcd_menu task
    uint8_t button_lcd = 0;         // holds the command number to send to the lcd_menu task

    // Initial setup: acquire the full song list
    findNextSongInList("1:songlist.txt", pFilename);

    // Main control loop
    while (1) {
        vTaskDelay(50);
        #if DBG_CU
        u0_dbg_printf("sng: %s\n", pFilename);
        #endif

        // (Contextual) Continuous Playback
        if (xStateMutex != NULL) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {

                #if DBG_CU
                // u0_dbg_printf("CS: %d PS: %d\n", currentState, prevState);
                #endif

                // If prompted by the user via the lcd_menu task, handle a user request to change the song/play a playlist
                if (lcdActionRequest) {
                    lcdActionRequest = false;

                    #if DBG_CU
                    u0_dbg_printf("LCD_ST:%d LCD_FN:%s\n", lcd_state.state, lcd_state.filename);
                    #endif

                    switch (lcd_state.state) {
                        case INITIAL: {
                            // Do nothing
                            break;
                        }
                        case PLAYSONG: {
                            // User manually wants to change the song
                            strncpy(pFilename, lcd_state.filename, MAX_FILE_NAME_SIZE);
                            pFilename[MAX_FILE_NAME_SIZE - 1] = '\0';   // ensure last char is a null terminator
                            invokeSongChange = true;    // request a song change to the new song
                            break;
                        }
                        case PLAYPLAYLIST: {
                            // User wants to play a playlist
                            findNextSongInList(lcd_state.filename, pFilename);
                            invokeSongChange = true;
                            break;
                        }
                        case CREATEPLAYLIST: {
                            // Do nothing; lcd_menu task will handle this...
                            break;
                        }
                    }
                }

                // Else, determine mp3 player behavior based off of "cuSettings[1:0]" (i.e. selecting next song without user intervention)
                else {
                    switch (currentState) {
                        case IDLE: {    // If the song has finished/stopped, or if nothing has happened yet
                            if (prevState == IDLE) {    // here, the mp3 has just initialized
                                // Do nothing; the user has to select a song and hit play, first
                            } else if (prevState == PLAYING || prevState == PAUSED) {  // here, the mp3 has paused or has just finished a song
                                switch (cuSettings & 0x03) {
                                    // Playthrough Mode (default)
                                    case 0x00: {
                                        // Keep getting songs from the current song list
                                        findNextSongInList(NULL, pFilename);

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
                                        strncpy(pFilename, "1:Songs/Despasito.mp3", MAX_FILE_NAME_SIZE);
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
                        case '4': { //down button
                            button_lcd = 1;
                            if (xQueueSend(button_pushed, &button_lcd, portMAX_DELAY) == pdTRUE) {
                                u0_dbg_printf("Down button sent!\n");
                            } else {
                                u0_dbg_printf("nope\n");
                            }
                            // vTaskResume(xHandleLcdMenu);
                            break;
                        }

                        case '5': { //back button
                            button_lcd = 2;
                            xQueueSend(button_pushed, &button_lcd, portMAX_DELAY);
                            // vTaskResume(xHandleLcdMenu);
                            break;
                        }

                        case '6': { //enter button
                            button_lcd = 3;
                            // Send the lcd_menu task my request to process the menu option
                            xQueueSend(button_pushed, &button_lcd, portMAX_DELAY);

                            // Wait for the response packet form the lcd_menu task
                            xQueueReceive(menu_block, &lcd_state, portMAX_DELAY);
                            #if DBG_CU
                            u0_dbg_printf("st: %d fn:%s\n", lcd_state.state, lcd_state.filename);
                            #endif

                            // Signal that an lcd_menu task action has been requested
                            lcdActionRequest = true;
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


void display_lcd(char *row1, char *row2)
{
    clearLCD();
    setCursor(1,1);
    // putChar(row1[1]);
    putString(row1);
    setCursor(2,1);
    // putChar(row2[1]);
    putString(row2);
    // u0_dbg_printf("%s\n", row1);
    // u0_dbg_printf("%s\n", row2);
}

//if button pressed
//@file_dir '0' file '1' dir
int update_screen(char *row1, char *row2, int offset, unsigned int bytesRead)
{
    f_lseek(&file2, offset);
    printf("offset: %d\n", offset);
    char songname[32] = {0};
    char extract[32] = {0};
    f_read(&file2, &songname, 32, &bytesRead);
    sscanf(songname, "%s", extract);
    
    *row1 = {0};
    *row2 = {0};

    for(int i = 0; i < 32; i++)
    {
        if(extract[i] == '\0')
        {
            row1[i] = extract[i];
            offset += (i+1);
            break;
        }
        row1[i] = extract[i];
    }

    f_lseek(&file2, offset);
    printf("offset: %d\n", offset);
    *songname = {0};
    *extract = {0};
    f_read(&file2, &songname, 32, &bytesRead);
    sscanf(songname, "%s", extract);
    
    for(int i = 0; i < 32; i++)
    {
        if(extract[i] == '\0')
        {
            row2[i] = extract[i];
            break;
        }
        row2[i] = extract[i];
    }

    if(bytesRead < 2)
    {
        offset = 0;
    }
    return offset;
}    


void lcd_menu(void *p)
{
    //Directory stuff
    DIR dp;
    FRESULT status = FR_INT_ERR;
    FILINFO info;
    char Lfname[_MAX_LFN];
    //end of directory stuff

    menu_packet_t menu_state;
    menu_state.state = INITIAL;
    uint8_t button_pressed = 0;

    char lcd_top_row[32] = {0};
    char lcd_bottom_row[32] = {0};

    char *anotherFileName = "1:table_of_contents.txt";
    f_open(&file2, anotherFileName, FA_OPEN_EXISTING | FA_READ); 
    static int offset = 0;
    static unsigned int bytesRead = 0;

    //fill in top row
    f_lseek(&file2, offset);
    printf("offset: %d\n", offset);
    char songname[32] = {0};
    f_read(&file2, &songname, 32, &bytesRead);
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
    f_lseek(&file2, offset);
    printf("offset: %d\n", offset);
    *songname = {0};
    f_read(&file2, &songname, 32, &bytesRead);
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
        vTaskDelay(50);
        switch(menu_state.state)
        {
            //displaying the initial menu
            case INITIAL:
            if(xQueueReceive(button_pushed, &button_pressed, portMAX_DELAY))
            {
                if(button_pressed == 1)
                {
                    f_open(&file2, anotherFileName, FA_OPEN_EXISTING | FA_READ); 
                    offset = update_screen(lcd_top_row, lcd_bottom_row, offset, bytesRead);
                    display_lcd(lcd_top_row, lcd_bottom_row);
                    f_close(&file2);
                }
                else if(button_pressed == 3)
                {
                    xQueueSend(menu_block, &menu_state, portMAX_DELAY);

                    if (strcmp(lcd_top_row,"Play_Song") == 0)
                    {
                        menu_state.state = PLAYSONG;
                        f_opendir(&dp, "1:/Songs");
                        //clears screen and updates screen
                        clearLCD();

                        info.lfname = lcd_top_row;
                        info.lfsize = sizeof(lcd_top_row);

                        status = f_readdir(&dp, &info);
                        if(FR_OK != status || !info.fname[0]) 
                        {
                            f_closedir(&dp);
                            f_opendir(&dp, "1:/Songs");
                        }

                        info.lfname = lcd_bottom_row;
                        info.lfsize = sizeof(lcd_bottom_row);

                        status = f_readdir(&dp, &info);
                        if(FR_OK != status || !info.fname[0]) 
                        {
                            f_closedir(&dp);
                            f_opendir(&dp, "1:/Songs");
                        }  

                        display_lcd(lcd_top_row, lcd_bottom_row);
                    }
                    else if(strcmp(lcd_top_row,"Play_Playlist") == 0)
                    {
                        menu_state.state = PLAYPLAYLIST;
                        f_opendir(&dp, "1:/Playlists");

                        //clears screen and updates screen
                        clearLCD();

                        info.lfname = lcd_top_row;
                        info.lfsize = sizeof(lcd_top_row);

                        status = f_readdir(&dp, &info);
                        if(FR_OK != status || !info.fname[0]) 
                        {
                            f_closedir(&dp);
                            f_opendir(&dp, "1:/Playlists");
                        }

                        info.lfname = lcd_bottom_row;
                        info.lfsize = sizeof(lcd_bottom_row);

                        status = f_readdir(&dp, &info);
                        if(FR_OK != status || !info.fname[0]) 
                        {
                            f_closedir(&dp);
                            f_opendir(&dp, "1:/Playlists");
                        }  

                        display_lcd(lcd_top_row, lcd_bottom_row);
                    }
                    else if(strcmp(lcd_top_row,"Create_Playlist") == 0)
                    {
                        menu_state.state = CREATEPLAYLIST;
                        f_opendir(&dp, "1:/Songs");
                        //clears screen and updates screen
                        clearLCD();

                        info.lfname = lcd_top_row;
                        info.lfsize = sizeof(lcd_top_row);

                        status = f_readdir(&dp, &info);
                        if(FR_OK != status || !info.fname[0]) 
                        {
                            f_closedir(&dp);
                            f_opendir(&dp, "1:/Songs");
                        }

                        info.lfname = lcd_bottom_row;
                        info.lfsize = sizeof(lcd_bottom_row);

                        status = f_readdir(&dp, &info);
                        if(FR_OK != status || !info.fname[0]) 
                        {
                            f_closedir(&dp);
                            f_opendir(&dp, "1:/Songs");
                        }  

                        display_lcd(lcd_top_row, lcd_bottom_row);
                    }
                }
            }
            break;
            //display the song list
            case PLAYSONG:
            if(xQueueReceive(button_pushed, &button_pressed, portMAX_DELAY))     //if button down is pressed
            {
                //lock this inside a blocking queue. wait for button press
                if(button_pressed == 1)
                {
                    for(int i = 0; i < 32; i++)
                    {
                        lcd_top_row[i] = lcd_bottom_row[i];
                    }

                    info.lfname = lcd_bottom_row;
                    info.lfsize = sizeof(lcd_bottom_row);

                    status = f_readdir(&dp, &info);
                    printf("%d", status);
                    if(FR_OK != status || !info.fname[0]) 
                    {
                        f_closedir(&dp);
                        f_opendir(&dp, "1:/Songs");
                    }
                    // printf("%s", lcd_bottom_row);            
                    printf("\n-----------\n");    
                
                    display_lcd(lcd_top_row, lcd_bottom_row);   
                }
                else if(button_pressed == 2)
                {
                    menu_state.state = INITIAL;

                    offset = 0;
                    f_open(&file2, anotherFileName, FA_OPEN_EXISTING | FA_READ); 
                    offset = update_screen(lcd_top_row, lcd_bottom_row, offset, bytesRead);
                    display_lcd(lcd_top_row, lcd_bottom_row);
                    f_close(&file2);
                }
                else if(button_pressed == 3)
                {
                    //tells menu to play the song specified by lcd_top_row
                    sprintf(menu_state.filename, "1:/Songs/%s\n", lcd_top_row);
                    // menu_state.file_offset = offset;
                    xQueueSend(menu_block, &menu_state, portMAX_DELAY);
                }
            }
            break;

            //play a playlist
            case PLAYPLAYLIST:
            if(xQueueReceive(button_pushed, &button_pressed, portMAX_DELAY))         //choosing by button down
            {
                if(button_pressed == 1)
                {    
                    //lock this inside a blocking queue. wait for button press
                    for(int i = 0; i < 32; i++)
                    {
                        lcd_top_row[i] = lcd_bottom_row[i];
                    }

                    info.lfname = lcd_bottom_row;
                    info.lfsize = sizeof(lcd_bottom_row);

                    status = f_readdir(&dp, &info);
                    if(FR_OK != status || !info.fname[0]) 
                    {
                        f_closedir(&dp);
                        f_opendir(&dp, "1:/Playlists");
                    }      
                    printf("\n-----------\n");    
                
                    display_lcd(lcd_top_row, lcd_bottom_row);    
                }
                else if(button_pressed == 2)
                {
                    menu_state.state = INITIAL;
                    offset = 0;
                    f_open(&file2, anotherFileName, FA_OPEN_EXISTING | FA_READ); 
                    offset = update_screen(lcd_top_row, lcd_bottom_row, offset, bytesRead);
                    display_lcd(lcd_top_row, lcd_bottom_row);
                    f_close(&file2);
                }
                else if(button_pressed == 3)         //selecting playlist
                {
                    //play playlist
                    sprintf(menu_state.filename, "1:/Playlists/%s\n", lcd_top_row);
                    xQueueSend(menu_block, &menu_state, portMAX_DELAY);

                    // f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
                }
            }
            break;

            //creating a playlist
            case CREATEPLAYLIST:
            if(xQueueReceive(button_pushed, &button_pressed, portMAX_DELAY))             //enters song list
            {
                if(button_pressed == 1)
                {
                    //lock this inside a blocking queue. wait for button press
                    for(int i = 0; i < 32; i++)
                    {
                        lcd_top_row[i] = lcd_bottom_row[i];
                    }

                    info.lfname = lcd_bottom_row;
                    info.lfsize = sizeof(lcd_bottom_row);

                    status = f_readdir(&dp, &info);
                    if(FR_OK != status || !info.fname[0]) 
                    {
                        f_closedir(&dp);
                        f_opendir(&dp, "1:/Songs");
                    }
                    // printf("%s", lcd_bottom_row);            
                    printf("\n-----------\n");    
                
                    display_lcd(lcd_top_row, lcd_bottom_row);    
                }
                else if (button_pressed == 2)
                {
                    menu_state.state = INITIAL;
                    offset = 0;
                    f_open(&file2, anotherFileName, FA_OPEN_EXISTING | FA_READ); 
                    offset = update_screen(lcd_top_row, lcd_bottom_row, offset, bytesRead);
                    display_lcd(lcd_top_row, lcd_bottom_row);
                    f_close(&file2);
                }
                else if(button_pressed == 3)        //selects a song to add to playlist
                {
                    int length = sprintf(songname, "1:%s\n", lcd_top_row);
                    Storage::append("1:Playlists/playlist1.txt", &songname, length);
                    xQueueSend(menu_block, &menu_state, portMAX_DELAY);
                }
            }
            break;


            default:
                break;
        }

        // have this task suspend itself
        // vTaskSuspend(NULL);
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

    xTaskCreate(initialize, "initialize", STACK_BYTES(2048), NULL, PRIORITY_HIGH, &xinitialize);
    xTaskCreate(play_music, "play_music", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, &xplay_music);

    // xTaskCreate(menu_toc, "menu_toc", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, NULL);
    // xTaskCreate(dir_test, "dir_test", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, NULL);

    xTaskCreate(lcd_menu, "lcd_menu", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, &xHandleLcdMenu);

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
    xTaskCreate(keypadRead, "keypad", STACK_BYTES(1024), 0, PRIORITY_MEDIUM, &xHandleKeypadRead);
    xTaskCreate(controlUnit, "mp3_cu", STACK_BYTES(2048), 0, PRIORITY_HIGH, &xHandleControlUnit);
    /* END RJ's Code */

    scheduler_start(); ///< This shouldn't return
    return -1;
}
