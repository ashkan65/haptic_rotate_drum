#include <iostream>
#include "simple826.hpp"
#define TSETTLE    7        // Settling delay after switching AIN (adjust as necessary).
#define SLOTFLAGS  0xFFFF   // Timeslot flags: use all 16 timeslots. -10V to +10V 0x0000 to 0xFFFF
/*
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// Settling times in microseconds
#include <sstream>


#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PI  3.1415926535

float Offset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Offset2[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float conv = 101.97;
float Sub[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Calib_Force[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float OffsettedVoltage[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Calib[6][6] = {
  {0.41388  , 0.14592  , 4.19733  , -47.43412,  -2.46789,  46.50125},
  { -4.25833 , 54.21553 , 3.63101  , -27.03922 , -0.56011 , -27.53264},
  { 66.98523, 3.36221  , 67.82584 ,  2.86437 , 66.75031 ,  4.08293 },
  { -0.03263 , 0.38512  , -1.06996 ,  -0.24152,   1.05840,  -0.12555 },
  { 1.24766 , 0.07060  , -0.65865 ,  0.30393 , -0.62320 , -0.36835 },
  { 0.06922 , -0.69457 ,  0.03705 , -0.69582 ,  0.04867 ,  -0.69297},
};

*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The const for the board. 
Simple826::Simple826(){
    board      = 0;                        // change this if you want to use other than board number 0
    errcode     = S826_ERR_OK;  
    boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards
    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0) {
        int i;
        printf("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
        for (i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            }
        }
    }
    PrintError();
    S826_DacRangeWrite(0, 0, S826_DAC_SPAN_10_10, 1);
 
    ////////////Analog input initialization:

    int i;
    int adcbuf[16];  // Sample buffer -- always set size=16 (even if fewer samples needed)

    // Configure all timeslots: 1 slot per AIN; constant settling time for all slots.
    //  measuring ain i on slot i (The first i is the slot number, the second i is the AIN channel number which is an attribute of slot)
    for (i = 0; i < 16; i++)
      S826_AdcSlotConfigWrite(board, i, i, TSETTLE, S826_ADC_GAIN_1); //Analog channel number-PGA gain-Settling time

    // Configure adc system and start it running.
    S826_AdcSlotlistWrite(board, SLOTFLAGS, S826_BITWRITE); // Enable all 16 timeslots.
    S826_AdcTrigModeWrite(board, 0);                        // Select free-running mode.
    S826_AdcEnableWrite(board, 1); 
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The Dist for the board. 

Simple826::~Simple826(){
    S826_SystemClose();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AnalogWrite. 

void Simple826::SetDacOutput(uint *chan, double *volts){ // DAC for one channel ----> DOTO add a function for vector update!
    errcode = S826_DacDataWrite(board, *chan, (int)(*volts * 0xFFFF / 20) + 0x8000, 0);  // program DAC output and return error code
};




void Simple826::GetDacOutput(uint *chan, double *volts){   //Reads the current voltage for a given channel.
    uint range, setpoint;
    errcode = S826_DacRead(board, *chan, &range, &setpoint, 0);
    *volts = (setpoint - 32767.0) * (20.0 / 65535.0);
    PrintError(); //Incase of error!!!
}; 



void Simple826::ReadAdcOutput(int* adcbuf, double *data){ // Abalog input read for every channel from 1 to 16
    errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 1000) ; // read adc data from 16 slots
        // Converting buffer value to voltage in each slot (data*10volt/2^8) setting: -10V to 10V, -2^8 to 2^8 bits : 
        for (int slot = 0; slot < 16; slot++){ 
            data[slot] = (short)( adcbuf[slot] & 0xFFFF );
            data[slot] = (double)(data[slot]*10)/(32768);
        };     


  }




int Simple826::GetError(){ //return error code 
    return errcode; 
};

void Simple826::PrintError(){ //return error code 
    switch (errcode)
    {
        case S826_ERR_OK:           break;
        case S826_ERR_BOARD:        printf("Illegal board number"); break;
        case S826_ERR_VALUE:        printf("Illegal argument"); break;
        case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
        case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
        case S826_ERR_DRIVER:       printf("Driver call failed"); break;
        case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
        case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
        case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
        case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
        case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
        default:                    printf("Unknown error"); break;
    }
}
