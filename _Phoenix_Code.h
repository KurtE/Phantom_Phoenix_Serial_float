//#define DEBUG_IK
//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino 
//
// Phoenix_Code.h
//
//     This contains the main code for the Phoenix project.  It is included in
//     all of the different configurations of the phoenix code.
//
//NEW IN V2.X
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#if ARDUINO>99
#include <Arduino.h>
#else
#endif
//#include <EEPROM.h>
//#include <PS2X_lib.h>
#include <pins_arduino.h>
//#include <SoftwareSerial.h>        
#define BalanceDivFactor CNT_LEGS    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)
//#include <Wire.h>
//#include <I2CEEProm.h>

// Only compile in Debug code if we have something to output to
#ifdef DBGSerial
#define DEBUG
//#define DEBUG_X
#endif


//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1. 
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277

static const byte GetACos[] PROGMEM = {    
  255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225, 
  224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193, 
  192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158, 
  157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117, 
  115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70, 
  70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
  59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
  46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
  28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
  16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0 };//

//Sin table 90 deg, persision 0.5 deg [180 values]
static const word GetSin[] PROGMEM = {
  0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 
  1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007, 
  3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 
  4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 
  5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819, 
  6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 
  7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 
  8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 
  9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 
  9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969, 
  9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };//


//Build tables for Leg configuration like I/O and MIN/ Max values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

// BUGBUG: Need a cleaner way to define...
// Lets allow for which legs servos to be inverted to be defined by the robot
// This is used by the Lynxmotion Symetrical Quad.
#ifndef cRRCoxaInv
#define cRRCoxaInv 1 
#endif
#ifndef cRMCoxaInv 
#define cRMCoxaInv 1 
#endif
#ifndef cRFCoxaInv 
#define cRFCoxaInv 1 
#endif

#ifndef cLRCoxaInv 
#define cLRCoxaInv 0 
#endif
#ifndef cLMCoxaInv 
#define cLMCoxaInv 0 
#endif
#ifndef cLFCoxaInv 
#define cLFCoxaInv 0 
#endif

#ifndef cRRFemurInv 
#define cRRFemurInv 1 
#endif
#ifndef cRMFemurInv 
#define cRMFemurInv 1 
#endif
#ifndef cRFFemurInv 
#define cRFFemurInv 1 
#endif

#ifndef cLRFemurInv 
#define cLRFemurInv 0 
#endif
#ifndef cLMFemurInv 
#define cLMFemurInv 0 
#endif
#ifndef cLFFemurInv 
#define cLFFemurInv 0 
#endif

#ifndef cRRTibiaInv 
#define cRRTibiaInv 1 
#endif
#ifndef cRMTibiaInv 
#define cRMTibiaInv 1 
#endif
#ifndef cRFTibiaInv 
#define cRFTibiaInv 1 
#endif

#ifndef cLRTibiaInv 
#define cLRTibiaInv 0 
#endif
#ifndef cLMTibiaInv 
#define cLMTibiaInv 0 
#endif
#ifndef cLFTibiaInv 
#define cLFTibiaInv 0 
#endif

#ifndef cRRTarsInv
#define cRRTarsInv 1 
#endif
#ifndef cRMTarsInv 
#define cRMTarsInv 1 
#endif
#ifndef cRFTarsInv 
#define cRFTarsInv 1 
#endif

#ifndef cLRTarsInv 
#define cLRTarsInv 0 
#endif
#ifndef cLMTarsInv 
#define cLMTarsInv 0 
#endif
#ifndef cLFTarsInv 
#define cLFTarsInv 0 
#endif

// Also define default BalanceDelay
#ifndef BALANCE_DELAY
#define BALANCE_DELAY 100
#endif


#ifndef QUADMODE
// Standard Hexapod...
// Servo Horn offsets
#ifdef cRRFemurHornOffset1   // per leg configuration
static const short cFemurHornOffset1[] PROGMEM = {
  cRRFemurHornOffset1, cRMFemurHornOffset1, cRFFemurHornOffset1, cLRFemurHornOffset1, cLMFemurHornOffset1, cLFFemurHornOffset1};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif

#ifdef cRRTibiaHornOffset1   // per leg configuration
static const short cTibiaHornOffset1[] PROGMEM = {
  cRRTibiaHornOffset1, cRMTibiaHornOffset1, cRFTibiaHornOffset1, cLRTibiaHornOffset1, cLMTibiaHornOffset1, cLFTibiaHornOffset1};
#define CTIBIAHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTibiaHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cTibiaHornOffset1
#define cTibiaHornOffset1  0
#endif
#define CTIBIAHORNOFFSET1(LEGI)  (cTibiaHornOffset1)
#endif

#ifdef c4DOF
#ifdef cRRTarsHornOffset1   // per leg configuration
static const short cTarsHornOffset1[] PROGMEM = {
  cRRTarsHornOffset1,  cRMTarsHornOffset1,  cRFTarsHornOffset1,  cLRTarsHornOffset1,  cLMTarsHornOffset1,  cLFTarsHornOffset1};
#define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cTarsHornOffset1
#define cTarsHornOffset1  0
#endif
#define CTARSHORNOFFSET1(LEGI)  cTarsHornOffset1
#endif
#endif

//Min / Max values
#ifndef SERVOS_DO_MINMAX
const short cCoxaMin[] PROGMEM = {
  cRRCoxaMin,  cRMCoxaMin,  cRFCoxaMin,  cLRCoxaMin,  cLMCoxaMin,  cLFCoxaMin};
const short cCoxaMax[] PROGMEM = {
  cRRCoxaMax,  cRMCoxaMax,  cRFCoxaMax,  cLRCoxaMax,  cLMCoxaMax,  cLFCoxaMax};
const short cFemurMin[] PROGMEM ={
  cRRFemurMin, cRMFemurMin, cRFFemurMin, cLRFemurMin, cLMFemurMin, cLFFemurMin};
const short cFemurMax[] PROGMEM ={
  cRRFemurMax, cRMFemurMax, cRFFemurMax, cLRFemurMax, cLMFemurMax, cLFFemurMax};
const short cTibiaMin[] PROGMEM ={
  cRRTibiaMin, cRMTibiaMin, cRFTibiaMin, cLRTibiaMin, cLMTibiaMin, cLFTibiaMin};
const short cTibiaMax[] PROGMEM = {
  cRRTibiaMax, cRMTibiaMax, cRFTibiaMax, cLRTibiaMax, cLMTibiaMax, cLFTibiaMax};

#ifdef c4DOF
const short cTarsMin[] PROGMEM = {
  cRRTarsMin, cRMTarsMin, cRFTarsMin, cLRTarsMin, cLMTarsMin, cLFTarsMin};
const short cTarsMax[] PROGMEM = {
  cRRTarsMax, cRMTarsMax, cRFTarsMax, cLRTarsMax, cLMTarsMax, cLFTarsMax};
#endif
#endif

// Servo inverse direction
const bool cCoxaInv[] = {cRRCoxaInv, cRMCoxaInv, cRFCoxaInv, cLRCoxaInv, cLMCoxaInv, cLFCoxaInv};
bool cFemurInv[] = {cRRFemurInv, cRMFemurInv, cRFFemurInv, cLRFemurInv, cLMFemurInv, cLFFemurInv};
const bool cTibiaInv[] = {cRRTibiaInv, cRMTibiaInv, cRFTibiaInv, cLRTibiaInv, cLMTibiaInv, cLFTibiaInv};

#ifdef c4DOF
const boolean cTarsInv[] = {cRRTarsInv, cRMTarsInv, cRFTarsInv, cLRTarsInv, cLMTarsInv, cLFTarsInv};
#endif  

//Leg Lengths
const byte cCoxaLength[] PROGMEM = {
  cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength};
const byte cFemurLength[] PROGMEM = {
  cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
const byte cTibiaLength[] PROGMEM = {
  cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};
#ifdef c4DOF
const byte cTarsLength[] PROGMEM = {
  cRRTarsLength, cRMTarsLength, cRFTarsLength, cLRTarsLength, cLMTarsLength, cLFTarsLength};
#endif


//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {
  cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
const short cOffsetZ[] PROGMEM = {
  cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};

//Default leg angle
const short cCoxaAngle[] PROGMEM = {
  cRRCoxaAngle, cRMCoxaAngle, cRFCoxaAngle, cLRCoxaAngle, cLMCoxaAngle, cLFCoxaAngle};

#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
const short cCoxaInitAngle[] PROGMEM = {
  cRRInitCoxaAngle, cRMInitCoxaAngle, cRFInitCoxaAngle, cLRInitCoxaAngle, cLMInitCoxaAngle, cLFInitCoxaAngle};
#endif

//Start positions for the leg
const short cInitPosX[] PROGMEM = {
  cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
const short cInitPosY[] PROGMEM = {
  cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
const short cInitPosZ[] PROGMEM = {
  cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};

//=============================================================================
#else
// Quads...
// Servo Horn offsets
#ifdef cRRFemurHornOffset1   // per leg configuration
static const short cFemurHornOffset1[] PROGMEM = {
  cRRFemurHornOffset1, cRFFemurHornOffset1, cLRFemurHornOffset1, cLFFemurHornOffset1};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif

#ifdef cRRTibiaHornOffset1   // per leg configuration
static const short cTibiaHornOffset1[] PROGMEM = {
  cRRTibiaHornOffset1, cRFTibiaHornOffset1, cLRTibiaHornOffset1, cLFTibiaHornOffset1};
#define CTIBIAHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTibiaHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cTibiaHornOffset1
#define cTibiaHornOffset1  0
#endif
#define CTIBIAHORNOFFSET1(LEGI)  (cTibiaHornOffset1)
#endif



#ifdef c4DOF
#ifdef cRRTarsHornOffset1   // per leg configuration
static const short cTarsHornOffset1[] PROGMEM = {
  cRRTarsHornOffset1, cRFTarsHornOffset1,  cLRTarsHornOffset1, cLFTarsHornOffset1};
#define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cTarsHornOffset1
#define cTarsHornOffset1  0
#endif
#define CTARSHORNOFFSET1(LEGI)  cTarsHornOffset1
#endif
#endif

//Min / Max values
#ifndef SERVOS_DO_MINMAX
const short cCoxaMin[] PROGMEM = {
  cRRCoxaMin,  cRFCoxaMin,  cLRCoxaMin,  cLFCoxaMin};
const short cCoxaMax[] PROGMEM = {
  cRRCoxaMax,  cRFCoxaMax,  cLRCoxaMax,  cLFCoxaMax};
const short cFemurMin[] PROGMEM ={
  cRRFemurMin, cRFFemurMin, cLRFemurMin, cLFFemurMin};
const short cFemurMax[] PROGMEM ={
  cRRFemurMax, cRFFemurMax, cLRFemurMax, cLFFemurMax};
const short cTibiaMin[] PROGMEM ={
  cRRTibiaMin, cRFTibiaMin, cLRTibiaMin, cLFTibiaMin};
const short cTibiaMax[] PROGMEM = {
  cRRTibiaMax, cRFTibiaMax, cLRTibiaMax, cLFTibiaMax};

#ifdef c4DOF
const short cTarsMin[] PROGMEM = {
  cRRTarsMin, cRFTarsMin, cLRTarsMin, cLFTarsMinMin};
const short cTarsMax[] PROGMEM = {
  cRRTarsMax, cRFTarsMax, cLRTarsMax, cLFTarsMax};
#endif
#endif

// Servo inverse direction
const bool cCoxaInv[] = {cRRCoxaInv, cRFCoxaInv, cLRCoxaInv, cLFCoxaInv};
bool cFemurInv[] = {cRRFemurInv, cRFFemurInv, cLRFemurInv, cLFFemurInv};
const bool cTibiaInv[] = {cRRTibiaInv, cRFTibiaInv, cLRTibiaInv, cLFTibiaInv};

#ifdef c4DOF
const boolean cTarsInv[] = {
    cRRTarsInv, cRFTarsInv, cLRTarsInv, cLFTarsInv};
#endif  
    


//Leg Lengths
const byte cCoxaLength[] PROGMEM = {
  cRRCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLFCoxaLength};
const byte cFemurLength[] PROGMEM = {
  cRRFemurLength, cRFFemurLength, cLRFemurLength, cLFFemurLength};
const byte cTibiaLength[] PROGMEM = {
  cRRTibiaLength, cRFTibiaLength, cLRTibiaLength, cLFTibiaLength};
#ifdef c4DOF
const byte cTarsLength[] PROGMEM = {
  cRRTarsLength, cRFTarsLength, cLRTarsLength, cLFTarsLength};
#endif


//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {
  cRROffsetX, cRFOffsetX, cLROffsetX, cLFOffsetX};
const short cOffsetZ[] PROGMEM = {
  cRROffsetZ, cRFOffsetZ, cLROffsetZ, cLFOffsetZ};

//Default leg angle
const short cCoxaAngle[] PROGMEM = {
  cRRCoxaAngle, cRFCoxaAngle, cLRCoxaAngle, cLFCoxaAngle};

#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
const short cCoxaInitAngle[] PROGMEM = {
  cRRInitCoxaAngle, cRFInitCoxaAngle, cLRInitCoxaAngle, cLFInitCoxaAngle};
#endif


//Start positions for the leg
const short cInitPosX[] PROGMEM = {
  cRRInitPosX, cRFInitPosX, cLRInitPosX, cLFInitPosX};
const short cInitPosY[] PROGMEM = {
  cRRInitPosY, cRFInitPosY, cLRInitPosY, cLFInitPosY};
const short cInitPosZ[] PROGMEM = {
  cRRInitPosZ, cRFInitPosZ, cLRInitPosZ, cLFInitPosZ};

#endif

// Define some globals for debug information
boolean g_fShowDebugPrompt;
boolean g_fDebugOutput;
boolean g_fEnableServos = true;

//--------------------------------------------------------------------
//[REMOTE]                 
#define cTravelDeadZone         4    //The deadzone for the analog input from the remote
//====================================================================
//[ANGLES]
float           CoxaAngle[CNT_LEGS];             //Actual Angle of the horizontal hip, decimals = 1
float           FemurAngle[CNT_LEGS];            //Actual Angle of the vertical hip, decimals = 1
float           TibiaAngle[CNT_LEGS];            //Actual Angle of the knee, decimals = 1
#ifdef c4DOF
float           TarsAngle[CNT_LEGS];             //Actual Angle of the knee, decimals = 1
#endif

//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]

float           LegPosX[CNT_LEGS];                //Actual X Posion of the Leg
float           LegPosY[CNT_LEGS];                //Actual Y Posion of the Leg
float           LegPosZ[CNT_LEGS];                //Actual Z Posion of the Leg
//--------------------------------------------------------------------
//[VARIABLES]
//byte            Index;                            //Index universal used
byte            LegIndex;                //Index used for leg Index Number

//GetSinCos / ArcCos
float           AngleDeg1;                        //Input Angle in degrees, decimals = 1
float           sinA;                             //Output Sinus of the given Angle, decimals = 4
float           cosA;                             //Output Cosinus of the given Angle, decimals = 4
float           AngleRad4;                        //Output Angle in radials, decimals = 4


//Body Inverse Kinematics
float           PosX;                             //Input position of the feet X
float           PosZ;                             //Input position of the feet Z
float           PosY;                             //Input position of the feet Y
float            BodyFKPosX;                       //Output Position X of feet with Rotation
float            BodyFKPosY;                       //Output Position Y of feet with Rotation
float            BodyFKPosZ;                       //Output Position Z of feet with Rotation

//Leg Inverse Kinematics
float            IKFeetPosX;                       //Input position of the Feet X
float            IKFeetPosY;                       //Input position of the Feet Y
float            IKFeetPosZ;                       //Input Position of the Feet Z
boolean         IKSolution;        //Output true if the solution is possible
boolean         IKSolutionWarning;    //Output true if the solution is NEARLY possible
boolean         IKSolutionError;    //Output true if the solution is NOT possible
//--------------------------------------------------------------------
//[TIMING]
unsigned long   lTimerStart;    //Start time of the calculation cycles
unsigned long   lTimerEnd;        //End time of the calculation cycles
byte            CycleTime;        //Total Cycle time

word            ServoMoveTime;        //Time for servo updates
word            PrevServoMoveTime;    //Previous time for the servo updates

//--------------------------------------------------------------------
//[GLOABAL]
//--------------------------------------------------------------------

// Define our global Input Control State object
INCONTROLSTATE   g_InControlState;      // This is our global Input control state object...

// Define our ServoWriter class
ServoDriver  g_ServoDriver;      // our global servo driver class

boolean         g_fLowVoltageShutdown;    // If set the bot shuts down because the input voltage is to low
word            Voltage;


//--boolean         g_InControlState.fRobotOn;            //Switch to turn on Phoenix
//--boolean         g_InControlState.fPrev_RobotOn;        //Previous loop state 
//--------------------------------------------------------------------
//[Balance]
float            TotalTransX;
float            TotalTransZ;
float            TotalTransY;
float            TotalYBal1;
float            TotalXBal1;
float            TotalZBal1;
//[Single Leg Control]
byte            PrevSelectedLeg;
boolean         AllDown;

//[gait - State]
// Note: Information about the current gait is now part of the g_InControlState...
boolean         TravelRequest;          //Temp to check if the gait is in motion

float            GaitPosX[CNT_LEGS];               //Array containing Relative X position corresponding to the Gait
float            GaitPosY[CNT_LEGS];               //Array containing Relative Y position corresponding to the Gait
float            GaitPosZ[CNT_LEGS];               //Array containing Relative Z position corresponding to the Gait
float            GaitRotY[CNT_LEGS];               //Array containing Relative Y rotation corresponding to the Gait

//boolean           GaitLegInAir[CNT_LEGS];     // True if leg is in the air
//byte          GaitNextLeg;                // The next leg which will be lifted

boolean         fWalking;            //  True if the robot are walking
byte            bExtraCycle;          // Forcing some extra timed cycles for avoiding "end of gait bug"
#define         cGPlimit 2           // GP=GaitPos testing different limits

boolean        g_fRobotUpsideDown;    // Is the robot upside down?
boolean        fRobotUpsideDownPrev;
//=============================================================================
// Define our default standard Gaits
//=============================================================================
#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 50
#define DEFAULT_SLOW_GAIT 70
#endif

//cRR=0, cRF, cLR, cLF, CNT_LEGS};

#ifndef OVERWRITE_GAITS
#ifndef QUADMODE
//  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground, 
//     Quad extra: COGAngleStart, COGAngleStep, CogRadius, COGCCW
//                      { RR, <RM> RF, LR, <LM>, LF}
#ifdef DISPLAY_GAIT_NAMES
extern "C" {
  // Move the Gait Names to program space...
  const char s_szGN1[] PROGMEM = "Ripple 12";
  const char s_szGN2[] PROGMEM = "Tripod 8";
  const char s_szGN3[] PROGMEM = "Tripple 12";
  const char s_szGN4[] PROGMEM = "Tripple 16";
  const char s_szGN5[] PROGMEM = "Wave 24";
  const char s_szGN6[] PROGMEM = "Tripod 6";
};  
#endif

PHOENIXGAIT APG[] = { 
    {DEFAULT_SLOW_GAIT, 12, 3, 2, 2, 8, 3, {7, 11, 3, 1, 5, 9} GATENAME(s_szGN1)},        // Ripple 12
    {DEFAULT_SLOW_GAIT, 8, 3, 2, 2, 4, 3, {1, 5, 1, 5, 1, 5} GATENAME(s_szGN2)},           //Tripod 8 steps
    {DEFAULT_GAIT_SPEED, 12, 3, 2, 2, 8, 3, {5, 10, 3, 11, 4, 9} GATENAME(s_szGN3) },      //Triple Tripod 12 step
    {DEFAULT_GAIT_SPEED, 16, 5, 3, 4, 10, 1, {6, 13, 4, 14, 5, 12} GATENAME(s_szGN4)},    // Triple Tripod 16 steps, use 5 lifted positions
    {DEFAULT_SLOW_GAIT, 24, 3, 2, 2, 20, 3, {13, 17, 21, 1, 5, 9} GATENAME(s_szGN5)},     //Wave 24 steps
    {DEFAULT_GAIT_SPEED, 6, 2, 1, 2, 4, 1, {1, 4, 1, 4, 1, 4} GATENAME(s_szGN6)}          //Tripod 6 steps
};    

#else
#ifdef DISPLAY_GAIT_NAMES
extern "C" {
  // Move the Gait Names to program space...
  const char s_szGN1[] PROGMEM = "Ripple 12";
  const char s_szGN2[] PROGMEM = "Tripod 8";
}
#endif
PHOENIXGAIT APG[] = { 
    {DEFAULT_GAIT_SPEED, 16, 3, 2, 2, 12, 3, 2250, 3600/16, 30, true, {5, 9, 1, 13} GATENAME(s_szGN1)},            // Wave 16
    {1, 28, 3, 2, 2, 24, 3, 2250, 3600/28, 30, true, {8, 15, 1, 22} GATENAME(s_szGN2)}                             // Wave 28?
};    

#endif
#endif
//--------------------------------------------------------------------

#ifdef ADD_GAITS
byte NUM_GAITS = sizeof(APG)/sizeof(APG[0]) + sizeof(APG_EXTRA)/sizeof(APG_EXTRA[0]);
#else
byte NUM_GAITS = sizeof(APG)/sizeof(APG[0]);
#endif



//=============================================================================
// Function prototypes
//=============================================================================
extern void GaitSelect(void);
extern void  WriteOutputs(void);    
extern void SingleLegControl(void);
extern void GaitSeq(void);
extern void BalanceBody(void);
extern void CheckAngles();

extern void    PrintSystemStuff(void);            // Try to see why we fault...


//extern void  GaitGetNextLeg(byte GaitStep);
extern void BalCalcOneLeg (float PosX, float PosZ, float PosY, byte BalLegNr);
extern void BodyFK (float PosX, float PosZ, float PosY, float RotationY, byte BodyIKLeg) ;
extern void LegIK (float feetPosX, float feetPosY, float feetPosZ, int legIndex);
extern void Gait (byte GaitCurrentLegNr);
extern void GetSinCos(float AngleDeg);

extern void StartUpdateServos(void);
extern boolean TerminalMonitor(void);

//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup(){
#ifdef OPT_SKETCHSETUP
  SketchSetup();
#endif  
  delay(500);  // giveus some time to init
  g_fShowDebugPrompt = true;
  g_fDebugOutput = false;
#ifdef DBGSerial    
  DBGSerial.begin(38400);
#endif
  // Init our ServoDriver
  g_ServoDriver.Init();

  //Checks to see if our Servo Driver support a GP Player
  //    DBGSerial.write("Program Start\n\r");
  // debug stuff
  delay(10);


  // Setup Init Positions
  for (LegIndex= 0; LegIndex < CNT_LEGS; LegIndex++ )
  {
    LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);    //Set start positions for each leg
    LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
    LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);  
  }

  ResetLegInitAngles();

  //Single leg control. Make sure no leg is selected
  #ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg = 255; // No Leg selected
  PrevSelectedLeg = 255;
#endif
  //Body Positions
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;

  //Body Rotations
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.BodyRotOffset.x = 0;
  g_InControlState.BodyRotOffset.y = 0;        //Input Y offset value to adjust centerpoint of rotation
  g_InControlState.BodyRotOffset.z = 0;


  //Gait
  g_InControlState.GaitType = 0; 
  g_InControlState.BalanceMode = 0;
  g_InControlState.LegLiftHeight = 50;
  g_InControlState.ForceGaitStepCnt = 0;    // added to try to adjust starting positions depending on height...
  g_InControlState.GaitStep = 1;
  GaitSelect();

#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle = cTurretTiltInit;    // the tile for the turret
#endif

  g_InputController.Init();

  // Servo Driver
  ServoMoveTime = 150;
  g_InControlState.fRobotOn = 0;
  g_fLowVoltageShutdown = false;
#ifdef DEBUG_IOPINS    
  //  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
#endif    
#ifdef OPT_WALK_UPSIDE_DOWN
  g_fRobotUpsideDown = false; //Assume off... 
#ifdef DBGSerial  
  DBGSerial.println(IsRobotUpsideDown, DEC);
#endif  
#endif


}


//=============================================================================
// Loop: the main arduino main Loop function
//=============================================================================


void loop(void)
{
  //Start time
  unsigned long lTimeWaitEnd;
  lTimerStart = millis(); 
  DoBackgroundProcess();
  //Read input
  CheckVoltage();        // check our voltages...
  if (!g_fLowVoltageShutdown) {
    //    DebugWrite(A0, HIGH);
    g_InputController.ControlInput();
    //    DebugWrite(A0, LOW);
  }
  WriteOutputs();        // Write Outputs


#ifdef OPT_GPPLAYER
    //GP Player
  g_ServoDriver.GPPlayer();
  if (g_ServoDriver.FIsGPSeqActive())
    return;  // go back to process the next message
#endif

  //Single leg control
  SingleLegControl ();
  DoBackgroundProcess();

  //Gait
  GaitSeq();

  DoBackgroundProcess();

  //Balance calculations
  TotalTransX = 0;     //reset values used for calculation of balance
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal1 = 0;
  TotalYBal1 = 0;
  TotalZBal1 = 0;
  
  if (g_InControlState.BalanceMode) {
#ifdef DEBUG
      if (g_fDebugOutput) {
  TravelRequest = (abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) 
    || (abs(g_InControlState.TravelLength.y)>cTravelDeadZone) || (g_InControlState.ForceGaitStepCnt != 0) || fWalking;

        DBGSerial.print("T("); 
        DBGSerial.print(fWalking, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(g_InControlState.TravelLength.x,DEC); 
        DBGSerial.print(","); 
        DBGSerial.print(g_InControlState.TravelLength.y,DEC); 
        DBGSerial.print(","); 
        DBGSerial.print(g_InControlState.TravelLength.z,DEC); 
        DBGSerial.print(")"); 
      }
#endif
    for (LegIndex = 0; LegIndex < (CNT_LEGS/2); LegIndex++) {    // balance calculations for all Right legs

      DoBackgroundProcess();
      BalCalcOneLeg (-LegPosX[LegIndex]+GaitPosX[LegIndex], LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
          (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }

    for (LegIndex = (CNT_LEGS/2); LegIndex < CNT_LEGS; LegIndex++) {    // balance calculations for all Right legs
      DoBackgroundProcess();
      BalCalcOneLeg(LegPosX[LegIndex]+GaitPosX[LegIndex], LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
          (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
  }


  //Reset IKsolution indicators 
  IKSolution = 0 ;
  IKSolutionWarning = 0; 
  IKSolutionError = 0 ;

  //Do IK for all Right legs
#ifdef DEBUG
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print(g_InControlState.GaitStep,DEC);
        DBGSerial.print(":");
    }
#endif

  for (LegIndex = 0; LegIndex < (CNT_LEGS/2); LegIndex++) {    
    DoBackgroundProcess();
    BodyFK(-LegPosX[LegIndex]+g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
        LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
        LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
        GaitRotY[LegIndex], LegIndex);

    LegIK (LegPosX[LegIndex]-g_InControlState.BodyPos.x+BodyFKPosX-(GaitPosX[LegIndex] - TotalTransX), 
        LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
        LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Do IK for all Left legs  
  for (LegIndex = (CNT_LEGS/2); LegIndex < CNT_LEGS; LegIndex++) {
    DoBackgroundProcess();
    BodyFK(LegPosX[LegIndex]-g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
        LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
        LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
        GaitRotY[LegIndex], LegIndex);
    LegIK (LegPosX[LegIndex]+g_InControlState.BodyPos.x-BodyFKPosX+GaitPosX[LegIndex] - TotalTransX,
        LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
        LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){ //Need to set them back for not messing with the SmoothControl
    g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
    g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
    g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
  }
#endif
  //Check mechanical limits
  CheckAngles();

  //Drive Servos
  if (g_InControlState.fRobotOn) {
    if (g_InControlState.fRobotOn && !g_InControlState.fPrev_RobotOn) {
      MSound(3, 60, 2000, 80, 2250, 100, 2500);
#ifdef USEXBEE
      XBeePlaySounds(3, 60, 2000, 80, 2250, 100, 2500);
#endif            

    }

    //Calculate Servo Move time
    if ((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) ||
      (abs(g_InControlState.TravelLength.y*2)>cTravelDeadZone)) {         
      ServoMoveTime = g_InControlState.gaitCur.NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;

      //Add aditional delay when Balance mode is on
      if (g_InControlState.BalanceMode)
        ServoMoveTime = ServoMoveTime + BALANCE_DELAY;
    } 
    else //Movement speed excl. Walking
    ServoMoveTime = 200 + g_InControlState.SpeedControl;

    // note we broke up the servo driver into start/commit that way we can output all of the servo information
    // before we wait and only have the termination information to output after the wait.  That way we hopefully
    // be more accurate with our timings...
    DoBackgroundProcess();
    StartUpdateServos();

    // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed 
    //before sending the next one


    // Finding any incident of GaitPos/Rot <>0:
    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
      if ( (GaitPosX[LegIndex] > cGPlimit) || (GaitPosX[LegIndex] < -cGPlimit)
        || (GaitPosZ[LegIndex] > cGPlimit) || (GaitPosZ[LegIndex] < -cGPlimit) 
        || (GaitRotY[LegIndex] > cGPlimit) || (GaitRotY[LegIndex] < -cGPlimit))    {

        bExtraCycle = g_InControlState.gaitCur.NrLiftedPos + 1;//For making sure that we are using timed move until all legs are down
        break;
      }
    }
    if (bExtraCycle>0){ 
      bExtraCycle--;
      fWalking = !(bExtraCycle==0);

      //Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;

      DebugWrite(A1, HIGH);
      do {
        // Wait the appropriate time, call any background process while waiting...
        DoBackgroundProcess();
      } 
      while (millis() < lTimeWaitEnd);
      DebugWrite(A1, LOW);
#ifdef DEBUG_X
      if (g_fDebugOutput) {

        DBGSerial.print("BRX:");
        DBGSerial.print(g_InControlState.BodyRot1.x,DEC); 
        DBGSerial.print("W?:");
         DBGSerial.print(fWalking,DEC);  
         DBGSerial.print(" GS:");
         DBGSerial.print(g_InControlState.GaitStep,DEC);  
         //Debug LF leg
         DBGSerial.print(" GPZ:");
         DBGSerial.print(GaitPosZ[cLF],DEC);
         DBGSerial.print(" GPY:");
         DBGSerial.println(GaitPosY[cLF],DEC);
      }
#endif
    }
#ifdef DEBUG_X
    if (g_fDebugOutput) {


      DBGSerial.print("TY:");
      DBGSerial.print(TotalYBal1,DEC); 
      DBGSerial.print(" LFZ:");
      DBGSerial.println(LegPosZ[cLF],DEC);
      DBGSerial.flush();  // see if forcing it to output helps...
    }
#endif
    // Only do commit if we are actually doing something...
    DebugToggle(A2);
    g_ServoDriver.CommitServoDriver(ServoMoveTime);


  } 
  else {
    //Turn the bot off - May need to add ajust here...
    if (g_InControlState.fPrev_RobotOn || (AllDown= 0)) {
      ServoMoveTime = 600;
      StartUpdateServos();
      g_ServoDriver.CommitServoDriver(ServoMoveTime);
      MSound(3, 100, 2500, 80, 2250, 60, 2000);
#ifdef USEXBEE            
      XBeePlaySounds(3, 100, 2500, 80, 2250, 60, 2000);
#endif    
      lTimeWaitEnd = millis() + 600;    // setup to process background stuff while we wait...
      do {
        // Wait the appropriate time, call any background process while waiting...
        DoBackgroundProcess();
      } 
      while (millis() < lTimeWaitEnd);
      //delay(600);
    } 
    else {
      g_ServoDriver.FreeServos();
    }

    // Allow the Servo driver to do stuff durint our idle time
    g_ServoDriver.IdleTime();

    // We also have a simple debug monitor that allows us to 
    // check things. call it here..
#ifdef OPT_TERMINAL_MONITOR  
    if (TerminalMonitor())
      return;           
#endif
    delay(20);  // give a pause between times we call if nothing is happening
  }

  PrevServoMoveTime = ServoMoveTime;

  //Store previous g_InControlState.fRobotOn State
  if (g_InControlState.fRobotOn)
    g_InControlState.fPrev_RobotOn = 1;
  else
    g_InControlState.fPrev_RobotOn = 0;
}


void StartUpdateServos()
{        
  byte    LegIndex;

  // First call off to the init...
  g_ServoDriver.BeginServoUpdate();    // Start the update 

    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
#ifdef c4DOF
    g_ServoDriver.OutputServoInfoForLeg(LegIndex, 
        cCoxaInv[LegIndex]? -CoxaAngle[LegIndex] : CoxaAngle[LegIndex], 
        cFemurInv[LegIndex]? -FemurAngle[LegIndex] : FemurAngle[LegIndex], 
        cTibiaInv[LegIndex]? -TibiaAngle[LegIndex] : TibiaAngle[LegIndex], 
        cTarsInv[LegIndex]? -TarsAngle[LegIndex] : TarsAngle[LegIndex]);
#else
    g_ServoDriver.OutputServoInfoForLeg(LegIndex, 
        cCoxaInv[LegIndex]? -CoxaAngle[LegIndex] : CoxaAngle[LegIndex], 
        cFemurInv[LegIndex]? -FemurAngle[LegIndex] : FemurAngle[LegIndex], 
        cTibiaInv[LegIndex]? -TibiaAngle[LegIndex] : TibiaAngle[LegIndex]);
#endif      
  }
#ifdef cTurretRotPin
  g_ServoDriver.OutputServoInfoForTurret(g_InControlState.TurretRotAngle, g_InControlState.TurretTiltAngle);  // fist just see if it will talk
#endif  
}




//--------------------------------------------------------------------
//[WriteOutputs] Updates the state of the leds
//--------------------------------------------------------------------
void WriteOutputs(void)
{
#ifdef cEyesPin
  digitalWrite(cEyesPin, Eyes);
#endif        
}
//--------------------------------------------------------------------
//[CHECK VOLTAGE]
//Reads the input voltage and shuts down the bot when the power drops
byte s_bLVBeepCnt;
boolean CheckVoltage() {
#ifdef cTurnOffVol
  // Moved to Servo Driver - BUGBUG: Need to do when I merge back...
  //    Voltage = analogRead(cVoltagePin); // Battery voltage 
  //    Voltage = ((long)Voltage*1955)/1000;
  Voltage = g_ServoDriver.GetBatteryVoltage();

  // BUGBUG:: if voltage is 0 it failed to retrieve don't hang program...
  //    if (!Voltage)
  //      return;

  if (!g_fLowVoltageShutdown) {
    if ((Voltage < cTurnOffVol) || (Voltage >= 1999)) {
#ifdef DBGSerial          
      DBGSerial.print("Voltage went low, turn off robot ");
      DBGSerial.println(Voltage, DEC);
#endif            
      //Turn off
      g_InControlState.BodyPos.x = 0;
      g_InControlState.BodyPos.y = 0;
      g_InControlState.BodyPos.z = 0;
      g_InControlState.BodyRot1.x = 0;
      g_InControlState.BodyRot1.y = 0;
      g_InControlState.BodyRot1.z = 0;
      g_InControlState.TravelLength.x = 0;
      g_InControlState.TravelLength.z = 0;

#ifdef OPT_SINGLELEG
      g_InControlState.TravelLength.y = 0;
      g_InControlState.SelectedLeg = 255;
#endif
      g_fLowVoltageShutdown = 1;
      s_bLVBeepCnt = 0;    // how many times we beeped...
      g_InControlState.fRobotOn = false;
    }
#ifdef cTurnOnVol
  } 
  else if ((Voltage > cTurnOnVol) && (Voltage < 1999)) {
#ifdef DBGSerial
    DBGSerial.print(F("Voltage restored: "));
    DBGSerial.println(Voltage, DEC);
#endif          
    g_fLowVoltageShutdown = 0;

#endif      
  } 
  else {
    if (s_bLVBeepCnt < 5) {
      s_bLVBeepCnt++;
#ifdef DBGSerial
      DBGSerial.println(Voltage, DEC);
#endif          
      MSound( 1, 45, 2000);
    }
    delay(2000);
  }
#endif  
  return g_fLowVoltageShutdown;
}

//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void)
{
#ifdef OPT_SINGLELEG

  //Check if all legs are down
  AllDown = (LegPosY[cRF]==(short)pgm_read_word(&cInitPosY[cRF])) && 
    (LegPosY[cRR]==(short)pgm_read_word(&cInitPosY[cRR])) && 
    (LegPosY[cLR]==(short)pgm_read_word(&cInitPosY[cLR])) && 
#ifndef QUADMODE
    (LegPosY[cRM]==(short)pgm_read_word(&cInitPosY[cRM])) && 
    (LegPosY[cLM]==(short)pgm_read_word(&cInitPosY[cLM])) && 
#endif  
    (LegPosY[cLF]==(short)pgm_read_word(&cInitPosY[cLF]));

  if (g_InControlState.SelectedLeg<= (CNT_LEGS-1)) {
    if (g_InControlState.SelectedLeg!=PrevSelectedLeg) {
      if (AllDown) { //Lift leg a bit when it got selected
        LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg])-20;

        //Store current status
        PrevSelectedLeg = g_InControlState.SelectedLeg;
      } 
      else {//Return prev leg back to the init position
        LegPosX[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosX[PrevSelectedLeg]);
        LegPosY[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosY[PrevSelectedLeg]);
        LegPosZ[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosZ[PrevSelectedLeg]);
      }
    } 
    else if (!g_InControlState.fSLHold) {
      //LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;
      LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.y;// Using DIY remote Zenta prefer it this way
      LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.x;
      LegPosZ[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.z;     
    }
  } 
  else {//All legs to init position
    if (!AllDown) {
      for(LegIndex = 0; LegIndex <= (CNT_LEGS-1);LegIndex++) {
        LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);
        LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
        LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);
      }
    } 
    if (PrevSelectedLeg!=255)
      PrevSelectedLeg = 255;
  }
#endif
}


void GaitSelect(void)
{
  //Gait selector
  // First pass simply use defined table, next up will allow robots to add or relace set...
  if (g_InControlState.GaitType < NUM_GAITS) {
#ifdef ADD_GAITS
    if (g_InControlState.GaitType < (sizeof(APG_EXTRA)/sizeof(APG_EXTRA[0])))
        g_InControlState.gaitCur = APG_EXTRA[g_InControlState.GaitType];
    else
        g_InControlState.gaitCur = APG[g_InControlState.GaitType - (sizeof(APG_EXTRA)/sizeof(APG_EXTRA[0]))];
#else
    g_InControlState.gaitCur = APG[g_InControlState.GaitType];
#endif
  }

#ifdef DBGSerial  
  if (g_fDebugOutput) {
    DBGSerial.print(g_InControlState.GaitType, DEC);
    DBGSerial.print("    {");
    DBGSerial.print(g_InControlState.gaitCur.NomGaitSpeed, DEC);
    DBGSerial.print(", ");
    DBGSerial.print(g_InControlState.gaitCur.StepsInGait, DEC); 
    DBGSerial.print(", ");
    DBGSerial.print(g_InControlState.gaitCur.NrLiftedPos, DEC); 
    DBGSerial.print(", ");
    DBGSerial.print(g_InControlState.gaitCur.FrontDownPos, DEC);
    DBGSerial.print(", ");
    DBGSerial.print(g_InControlState.gaitCur.LiftDivFactor, DEC);
    DBGSerial.print(", ");
    DBGSerial.print(g_InControlState.gaitCur.TLDivFactor, DEC);  
    DBGSerial.print(", ");
    DBGSerial.print(g_InControlState.gaitCur.HalfLiftHeight, DEC); 
    DBGSerial.print(", {");
    for (int il = 0; il < CNT_LEGS; il++) {
        DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[il], DEC);
        if (il < (CNT_LEGS-1))
            DBGSerial.print(", ");
    }
    DBGSerial.println("}}");
  }  
#endif  

}    

//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void)
{
  //Check if the Gait is in motion - If not if we are going to start a motion try to align our Gaitstep to start with a good foot
  // for the direction we are about to go...
  
  if (fWalking || (g_InControlState.ForceGaitStepCnt != 0))
    TravelRequest = true;   // Is walking or was walking...
  else {
    TravelRequest = (abs(g_InControlState.TravelLength.x)>cTravelDeadZone) 
        || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) 
        || (abs(g_InControlState.TravelLength.y)>cTravelDeadZone) ;

    if (TravelRequest) {
#ifdef QUADCODE
        // just start walking - Try to guess a good foot to start off on...
        if (g_InControlState.TravelLength.z < 0) 
            g_InControlState.GaitStep = ((g_InControlState.TravelLength.X < 0)? g_InControlState.gaitCur.GaitLegNr[cLR] : g_InControlState.gaitCur.GaitLegNr[cRR]);
        else 
            g_InControlState.GaitStep = ((g_InControlState.TravelLength.X < 0)? g_InControlState.gaitCur.GaitLegNr[cLF] : g_InControlState.gaitCur.GaitLegNr[cRF]);
        // And lets backup a few Gaitsteps before this to allow it to start the up swing... 
        g_InControlState.GaitStep = ((g_InControlState.GaitStep > g_InControlState.gaitCur.FrontDownPos)? (g_InControlState.GaitStep - g_InControlState.gaitCur.FrontDownPos) : (g_InControlState.GaitStep + g_InControlState.gaitCur.StepsInGait - g_InControlState.gaitCur.FrontDownPos);
#endif      
    }
    else {    //Clear values under the cTravelDeadZone
      g_InControlState.TravelLength.x=0;
      g_InControlState.TravelLength.z=0;
      g_InControlState.TravelLength.y=0;//Gait NOT in motion, return to home position
    } 
  }

  //Calculate Gait sequence
  for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) { // for all legs
    Gait(LegIndex);
  }    // next leg

  //Advance to the next step
  g_InControlState.GaitStep++;
  if (g_InControlState.GaitStep>g_InControlState.gaitCur.StepsInGait)
    g_InControlState.GaitStep = 1;

  // If we have a force count decrement it now... 
  if (g_InControlState.ForceGaitStepCnt)
    g_InControlState.ForceGaitStepCnt--;
}


//--------------------------------------------------------------------
//[GAIT]
void Gait (byte GaitCurrentLegNr)
{

  // Try to reduce the number of time we look at GaitLegnr and Gaitstep
  short int LegStep = g_InControlState.GaitStep - g_InControlState.gaitCur.GaitLegNr[GaitCurrentLegNr];

  //Leg middle up position OK
  //Gait in motion                                                                                    
  // For Lifted pos = 1, 3, 5
  if ((TravelRequest && (g_InControlState.gaitCur.NrLiftedPos&1) && 
    LegStep==0) || (!TravelRequest && LegStep==0 && ((abs(GaitPosX[GaitCurrentLegNr])>2) || 
    (abs(GaitPosZ[GaitCurrentLegNr])>2) || (abs(GaitRotY[GaitCurrentLegNr])>2)))) { //Up
    GaitPosX[GaitCurrentLegNr] = 0;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
    GaitPosZ[GaitCurrentLegNr] = 0;
    GaitRotY[GaitCurrentLegNr] = 0;
  }
  //Optional Half heigth Rear (2, 3, 5 lifted positions)
  else if (((g_InControlState.gaitCur.NrLiftedPos==2 && LegStep==0) || (g_InControlState.gaitCur.NrLiftedPos>=3 && 
    (LegStep==-1 || LegStep==(g_InControlState.gaitCur.StepsInGait-1))))
    && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/g_InControlState.gaitCur.LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+g_InControlState.gaitCur.HalfLiftHeight);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/g_InControlState.gaitCur.LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/g_InControlState.gaitCur.LiftDivFactor;
  }    
  // _A_      
  // Optional Half heigth front (2, 3, 5 lifted positions)
  else if ((g_InControlState.gaitCur.NrLiftedPos>=2) && (LegStep==1 || LegStep==-(g_InControlState.gaitCur.StepsInGait-1)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/g_InControlState.gaitCur.LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+g_InControlState.gaitCur.HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/g_InControlState.gaitCur.LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/g_InControlState.gaitCur.LiftDivFactor;
  }

  //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
  else if (((g_InControlState.gaitCur.NrLiftedPos==5 && (LegStep==-2 ))) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/2;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
    GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/2;
  }         

  //Optional Half heigth Front 5 LiftedPos (5 lifted positions)
  else if ((g_InControlState.gaitCur.NrLiftedPos==5) && (LegStep==2 || LegStep==-(g_InControlState.gaitCur.StepsInGait-2)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;
  }
  //_B_
  //Leg front down position //bug here?  From _A_ to _B_ there should only be one gaitstep, not 2!
  //For example, where is the case of LegStep==0+2 executed when NRLiftedPos=3?
  else if ((LegStep==g_InControlState.gaitCur.FrontDownPos || LegStep==-(g_InControlState.gaitCur.StepsInGait-g_InControlState.gaitCur.FrontDownPos)) && GaitPosY[GaitCurrentLegNr]<0) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;         
    GaitPosY[GaitCurrentLegNr] = 0; 
  }

  //Move body forward      
  else {
    GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (g_InControlState.TravelLength.x/(short)g_InControlState.gaitCur.TLDivFactor);
    GaitPosY[GaitCurrentLegNr] = 0; 
    GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (g_InControlState.TravelLength.z/(short)g_InControlState.gaitCur.TLDivFactor);
    GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (g_InControlState.TravelLength.y/(short)g_InControlState.gaitCur.TLDivFactor);
  }

}  

//--------------------------------------------------------------------
//[BalCalcOneLeg]
#define PI 3.14159265

void BalCalcOneLeg (float PosX, float PosZ, float PosY, byte BalLegNr)
{
    float            cpr_x;                        //Final X value for centerpoint of rotation
    float            cpr_y;                    //Final Y value for centerpoint of rotation
    float            cpr_z;                    //Final Z value for centerpoint of rotation


#ifdef QUADMODE
  if (g_InControlState.gaitCur.COGAngleStep1 == 0) {  // In Quad mode only do those for those who don't support COG Balance...
#endif

      //Calculating totals from center of the body to the feet
      cpr_z = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
      cpr_x = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
      cpr_y = 150 + PosY;        // using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +
      
      TotalTransY += PosY;
      TotalTransZ += cpr_z;
      TotalTransX += cpr_x;

	TotalYBal1 += atan2(cpr_z, cpr_x)*180.0/PI;
#ifdef DEBUG
      if (g_fDebugOutput) {
          DBGSerial.print(" ");
          DBGSerial.print(cpr_x, DEC);
          DBGSerial.print(":");
          DBGSerial.print(cpr_y, DEC);
          DBGSerial.print(":");
          DBGSerial.print(cpr_z, DEC);
          DBGSerial.print(":");
          DBGSerial.print(TotalYBal1, DEC);
      }    
#endif

	TotalZBal1 += (atan2(cpr_y, cpr_x)*180.0)/PI -90.0;  //Rotate balance circle 90.0 deg

	TotalXBal1 += (atan2(cpr_y, cpr_z)*180.0) / PI - 90.0;//Rotate balance circle 90.0 deg

#ifdef QUADMODE
    }
#endif  

}  
//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void)
{
#ifdef QUADMODE
  if (g_InControlState.gaitCur.COGAngleStep1 == 0) {  // In Quad mode only do those for those who don't support COG Balance...
#endif
      TotalTransZ = TotalTransZ/BalanceDivFactor ;
      TotalTransX = TotalTransX/BalanceDivFactor;
      TotalTransY = TotalTransY/BalanceDivFactor;

#ifndef QUADMODE // ??? on PhantomX Hex at no movment YBal1 = 1800, on Quad = 0...  Need to experiment
      if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
		TotalYBal1 -=  180.0;
      else
		TotalYBal1 += 180.0;
#endif        

      if (TotalZBal1 < -180.0)    //Compensate for extreme balance positions that causes overflow
		TotalZBal1 += 360;

      if (TotalXBal1 < -180.0)    //Compensate for extreme balance positions that causes overflow
		TotalXBal1 += 360;

      //Balance rotation
      TotalYBal1 = -TotalYBal1/BalanceDivFactor;
      TotalXBal1 = -TotalXBal1/BalanceDivFactor;
      TotalZBal1 = TotalZBal1/BalanceDivFactor;
#ifdef DEBUG
      if (g_fDebugOutput) {
          DBGSerial.print(" L ");
          DBGSerial.print(BalanceDivFactor, DEC);
          DBGSerial.print(" TTrans: ");
          DBGSerial.print(TotalTransX, DEC);
          DBGSerial.print(" ");
          DBGSerial.print(TotalTransY, DEC);
          DBGSerial.print(" ");
          DBGSerial.print(TotalTransZ, DEC);
          DBGSerial.print(" TBal: ");
          DBGSerial.print(TotalXBal1, DEC);
          DBGSerial.print(" ");
          DBGSerial.print(TotalYBal1, DEC);
          DBGSerial.print(" ");
          DBGSerial.println(TotalZBal1, DEC);
      }
#endif
#ifdef QUADMODE
  } else {  
    // Quad mode with COG balance mode...
      byte COGShiftNeeded;
      byte BalCOGTransX;
      byte BalCOGTransZ;
      word COGAngle;
      float BalTotTravelLength;

      COGShiftNeeded = TravelRequest;
      for (LegIndex = 0; LegIndex <= CNT_LEGS; LegIndex++)
      {
        // Check if the cog needs to be shifted (travelRequest or legs goto home.)
        COGShiftNeeded = COGShiftNeeded || (abs(GaitPosX[LegIndex])>2) || (abs(GaitPosZ[LegIndex])>2) || (abs(GaitRotY[LegIndex])>2);
      }

      if (COGShiftNeeded) {
        if (g_InControlState.gaitCur.COGCCW) {
          COGAngle = g_InControlState.gaitCur.COGAngleStart1 - (g_InControlState.GaitStep-1) * g_InControlState.gaitCur.COGAngleStep1;
        } else {
          COGAngle = g_InControlState.gaitCur.COGAngleStart1 + (g_InControlState.GaitStep-1) * g_InControlState.gaitCur.COGAngleStep1;
        }
        GetSinCos(COGAngle);
        TotalTransX = g_InControlState.gaitCur.COGRadius * sinA;
        TotalTransZ = g_InControlState.gaitCur.COGRadius * cosA;
    
#ifdef DEBUG
        if (g_fDebugOutput) {
          DBGSerial.print(" TotalTransX: ");
          DBGSerial.print(TotalTransX, DEC);
          DBGSerial.print(" TotalTransZ: ");
          DBGSerial.print(TotalTransZ, DEC);
        }
#endif
        // Add direction variable. The body will not shift in the direction you're walking
        if (((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone)) && (abs(g_InControlState.TravelLength.y)<=cTravelDeadZone) ) {
        //if(TravelRotationY = 0) then

          BalTotTravelLength = sqrt(abs(g_InControlState.TravelLength.x * g_InControlState.TravelLength.x) + abs(g_InControlState.TravelLength.z*g_InControlState.TravelLength.z));
          BalCOGTransX = abs(g_InControlState.TravelLength.z)*c2DEC/BalTotTravelLength;
          BalCOGTransZ = abs(g_InControlState.TravelLength.x)*c2DEC/BalTotTravelLength;
          TotalTransX = TotalTransX*BalCOGTransX/c2DEC;
          TotalTransZ = TotalTransZ*BalCOGTransZ/c2DEC;
        }
  
#ifdef DEBUG
        if (g_fDebugOutput) {
          DBGSerial.print(" COGRadius: ");
          DBGSerial.print(g_InControlState.gaitCur.COGRadius, DEC);
          DBGSerial.print(" TotalTransX: ");
          DBGSerial.print(TotalTransX, DEC);
          DBGSerial.print(" TotalTransZ: ");
          DBGSerial.println(TotalTransZ, DEC);
        }
    }
#endif
      }  
#endif  
}


//=============================================================================
// GetSinCos:  Get the sinus and cosinus from the angle +/- multiple circles
// angleDeg     - Input Angle in degrees
// cosA         - Output Sinus of angleDeg
// sinA         - Output Cosinus of angleDeg
//=============================================================================

void GetSinCos ( float angleDeg )
  {
    float absangleDeg; // Absolute value of the Angle in Degrees
    float angleRad;

    // Get the absolute value of angleDeg
    absangleDeg = abs ( angleDeg );

    // Shift rotation to a full circle of 360 deg -> angleDeg // 360
    if ( angleDeg < 0.0 )
  {
        angleDeg = 360.0 - ( absangleDeg - 360.0 *  ( ( int ) ( absangleDeg / 360.0 ) ) ); // Negative values
}    

    else
{
        angleDeg = absangleDeg - 360.0 * ( ( int ) ( absangleDeg / 360.0 ) ); // Positive values
  }

    if ( angleDeg < 180.0 )
  {
        angleDeg = angleDeg - 90.0;
        angleRad =  angleDeg * PI / 180.0; // Convert degree to radials
        sinA =  cos ( angleRad );
        cosA = -sin ( angleRad );
  }

    else
{
        angleDeg = angleDeg - 270.0; // Subtract 270 to shift range
        angleRad = angleDeg * PI / 180.0; // Convert degree to radials

        sinA = -cos ( angleRad );
        cosA =  sin ( angleRad );
  }
}    

//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY         - Input Rotation for the gait 
//PosX            - Input position of the feet X 
//PosZ            - Input position of the feet Z 
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyFKPosX         - Output Position X of feet with Rotation 
//BodyFKPosY         - Output Position Y of feet with Rotation 
//BodyFKPosZ         - Output Position Z of feet with Rotation
void BodyFK (float PosX, float PosZ, float PosY, float RotationY, byte BodyIKLeg)
{
	float            sinB;                   //Sin buffer for BodyRotX calculations
	float            cosB;                   //Cos buffer for BodyRotX calculations
	float            sinG;                   //Sin buffer for BodyRotZ calculations
	float            cosG;                   //Cos buffer for BodyRotZ calculations
	float            cpr_x;                  //Final X value for centerpoint of rotation
	float            cpr_y;                   //Final Y value for centerpoint of rotation
	float            cpr_z;                   //Final Z value for centerpoint of rotation

  //Calculating totals from center of the body to the feet 
  cpr_x = (short)pgm_read_word(&cOffsetX[BodyIKLeg])+PosX + g_InControlState.BodyRotOffset.x;
  cpr_y = PosY + g_InControlState.BodyRotOffset.y;         //Define centerpoint for rotation along the Y-axis
  cpr_z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + g_InControlState.BodyRotOffset.z;

  //Successive global rotation matrix: 
  //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
  //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 

  //First calculate sinus and cosinus for each rotation: 
  GetSinCos ((g_InControlState.BodyRot1.x+TotalXBal1)/10.0);
  sinG = sinA;
  cosG = cosA;

  GetSinCos ((g_InControlState.BodyRot1.z+TotalZBal1)/10.0); 
  sinB = sinA;
  cosB = cosA;

  GetSinCos ((g_InControlState.BodyRot1.y+(RotationY*c1DEC)+TotalYBal1)/10.0) ;

  //Calcualtion of rotation matrix: 
   // Calculation of rotation matrix:
   BodyFKPosX = cpr_x - ( cpr_x * cosA * cosB - cpr_z * cosB * sinA + cpr_y * sinB );
   BodyFKPosZ = cpr_z - ( cpr_x * cosG * sinA + cpr_x * cosA * sinB * sinG + cpr_z * cosA * cosG - cpr_z * sinA * sinB * sinG - cpr_y * cosB * sinG );
   BodyFKPosY = cpr_y - ( cpr_x * sinA * sinG - cpr_x * cosA * cosG * sinB + cpr_z * cosA * sinG + cpr_z * cosG * sinA * sinB + cpr_y * cosB * cosG );
}  



//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//feetPosX            - Input position of the Feet X
//feetPosY            - Input position of the Feet Y
//feetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle           - Output Angle of Femur in degrees
//TibiaAngle           - Output Angle of Tibia in degrees
//CoxaAngle            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK (float feetPosX, float feetPosY, float feetPosZ, int legIndex)
{
    float            IKSW2;                 //Length between Shoulder and Wrist, decimals = 2
    float            IKA14;                  //Angle of the line S>W with respect to the ground in radians, decimals = 4
    float            IKA24;                  //Angle of the line S>W with respect to the femur in radians, decimals = 4
    float            feetPosXZ;              //Diagonal direction from Input X and Z
#ifdef c4DOF
  // these were shorts...
	float            TarsOffsetXZ;           //Vector value \ ;
	float            TarsOffsetY;            //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
	float            TarsToGroundAngle = 0;  //Angle between tars and ground. Note: the angle are 0 when the tars are perpendicular to the ground
	float            TGA_A_H4;
	float            TGA_B_H3;
#else
#define TarsOffsetXZ 0      // Vector value
#define TarsOffsetY  0      //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
#endif
    //Calculate IKCoxaAngle and feetPosXZ
    CoxaAngle[legIndex] = atan2( feetPosZ, feetPosX ) * 180.0 / PI + cCoxaAngle[legIndex];

  //Length between the Coxa and tars [foot]
    feetPosXZ = sqrt ( pow (feetPosX, 2 ) + pow (feetPosZ, 2 ) );
#ifdef c4DOF
  // Some legs may have the 4th DOF and some may not, so handle this here...
    //Calc the TarsToGroundAngle:
    if (cTarsLength[legIndex]) {    // We allow mix of 3 and 4 DOF legs...
        TarsToGroundAngle = -cTarsConst + cTarsMulti*feetPosY + (feetPosXZ*cTarsFactorA) - (feetPosXZ*feetPosY)/cTarsFactorB;
#ifdef DEBUG
        if (g_fDebugOutput && g_InControlState.fRobotOn) {
            DBGSerial.print(" @");
            DBGSerial.print(TarsToGroundAngle, 4);
        }   
#endif 
        if (feetPosY < 0)                   //Always compensate TarsToGroundAngle when feetPosY it goes below zero
            TarsToGroundAngle = TarsToGroundAngle - (feetPosY*cTarsFactorC);     //TGA base, overall rule
        if (TarsToGroundAngle > 40)
            TGA_B_H3 = 20 + (TarsToGroundAngle/2);
		else
            TGA_B_H3 = TarsToGroundAngle;

		if (TarsToGroundAngle > 30)
            TGA_A_H4 = 24 + (TarsToGroundAngle/5);
		else
            TGA_A_H4 = TarsToGroundAngle;

		if (feetPosY > 0)                   //Only compensate the TarsToGroundAngle when it exceed 30 deg (A, H4 PEP note)
            TarsToGroundAngle = TGA_A_H4;
        else if (((feetPosY <= 0) & (feetPosY > -10))) // linear transition between case H3 and H4 (from PEP: H4-K5*(H3-H4))
            TarsToGroundAngle = TGA_A_H4 - (feetPosY*(TGA_B_H3-TGA_A_H4));
		else                                  //feetPosY <= -10, Only compensate TGA1 when it exceed 40 deg
            TarsToGroundAngle = TGA_B_H3;

    //Calc Tars Offsets:
		TarsOffsetXZ = sin(TarsToGroundAngle*PI/180.0) * cTarsLength[legIndex];
		TarsOffsetY = cos(TarsToGroundAngle*PI/180.0) * cTarsLength[legIndex];
  } 
  else {
    TarsOffsetXZ = 0;
    TarsOffsetY = 0;
  }
#ifdef DEBUG
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print(" ");
        DBGSerial.print(TarsOffsetXZ, 4);
        DBGSerial.print(" ");
        DBGSerial.print(TarsOffsetY, 4);
    }    
#endif
#endif

  //Using GetAtan2 for solving IKA1 and IKSW
  //IKA14 - Angle between SW line and the ground in radians
    IKA14 = atan2(feetPosXZ-cCoxaLength[legIndex]-TarsOffsetXZ, feetPosY-TarsOffsetY);

  //IKSW2 - Length between femur axis and tars
    IKSW2 = sqrt ( pow (feetPosXZ-cCoxaLength[legIndex]-TarsOffsetXZ, 2 ) + pow (feetPosY-TarsOffsetY, 2 ) );

  //IKA2 - Angle of the line S>W with respect to the femur in radians
	IKA24 = acos ((pow(cFemurLength[legIndex], 2) - pow(cTibiaLength[legIndex], 2) + pow(IKSW2, 2))
           / (2 * cFemurLength[legIndex] * IKSW2));
#ifdef DEBUG_IK
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print(IKSW2, 4);
        DBGSerial.print(" ");
        DBGSerial.print(IKA14, 4);
        DBGSerial.print(" ");
        DBGSerial.print(IKA24, 4);
    }
#endif
  //IKFemurAngle
    FemurAngle[legIndex] = -(IKA14 + IKA24) * 180.0 / PI + 90.0 + CFEMURHORNOFFSET1(legIndex)/10.0;   //Normal

  //IKTibiaAngle
	IKA24 = acos ((pow(cFemurLength[legIndex], 2) + pow(cTibiaLength[legIndex], 2) - pow(IKSW2, 2)) 
            / (2 * cFemurLength[legIndex] * cTibiaLength[legIndex]));   // rused IKA24
#ifdef DEBUG_IK
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print("=");
        DBGSerial.print(IKA24, 4);
    }
#endif
    
    TibiaAngle[legIndex] = -(90.0 - IKA24*180.0/PI + CTIBIAHORNOFFSET1(legIndex)/10.0);

#ifdef c4DOF
  //Tars angle
    if (cTarsLength[legIndex]) {    // We allow mix of 3 and 4 DOF legs...
        TarsAngle[legIndex] = (TarsToGroundAngle + FemurAngle[legIndex] - TibiaAngle[legIndex])
            + CTARSHORNOFFSET1(legIndex)/10.0;
  }
#endif

  //Set the Solution quality    
    if(IKSW2 < (cFemurLength[legIndex]+cTibiaLength[legIndex]-30))
    IKSolution = 1;
  else
  {
        if(IKSW2 < (cFemurLength[legIndex]+cTibiaLength[legIndex]))
      IKSolutionWarning = 1;
    else
      IKSolutionError = 1    ;
  }
#ifdef DEBUG
    if (g_fDebugOutput && (g_InControlState.fRobotOn || g_InControlState.fPrev_RobotOn)) {
        DBGSerial.print("(");
        DBGSerial.print(feetPosX, 4);
        DBGSerial.print(",");
        DBGSerial.print(feetPosY, 4);
        DBGSerial.print(",");
        DBGSerial.print(feetPosZ, 4);
        DBGSerial.print(")=<");
        DBGSerial.print(CoxaAngle[legIndex], 2);
        DBGSerial.print(",");
        DBGSerial.print(FemurAngle[legIndex], 2);
        DBGSerial.print(",");
        DBGSerial.print(TibiaAngle[legIndex], 2 );
#ifdef c4DOF
        DBGSerial.print(",");
        DBGSerial.print(TarsAngle[legIndex], 2);
#endif
        DBGSerial.print(">");
        DBGSerial.print((IKSolutionError<<2)+(IKSolutionWarning<<1)+IKSolution, DEC);
        if (legIndex == (CNT_LEGS-1))
            DBGSerial.println();
    }
#endif  
}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
float CheckServoAngleBounds(short sID,  float sVal, float sMin, float sMax) {

    // Pull into simple function as so I can report errors on debug 
    // Note ID is bogus, but something to let me know which one.
    if (sVal < sMin) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print(sID, 4);
        DBGSerial.print(" ");
        DBGSerial.print(sVal, 4);
        DBGSerial.print("<");
        DBGSerial.println(sMin, 4);
      }
#endif
        return sMin;
    }

    if (sVal > sMax) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print(sID, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(sVal, DEC);
        DBGSerial.print(">");
        DBGSerial.println(sMax, DEC);
      }
#endif
        return sMax;
    }
    return sVal;
  
}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void)
{
#ifndef SERVOS_DO_MINMAX
  short s = 0;      // BUGBUG just some index so we can get a hint who errored out
  for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++)
  {
        CoxaAngle[LegIndex]  = CheckServoAngleBounds(s++, CoxaAngle[LegIndex], cCoxaMin[LegIndex], cCoxaMax[LegIndex]);
        FemurAngle[LegIndex] = CheckServoAngleBounds(s++, FemurAngle[LegIndex], cFemurMin[LegIndex], cFemurMax[LegIndex]);
        TibiaAngle[LegIndex] = CheckServoAngleBounds(s++, TibiaAngle[LegIndex], cTibiaMin[LegIndex], cTibiaMax[LegIndex]);
#ifdef c4DOF
        if ((byte)(cTarsLength[LegIndex])) {    // We allow mix of 3 and 4 DOF legs...
            TarsAngle[LegIndex] = CheckServoAngleBounds(s++, TarsAngle[LegIndex], cTarsMin[LegIndex], cTarsMax[LegIndex]);
    }
#endif
  }
#endif  
}


//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body 
//            rotation and translation much smoother 
//--------------------------------------------------------------------
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider)
{

  if (CtrlMoveOut < (CtrlMoveInp - 4))
    return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
  else if (CtrlMoveOut > (CtrlMoveInp + 4))
    return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);

  return CtrlMoveInp;
}


//--------------------------------------------------------------------
// GetLegsXZLength - 
//--------------------------------------------------------------------
word g_wLegsXZLength = 0xffff;
word GetLegsXZLength(void) 
{
    // Could save away or could do a little math on one leg... 
    if (g_wLegsXZLength != 0xffff)
        return g_wLegsXZLength;
        
    return sqrt ( pow(LegPosX[0], 2) + pow (LegPosZ[0], 2) );
}


//--------------------------------------------------------------------
// AdjustLegPositions() - Will adjust the init leg positions to the
//      width passed in.
//--------------------------------------------------------------------
#ifndef MIN_XZ_LEG_ADJUST 
#define MIN_XZ_LEG_ADJUST (cCoxaLength[0])      // don't go inside coxa...
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST   (cCoxaLength[0]+cTibiaLength[0] + cFemurLength[0]/4) 
#endif

void AdjustLegPositions(float XZLength) 
{
    //now lets see what happens when we change the leg positions...
    if (XZLength > MAX_XZ_LEG_ADJUST)
        XZLength = MAX_XZ_LEG_ADJUST;
    if (XZLength < MIN_XZ_LEG_ADJUST)
        XZLength = MIN_XZ_LEG_ADJUST;
        
    // see if same length as when we came in
    if (XZLength == g_wLegsXZLength)
        return;

    g_wLegsXZLength = XZLength;
    
        
    for (uint8_t LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print("(");
        DBGSerial.print(LegPosX[LegIndex], DEC);
        DBGSerial.print(",");
        DBGSerial.print(LegPosZ[LegIndex], DEC);
        DBGSerial.print(")->");
      }
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
      GetSinCos(g_InControlState.aCoxaInitAngle[LegIndex]);
#else
#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
      GetSinCos(cCoxaInitAngle[LegIndex]);
#else
      GetSinCos((short)pgm_read_word(&cCoxaAngle[LegIndex]));
#endif
#endif      
      LegPosX[LegIndex] = cosA * XZLength;  //Set start positions for each leg
      LegPosZ[LegIndex] = -sinA * XZLength;
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print("(");
	DBGSerial.print(LegPosX[LegIndex], 4);
        DBGSerial.print(",");
	DBGSerial.print(LegPosZ[LegIndex], 4);
        DBGSerial.print(") ");
      }
#endif
    }
#ifdef DEBUG
    if (g_fDebugOutput) {
      DBGSerial.println("");
    }
#endif
    // Make sure we cycle through one gait to have the legs all move into their new locations...
    g_InControlState.ForceGaitStepCnt = g_InControlState.gaitCur.StepsInGait;
}

//--------------------------------------------------------------------
// ResetLegInitAngles - This is used when we allow the user to 
// adjust the leg position angles.  This resets to what it was when the
// the program was started.
//--------------------------------------------------------------------
void ResetLegInitAngles(void)
{
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    for (int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
            g_InControlState.aCoxaInitAngle[LegIndex] = (short)(cCoxaInitAngle[LegIndex]);
#else
            g_InControlState.aCoxaInitAngle[LegIndex] = (short)(cCoxaAngle[LegIndex]);
#endif
    }
    g_wLegsXZLength = 0xffff;
#endif      
}

//--------------------------------------------------------------------
// ResetLegInitAngles - This is used when we allow the user to 
//--------------------------------------------------------------------
void RotateLegInitAngles (int iDeltaAngle)
{
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    for (int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
        // We will use the cCoxaAngle array to know which direction the legs logically are
        // If the initial angle is 0 don't mess with.  Hex middle legs...
        if ((short)(cCoxaAngle[LegIndex]) > 0) 
            g_InControlState.aCoxaInitAngle[LegIndex] += iDeltaAngle;
         else if ((short)(cCoxaAngle[LegIndex]) < 0)
            g_InControlState.aCoxaInitAngle[LegIndex] -= iDeltaAngle;
        
        // Make sure we don't exceed some min/max angles.
        // Right now hard coded to +-70 degrees... Should probably load
        if (g_InControlState.aCoxaInitAngle[LegIndex] > 700)
            g_InControlState.aCoxaInitAngle[LegIndex] = 700;
        else if (g_InControlState.aCoxaInitAngle[LegIndex] < -700)
            g_InControlState.aCoxaInitAngle[LegIndex] = -700;
    }
    g_wLegsXZLength = 0xffff;
#endif
}


//--------------------------------------------------------------------
// AdjustLegPositionsToBodyHeight() - Will try to adjust the position of the legs
//     to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

uint8_t g_iLegInitIndex = 0x00;    // remember which index we are currently using...

void AdjustLegPositionsToBodyHeight()
{
#ifdef CNT_HEX_INITS
  // Lets see which of our units we should use...
  // Note: We will also limit our body height here...
  if (g_InControlState.BodyPos.y > (short)(g_abHexMaxBodyY[CNT_HEX_INITS-1]))
    g_InControlState.BodyPos.y =  (short)(g_abHexMaxBodyY[CNT_HEX_INITS-1]);

  uint8_t i;
  word XZLength1 = (g_abHexIntXZ[CNT_HEX_INITS-1]);
  for(i = 0; i < (CNT_HEX_INITS-1); i++) {    // Don't need to look at last entry as we already init to assume this one...
    if (g_InControlState.BodyPos.y <= (short)(g_abHexMaxBodyY[i])) {
      XZLength1 = (g_abHexIntXZ[i]);
      break;
    }
  }
  if (i != g_iLegInitIndex) { 
    g_iLegInitIndex = i;  // remember the current index...
    
    // Call off to helper function to do the work.
#ifdef DEBUG
    if (g_fDebugOutput) {
        DBGSerial.print("ALPTBH: ");
        DBGSerial.print(g_InControlState.BodyPos.y, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(XZLength1, DEC);
    }
#endif    
    AdjustLegPositions(XZLength1);
  }
#endif // CNT_HEX_INITS

}

// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifndef __MK20DX256__
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop
#else
// The tone command does sort of work, but does not play multiple sounds smoothly
//  tone(SOUND_PIN, frequency, duration);  // Try the arduino library
//  delay(duration);
  // Try to get something working on DUE...
  long toggle_count = 0;
  long lusDelayPerHalfCycle;
  boolean fHigh = false;
  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, LOW);
  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    fHigh  = !fHigh;
    digitalWrite(SOUND_PIN, fHigh? LOW : HIGH);
    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  digitalWrite(SOUND_PIN, LOW);

#endif
}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif

#ifdef OPT_TERMINAL_MONITOR
#ifdef OPT_DUMP_EEPROM
extern void DumpEEPROMCmd(byte *pszCmdLine);
#endif
#ifdef QUADMODE
extern void UpdateGaitCmd(byte *pszCmdLine);
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
extern void UpdateInitialPosAndAngCmd(byte *pszCmdLine);
#endif

//==============================================================================
// TerminalMonitor - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
boolean TerminalMonitor(void)
{
  byte szCmdLine[20];  // currently pretty simple command lines...
  byte ich;
  int ch;
  // See if we need to output a prompt.
  if (g_fShowDebugPrompt) {
    DBGSerial.println(F("Arduino Phoenix Monitor"));
    DBGSerial.println(F("D - Toggle debug on or off"));
#ifdef OPT_DUMP_EEPROM
    DBGSerial.println(F("E - Dump EEPROM"));
#endif
#ifdef QUADMODE
//	DBGSerial.println(F("B <percent>"));
    DBGSerial.println(F("G ST NL RR RF LR LF"));
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    DBGSerial.println(F("I pos ang"));
#endif
#ifdef OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
    g_InputController.ShowTerminalCommandList(); 
#endif      

    // Let the Servo driver show it's own set of commands...
    g_ServoDriver.ShowTerminalCommandList();
    g_fShowDebugPrompt = false;
  }

  // First check to see if there is any characters to process.
  if ((ich = DBGSerial.available())) {
    ich = 0;
    // For now assume we receive a packet of data from serial monitor, as the user has
    // to click the send button...
    for (ich=0; ich < sizeof(szCmdLine); ich++) {
      ch = DBGSerial.read();        // get the next character
      if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
        break;
      szCmdLine[ich] = ch;
    }
    szCmdLine[ich] = '\0';    // go ahead and null terminate it...
    
    // Remove any extra EOL characters that may have been added
    for (;;) {
      ch = DBGSerial.peek();
      if ((ch >= 10) && (ch <= 15))
        DBGSerial.read();
      else
        break;
    }
    if (ich) {
      DBGSerial.print(F("Serial Cmd Line:"));        
      DBGSerial.write(szCmdLine, ich);
      DBGSerial.println(F("<eol>"));
    }
    // So see what are command is.
    if (!ich)  {
      g_fShowDebugPrompt = true;
    } 
    else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
      g_fDebugOutput = !g_fDebugOutput;
      if (g_fDebugOutput) 
        DBGSerial.println(F("Debug is on"));
      else
        DBGSerial.println(F("Debug is off"));
    } 
#ifdef OPT_DUMP_EEPROM
    else if (((szCmdLine[0] == 'e') || (szCmdLine[0] == 'E'))) {
      DumpEEPROMCmd(szCmdLine);
    } 
#endif
#ifdef QUADMODE
    else if (((szCmdLine[0] == 'g') || (szCmdLine[0] == 'G'))) {
      UpdateGaitCmd(szCmdLine);
    } 
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    else if (((szCmdLine[0] == 'i') || (szCmdLine[0] == 'I'))) {
      UpdateInitialPosAndAngCmd(szCmdLine);
    } 
#endif
#ifdef OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
    else if (g_InputController.ProcessTerminalCommand(szCmdLine, ich)) 
      ;  // See if the Input controller has added commands...
#endif      

    else
    {
      g_ServoDriver.ProcessTerminalCommand(szCmdLine, ich);
    }

    return true;
  }
  return false;
}


//--------------------------------------------------------------------
// DumpEEPROM
//--------------------------------------------------------------------
#ifdef OPT_DUMP_EEPROM
byte g_bEEPromDumpMode = 0;  // assume mode 0 - hex dump
word g_wEEPromDumpStart = 0;  // where to start dumps from
byte g_bEEPromDumpCnt = 16;  // how much to dump at a time

void DumpEEPROM() {
  byte i;
  word wDumpCnt = g_bEEPromDumpCnt;

  while (wDumpCnt) {
    DBGSerial.print(g_wEEPromDumpStart, HEX);
    DBGSerial.print(" - ");

    // First in Hex
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      byte b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      DBGSerial.print(b, HEX);
      DBGSerial.print(" ");
    }
    // Next in Ascii
    DBGSerial.print(" : ");
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      byte b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      if ((b > 0x1f) && (b < 0x7f))
        DBGSerial.write(b);
      else
        DBGSerial.print(".");
    }
    DBGSerial.println("");
    g_wEEPromDumpStart += i;  // how many bytes we output
    wDumpCnt -= i;            // How many more to go...
  } 

}
#endif

//--------------------------------------------------------------------
// GetCmdLineNum - passed pointer to pointer so we can update...
//--------------------------------------------------------------------
long GetCmdLineNum(byte **ppszCmdLine) {
  byte *psz = *ppszCmdLine;
  long iVal = 0;
  int iSign = 1;

  // Ignore any blanks
  while (*psz == ' ')
    psz++;

  // See if Hex value passed in
  if ((*psz == '0') && ((*(psz+1) == 'x') || (*(psz+1) == 'X'))) {
    // Hex mode
    psz += 2;  // get over 0x
    for (;;) {
      if ((*psz >= '0') && (*psz <= '9'))
        iVal = iVal * 16 + *psz++ - '0';
      else if ((*psz >= 'a') && (*psz <= 'f'))
        iVal = iVal * 16 + *psz++ - 'a' + 10;
      else if ((*psz >= 'A') && (*psz <= 'F'))
        iVal = iVal * 16 + *psz++ - 'A' + 10;
      else
        break;
    }

  }
  else {
    // decimal mode
    if (*psz == '-') {
        iSign = -1;
        psz++;
    }    
        
    while ((*psz >= '0') && (*psz <= '9'))
      iVal = iVal * 10 + *psz++ - '0';
  }
  *ppszCmdLine = psz;    // update command line pointer
  return iSign * iVal;

}

#ifdef OPT_DUMP_EEPROM
//--------------------------------------------------------------------
// DumpEEPROMCmd
//--------------------------------------------------------------------
void DumpEEPROMCmd(byte *pszCmdLine) {
  // first byte can be H for hex or W for words...
  if (!*++pszCmdLine)  // Need to get past the command letter first...
    DumpEEPROM();
  else if ((*pszCmdLine == 'h') || (*pszCmdLine == 'H')) 
    g_bEEPromDumpMode = 0;
  else if ((*pszCmdLine == 'w') || (*pszCmdLine == 'W')) 
    g_bEEPromDumpMode = 0;

  else {
    // First argument should be the start location to dump
    g_wEEPromDumpStart = GetCmdLineNum(&pszCmdLine);

    // If the next byte is an "=" may try to do updates...
    if (*pszCmdLine == '=') {
      // make sure we don't get stuck in a loop...
      byte *psz = pszCmdLine;
      word w;
      while (*psz) {
        w = GetCmdLineNum(&psz);
        if (psz == pszCmdLine)
          break;  // not valid stuff so bail!
        pszCmdLine = psz;  // remember how far we got...

        EEPROM.write(g_wEEPromDumpStart++, w & 0xff);
      }
    }
    else {
      if (*pszCmdLine == ' ') { // A blank assume we have a count...
        g_bEEPromDumpCnt = GetCmdLineNum(&pszCmdLine);
      }
    }
    DumpEEPROM();
  }
}
#endif

#ifdef QUADMODE
//--------------------------------------------------------------------
// UpdateGaitCmd
//--------------------------------------------------------------------
void UpdateGaitCmd(byte *pszCmdLine) {
  // If no other parameters, show current state
  if (!*++pszCmdLine) {  // Need to get past the command letter first...
    DBGSerial.print("St: ");
    DBGSerial.print(g_InControlState.gaitCur.StepsInGait, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.NrLiftedPos, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[cRR], DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[cRF], DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[cLR], DEC);
    DBGSerial.print(" ");
    DBGSerial.println(g_InControlState.gaitCur.GaitLegNr[cLF], DEC);
  }
  else {
    //Argument should be New percentage
    word wStepsInGait = GetCmdLineNum(&pszCmdLine);
    word wLifted = GetCmdLineNum(&pszCmdLine);
    
    // first pass only pass in number of steps and maybe Lifted pos
    if (wStepsInGait) {
        if (wLifted) {
            // UPdated the lifted so lets update some of the gait properties
            g_InControlState.gaitCur.NrLiftedPos = wLifted;
            g_InControlState.gaitCur.FrontDownPos = (wLifted+1)/2;
            g_InControlState.gaitCur.LiftDivFactor = (wLifted > 4)? 4 : 2;
        }

        // Assume the ordering of the gait legs here and equal spaced
        g_InControlState.gaitCur.StepsInGait = wStepsInGait;
        g_InControlState.gaitCur.TLDivFactor = g_InControlState.gaitCur.StepsInGait-g_InControlState.gaitCur.NrLiftedPos;
            
        // See if user did pass in leg positions...
        g_InControlState.gaitCur.GaitLegNr[cRR] = GetCmdLineNum(&pszCmdLine);
        if (g_InControlState.gaitCur.GaitLegNr[cRR]) {
            g_InControlState.gaitCur.GaitLegNr[cRF] = GetCmdLineNum(&pszCmdLine);
            g_InControlState.gaitCur.GaitLegNr[cLR] = GetCmdLineNum(&pszCmdLine);
            g_InControlState.gaitCur.GaitLegNr[cLF] = GetCmdLineNum(&pszCmdLine);
        }
        else {
            wStepsInGait /= 4;  // equal spacing.
            g_InControlState.gaitCur.GaitLegNr[cRR] = wStepsInGait / 2;
            g_InControlState.gaitCur.GaitLegNr[cRF] = g_InControlState.gaitCur.GaitLegNr[cRR] + wStepsInGait;
            g_InControlState.gaitCur.GaitLegNr[cLR] = g_InControlState.gaitCur.GaitLegNr[cRF] + wStepsInGait;
            g_InControlState.gaitCur.GaitLegNr[cLF] = g_InControlState.gaitCur.GaitLegNr[cLR] + wStepsInGait;
        }
    
        //g_InControlState.gaitCur.HalfLiftHeight = 3;
        //g_InControlState.gaitCur.NomGaitSpeed = DEFAULT_GAIT_SPEED;
    }   
  }
}
#endif //Quad Mode

//--------------------------------------------------------------------
// UpdateGaitCmd
//--------------------------------------------------------------------
#ifdef OPT_DYNAMIC_ADJUST_LEGS
void UpdateInitialPosAndAngCmd(byte *pszCmdLine) {
  // If no other parameters, show current state
  if (!*++pszCmdLine) {  // Need to get past the command letter first...
	DBGSerial.print("Len: ");
	DBGSerial.print(GetLegsXZLength() , DEC);
	DBGSerial.print(" Angs: ");
    for(int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
        DBGSerial.print(g_InControlState.aCoxaInitAngle[LegIndex], DEC);
        DBGSerial.print(" ");
    }
    DBGSerial.println();
  }
  else {
	// Get the new leg positions
    word wNewLegsXZPos = GetCmdLineNum(&pszCmdLine);
    if (*pszCmdLine) {
      int  iDeltaAngle = GetCmdLineNum(&pszCmdLine);
      RotateLegInitAngles(iDeltaAngle);
    }  
    AdjustLegPositions(wNewLegsXZPos);

  }
}
#endif

#endif

















