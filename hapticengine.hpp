//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 
    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================
#define _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_WARNINGS
#define ZERO 1e-10
#define isSame(A, B) ( ((A-B) < ZERO) && ((A-B) > -ZERO) )
#include <iostream>
#include <vector>
#include "simple826.hpp"
#include <unistd.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "forcesensor.hpp"
#include <chrono>  // for high_resolution_clock
#include "chai3d.h"
#include "staircase.hpp"
#include <GLFW/glfw3.h>
#include "Utils.h"
#include "PointATC3DG.h"
#include <usb.h>
#include <fstream>
#include <string>
#include <time.h> 
#include "EposCmd.hpp"
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
//typedef Eigen::Matrix<float, 7, 1> vector7f;
//typedef Eigen::Matrix<float, 8, 1> vector8f;
void runEpos(void *epos_position);

void readFTdata(void *shared_data);
ForceSensor Force;

//---------------------------------------------------------------------
//Position Sensor
PointATC3DG bird;


double dX, dY, dZ, dAzimuth, dElevation, dRoll; // Tracker data for the bird
double baseX, baseY, baseZ, baseAzimuth, baseElevation, baseRoll; // Tracker data for the falcon base
cVector3d birdLocalPosition; // Position of bird in the local (rendered coordinate)

int numsen=bird.getNumberOfSensors();
//----------------------------------------------------------------------
// EPOS

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;
volatile bool exitKey = false;
cVector3d position;
// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursor;

// a line representing the velocity vector of the haptic device
cShapeLine* velocity;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;
cThread* forceThread;
cThread* EposThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
//int swapInterval = 1;
int rec_count=0;     
//double Kp = 202.0; // [N/m]
double stiffness = 2000.0; // [N/m]


int case_num =0;
double adjust[22];
double Stiffness(double k);
//double K[]={269.1499, 490.81957,697.33,889.5498, 1068.3, 1234.55, 1389.062,1532.7,    1666.4,    1790.978,    1907.287,    2016.2,    2118.6,  2215.3,    2307.2,    2395.18,    2480.07,   2562.74,    2644.055,    2724.877,    2806.07,    2888.495,    2973.02,3060.49,3151.79,3247.775,3349.3,3457.24,3572.4};
//ouble first=0;

//double min_delta_stiff;
//double max_delta_stiff;
//double max_stiffness;
double min_stiffness=100.0;
double max_stiffness=1800.0;
//double min_delta_stiff=100.0;//k[2200],k[2100]
//double max_delta_stiff=max_stiffness-min_stiffness;//k[2200],k[200]
//double step_size=(max_delta_stiff-min_delta_stiff)/16;
double step_size=100;
//double initial_step_size=(max_delta_stiff-min_delta_stiff)/8;
//double min_step_size =10.0;
int trial_numbers=100; // After how many trials we want pest to exit.
Staircase staircase(min_stiffness, max_stiffness, step_size);
double first_stimulus=max_stiffness;//2200,200
//double init_stimulus=min_stiffness;
double second_stimulus=min_stiffness;
int bool_random=0;
double new_stimulus =max_stiffness;

// a few shape primitives that compose our scene

// A sphere for magnatic tracker
cShapeSphere* birdSphere;

// four different boxes
cShapeBox* box0;
cShapeBox* box1;
cShapeBox* box2;
cShapeBox* box3;

// Meter to Inch Conversion meter = inch / 39.37
const double InchPerMeter = 39.37;
const cVector3d OriginCorrection = cVector3d(- 0.27, 0.025, 0.32);
cVector3d CalibrationCorrection = cVector3d(0, 0, 0.16);
bool bIsCalibrationMode = true;
int currentPlateNumber = -1;

// Variable list for stiffness and face of different blocks
int stiffnessList[4] = {2000, 2000, 2000, 2000};


int faceList[4] = {1, 1, 0, 0};
int roundComplete = 0;
int recordHeight = 0;
// Boxes for different stiffness

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void* shared_data);

// this function closes the application
void close(void);

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 60, 0);
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    cMatrix3d rotationMatrixFalconStanding( 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0 );
    std::string currentPlateString = "Not On Plate     Stiffness: ";
    if (currentPlateNumber >= 0){
        if (currentPlateNumber == 0) {
            currentPlateString = " Box 0     Stiffness: ";
        }
        else if (currentPlateNumber == 1) {
            currentPlateString = " Box 1     Stiffness: ";
        }
        else if (currentPlateNumber == 2) {
            currentPlateString = " Box 2     Stiffness: ";
        }
        else if (currentPlateNumber == 3) {
            currentPlateString = " Box 3     Stiffness: ";
        }
    }
    if (bIsCalibrationMode){
        labelHapticDevicePosition->setText((rotationMatrixFalconStanding * hapticDevicePosition).str(3) + "        Press V To Exit Calibration Mode");
    }
    else{
        labelHapticDevicePosition->setText((rotationMatrixFalconStanding * hapticDevicePosition).str(3) + " Press C To Enter Calibration Mode    " + currentPlateString +  std::to_string(stiffness));
    }
    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    // bird local posion updated in haptic thread
    birdSphere->setLocalPos(birdLocalPosition);

    
    if (currentPlateNumber >= 0){
        // 0.05 is the base height and 0.01 is the offset
        double height = hapticDevicePosition.x() + 0.051;
        if (height > 0.05){
            height = 0.05;
        }
        else if (height < 0.01){
            height = 0.01;
        }

        if (currentPlateNumber >= 0) {
            box0->setSize(0.05, 0.05, 0.05);
            box1->setSize(0.05, 0.05, 0.05);
            box2->setSize(0.05, 0.05, 0.05);
            box3->setSize(0.05, 0.05, 0.05);
        }

        // In one of the regions
        if (currentPlateNumber == 0) {
            box0->setSize(0.05, 0.05, height);
        }
        else if (currentPlateNumber == 1) {
            box1->setSize(0.05, 0.05, height);
        }
        else if (currentPlateNumber == 2) {
            box2->setSize(0.05, 0.05, height);
        }
        else if (currentPlateNumber == 3 && box3) {
            box3->setSize(0.05, 0.05, height);
        }
    }
    

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{	
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // Zhian Li: Set Calibration
    else if (a_key == GLFW_KEY_C)
    {
       bIsCalibrationMode = true;
       roundComplete = 0;
    }

    else if (a_key == GLFW_KEY_V)
    {
       bIsCalibrationMode = false;
       roundComplete = 0;
    }

    else if (a_key == GLFW_KEY_1)
    {
        if (!bIsCalibrationMode){
            std::cout << "user selected block 0" << std::endl;
        }
        roundComplete = -1;
    }

    else if (a_key == GLFW_KEY_2)
    {
        if (!bIsCalibrationMode){
            std::cout << "user selected block 1" << std::endl;
        }
        roundComplete = -1;
    }

    else if (a_key == GLFW_KEY_3)
    {
        if (!bIsCalibrationMode){
            std::cout << "user selected block 2" << std::endl;
        }
        roundComplete = -1;
    }

    else if (a_key == GLFW_KEY_4)
    {
        if (!bIsCalibrationMode){
            std::cout << "user selected block 3" << std::endl;
        }
        roundComplete = 1;
    }

    else if (a_key == GLFW_KEY_R)
    {
       recordHeight = 1;
    }

}

//------------------------------------------------------------------------------
double horizontalSquareDistanceBetween(cVector3d v1, cVector3d v2)
{
    // y-z plane is the horizontal plane
    return (v1.x() - v2.x()) * (v1.x() - v2.x()) + (v1.y() - v2.y()) * (v1.y() - v2.y());
}

void updateHaptics(void* shared_data)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;
    double prev_stiffness=0.0;
    cVector3d forceField (0.0,0.0,0.0);
    cVector3d gravityCorrection (3.5, 0.0, -2.5);
    double add=0;
    // main haptic simulation loop
    int loopcount=0;
    cVector3d lastDesiredPosition(0.01, 0.0, 0.0);
    // Placeholder for stiffness and face each round


    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        hapticDevice->getPosition(position);
        
        // read orientation 
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read linear velocity 
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
        
        // Convert Axis
        cMatrix3d rotationMatrixFalconStanding( 0.0, 0.0, -1.0, 
                                                0.0, 1.0, 0.0, 
                                                1.0, 0.0, 0.0 ); 
        // update arrow
        velocity->m_pointA = rotationMatrixFalconStanding * position;
        velocity->m_pointB = cAdd(rotationMatrixFalconStanding * position, rotationMatrixFalconStanding * linearVelocity);

        // update position and orientation of cursor
        
        cursor->setLocalPos(rotationMatrixFalconStanding * position);
        cursor->setLocalRot(rotation);

        // adjust the  color of the cursor according to the status of
        // the user-switch (ON = TRUE / OFF = FALSE)
        if (*(double *)shared_data > 10.0)
        {
            cursor->m_material->setGreenMediumAquamarine();
        }

        // Add offset and convert axis
        cVector3d birdPositionMeter((-dZ / InchPerMeter) + OriginCorrection.x(),
                                (-dY / InchPerMeter) + OriginCorrection.y(),
                                (-dX / InchPerMeter) + OriginCorrection.z());

        birdPositionMeter = rotationMatrixFalconStanding * birdPositionMeter;

        // Add offset and convert axis
        cVector3d basePositionMeter((-baseZ / InchPerMeter) + OriginCorrection.x(),
                                (-baseY / InchPerMeter) + OriginCorrection.y(),
                                (-baseX / InchPerMeter) + OriginCorrection.z());

        basePositionMeter = rotationMatrixFalconStanding * basePositionMeter;

        birdLocalPosition = birdPositionMeter - basePositionMeter;
        birdLocalPosition -= CalibrationCorrection;
        hapticDevicePosition = position;

        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // variables for forces
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;
        
        cVector3d desiredPosition(0.01, 0.0, 0.0);
        cVector3d defaultPosition(0.01, 0.0, 0.0);

        // Calibration Mode: Force desired position to be (0, 0, 0)
        if (bIsCalibrationMode){
            // Set Falcon to Origin
            stiffness = 1000;
            CalibrationCorrection += birdLocalPosition;

            loopcount=0;

            cVector3d displacement = (desiredPosition - position);

            double corrected_stiffness = Stiffness(stiffness);
            forceField = cVector3d(corrected_stiffness * displacement.x(), corrected_stiffness * displacement.y() * 0.1, corrected_stiffness * displacement.z() * 0.1);

            force.add(gravityCorrection);
            force.add(forceField);

            hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

            freqCounterHaptics.signal(1);
            continue;
        }

        // box coordinate in axis of haptic device
        cVector3d box0Pos(0.01, 0.02, -0.02);
        cVector3d box1Pos(0.01, 0.02, 0.02);
        cVector3d box2Pos(0.01, -0.02, 0.02);
        cVector3d box3Pos(0.01, -0.02, -0.02);

        // the default desired position is the origin

        // If tracker is above a platform, we think the tracker is in workspace 
        // and start moving the haptic device 
        
        if (abs(birdLocalPosition.x()) < 0.1 && abs(birdLocalPosition.y()) < 0.1) // 
        {
            // box 0
            if (birdLocalPosition.x() > 0.01 && birdLocalPosition.y() > 0.01){
                desiredPosition = box0Pos;
                currentPlateNumber = 0;
            }
            // box 1
            else if (birdLocalPosition.x() < -0.01 && birdLocalPosition.y() > 0.01){
                desiredPosition = box1Pos;
                currentPlateNumber = 1;
            }
            // box 2
            else if (birdLocalPosition.x() < -0.01 && birdLocalPosition.y() < -0.01){
                desiredPosition = box2Pos;
                currentPlateNumber = 2;
            }
            // box 3
            else if (birdLocalPosition.x() > 0.01 && birdLocalPosition.y() < -0.01){
                desiredPosition = box3Pos;
                currentPlateNumber = 3;
            }
            // Not on box
            else {
                desiredPosition = lastDesiredPosition;
                currentPlateNumber = -1;
            }
        }
        // Not on box
        else {
            desiredPosition = lastDesiredPosition;
            currentPlateNumber = -1;
        }
        
        
        if (currentPlateNumber >= 0){
            stiffness = Stiffness((double) stiffnessList[currentPlateNumber]);
            TargetFace = faceList[currentPlateNumber];
        }

        lastDesiredPosition = desiredPosition;

        // desired orientation
        cMatrix3d desiredRotation;
        desiredRotation.identity();
        
        loopcount=0;

        cVector3d displacement = (desiredPosition - position);

        // Limit force
        double corrected_stiffness = Stiffness(stiffness);
        forceField = cVector3d(corrected_stiffness * displacement.x(), corrected_stiffness * displacement.y() * 0.02, corrected_stiffness * displacement.z() * 0.2);
        
        if (forceField.y() > 0.2){
            forceField.y(2);
        }
        if (forceField.y() < -0.2){
            forceField.y(-2);
        }
        if (forceField.z() > 0.2){
            forceField.z(2);
        }
        if (forceField.z() < -0.2){
            forceField.z(-2);
        }
        
        force.add(gravityCorrection);
        force.add(forceField);
            
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

void readFTdata(void *shared_data)
{
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    //snprintf(buffer, sizeof(buffer), "file_%d.txt", 10);
    strftime (buffer,80,"%Y-%m-%d-%H-%M-%S.csv",now);
    std::ofstream myfile;
    myfile.open (buffer);
    int sennum=0;
    //std::cout<<Kp<<std::endl;
    //if( !bird ) return -1;
    //bird.setSuddenOutputChangeLock( 0 );
    //std::cout << "nSensors: " << numsen << std::endl;
    //char outFileString[13] = "testing.csv";
    //FILE *outFilePtr;
    //outFilePtr = fopen(outFileString, "w+");
    bird.setSuddenOutputChangeLock( 0 );
    std::cout << "nSensors: " << numsen << std::endl;
    std::cout << "Here!!!" << std::endl;
    int num=1;
    Vector6FT FT_data;
    int numSample =4;//Number of samples needed to get average in buffer
    int frequency =1000;
    Force.FTSetOffset(1000);//Get the offset from the first 1000 data
    
    //int rec_count=0;     
    //time_t ta=time(NULL);
    //time_t tb=time(NULL);

    auto t0 = std::chrono::high_resolution_clock::now();
    int output_count = 0;
	while (!exitKey) {
        bird.getCoordinatesAngles( 0, dX, dY, dZ, dAzimuth, dElevation, dRoll );
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t0 ).count();
        FT_data = Force.GetCurrentFT(numSample) ;
        *(Vector6FT*)(shared_data) = FT_data;
        // myfile <<FT_data[0]<<" "<< FT_data[1]<< " "<<FT_data[2]<< " "<<FT_data[3]<<" "<<FT_data[4]<<" "<< FT_data[5]<< " "<<position(0)<< " "<< position(1)<< " "<< position(2)<< " "<<stiffness<<" "<<dX<< " "<<dY<< " "<<dZ<< " "<<dAzimuth<< " "<<dElevation<< " "<<dRoll<< " "<<duration<<" "<<" "<<first_stimulus<<" "<<second_stimulus<<"\n";
        //myfile2 <<FT_data[0]<<" "<< FT_data[1]<< " "<<FT_data[2]<< " "<<FT_data[3]<<" "<<FT_data[4]<<" "<< FT_data[5]<< " "<<position(0)<< " "<< position(1)<< " "<< position(2)<< " "<<Kp<<" "<<dX<< " "<<dY<< " "<<dZ<< " "<<dAzimuth<< " "<<dElevation<< " "<<dRoll<< " "<<duration<<"\n";
        //std::cout << "\rX: " << dX << ", \tY: " << dY << ", \tZ: " << dZ;
        //std::cout << ", \tA: " << dAzimuth << ", \tE: " << dElevation << ", \tR: " << dRoll << std::endl;
        output_count++;
        // double baseX, baseY, baseZ, baseAzimuth, baseElevation, baseRoll;
        bird.getCoordinatesAngles( 1, baseX, baseY, baseZ, baseAzimuth, baseElevation, baseRoll );
        if (output_count % 1000 == 0){
            // std::cout << birdLocalPosition.x() << " " << birdLocalPosition.y() << " "<< birdLocalPosition.z() << " "<<dAzimuth<< " "<<dElevation<< " "<< dRoll << std::endl;
        }

        if (bIsCalibrationMode){
            t0 = std::chrono::high_resolution_clock::now();
        }
        else{
            if (roundComplete){
                bIsCalibrationMode = true;
                roundComplete = 0;
                std::cout << "Time consumed: " << duration / 1000000  << " seconds" << std::endl;
            }
        }

        if (recordHeight){
            std::cout << "NO: " << currentPlateNumber << " Stiffness: " << stiffness << " Tracker: " << (-dZ / InchPerMeter) * 100 << " Encoder: " << position(0) * 100 << std::endl;
            myfile << "NO: " << currentPlateNumber << " Stiffness: " << stiffness << " Tracker: " << (-dZ / InchPerMeter) * 100 << " Encoder: " << position(0) * 100  << std::endl;
            recordHeight = 0;
        }
    }

    /*
    while (!exitKey) {
        //std::cout<<"Success"<<std::endl;
        FT_data = Force.GetCurrentFT(numSample) ;
        *(Vector6FT*)(shared_data) = FT_data;
        myfile <<FT_data[0]<<" "<< FT_data[1]<< " "<<FT_data[2]<< " "<<FT_data[3]<<" "<<FT_data[4]<<" "<< FT_data[5]<< " "<<position(0)<< " "<< position(1)<< " "<< position(2)<< " "<<stiffness<<"\n";
        //fprintf(outFilePtr, "%f, %f, %f, %f, %f, %f, %.4f, %.4f, %.4f, %.4f\n", FT_data[0], FT_data[1], FT_data[2], FT_data[3], FT_data[4], FT_data[5], position(0), position(1), position(2), Kp);
        //std::cout << "\rX: " << dX << ", \tY: " << dY << ", \tZ: " << dZ;
        //std::cout << ", \tA: " << dAzimuth << ", \tE: " << dElevation << ", \tR: " << dRoll << std::endl;
        
    }
    */
     myfile.close();
    /*
    while (!exitKey) {
        rec_count++;
        bird.getCoordinatesAngles( sennum, dX, dY, dZ, dAzimuth, dElevation, dRoll );
        FT_data = Force.GetCurrentFT(numSample) ;
        *(Vector6FT*)(shared_data) = FT_data;
        //fprintf(outFilePtr, "%f, %f, %f, %f, %f, %f, %.4f, %.4f, %.4f\n", FT_data[0], FT_data[1], FT_data[2], FT_data[3], FT_data[4], FT_data[5], position(0), position(1), position(2));
        //std::cout << "\rX: " << dX << ", \tY: " << dY << ", \tZ: " << dZ;
        //std::cout << ", \tA: " << dAzimuth << ", \tE: " << dElevation << ", \tR: " << dRoll << std::endl;
    }
    */
    //tb=time(NULL);
    //std::cout << rec_count << " samples collected" << std::endl;
    //std::cout << tb-ta << " seconds elapsed" << std::endl;
    //std::cout << rec_count/(tb-ta) << " samples per second" << std::endl;
    
}

double Stiffness(double k){
   //double stiffness= -5.27e-06*k*k + 0.9599*k + -6.851;
//   double stiffness= 80.88*k*k + 820.6*k + 1527;
	 double stiffness= -3.4899e-04*k*k + 2.0251*k + (-147.2680);
    
    return stiffness;
}

void runEpos(void *shared_data){
    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;

    PrintHeader();

    SetDefaultParameters();

    PrintSettings();

    if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("OpenDevice", lResult, ulErrorCode);
        //return lResult;
    }

    if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("PrepareDemo", lResult, ulErrorCode);
        //return lResult;
    }

    if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("Demo", lResult, ulErrorCode);
        // return lResult;
    }

    if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("CloseDevice", lResult, ulErrorCode);
        //  return lResult;
    }
}

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete forceThread;
    delete EposThread;
    delete world;
    delete handler;
}




int displaySetup(){
     //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    //glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (10.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, -1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);

    // insert cursor inside world
    world->addChild(cursor);

    // create small line to illustrate the velocity of the haptic device
    velocity = new cShapeLine(cVector3d(0,0,0), 
                              cVector3d(0,0,0));

    // insert line inside world
    world->addChild(velocity);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame
        cursor->setShowFrame(true);

        // set the size of the reference frame
        cursor->setFrameSize(0.05);
    }

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);

    return 0;
}