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
#include "pest.hpp"
#include <GLFW/glfw3.h>
#include "Utils.h"
#include "PointATC3DG.h"
#include <usb.h>
#include <fstream>
#include <string>
#include <time.h> 

using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
//typedef Eigen::Matrix<float, 7, 1> vector7f;
//typedef Eigen::Matrix<float, 8, 1> vector8f;
void readFTdata(void *shared_data);
ForceSensor Force;

//---------------------------------------------------------------------
//Position Sensor
PointATC3DG bird;


double dX, dY, dZ, dAzimuth, dElevation, dRoll;

int numsen=bird.getNumberOfSensors();

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
double stiffness = 200.0; // [N/m]


int case_num =0;
double adjust[22];
double Stiffness(double k);
//double K[]={269.1499, 490.81957,697.33,889.5498, 1068.3, 1234.55, 1389.062,1532.7,    1666.4,    1790.978,    1907.287,    2016.2,    2118.6,  2215.3,    2307.2,    2395.18,    2480.07,   2562.74,    2644.055,    2724.877,    2806.07,    2888.495,    2973.02,3060.49,3151.79,3247.775,3349.3,3457.24,3572.4};
//ouble first=0;

//double min_delta_stiff;
//double max_delta_stiff;
//double max_stiffness;

double max_stiffness=2200.0;
double min_delta_stiff=100.0;//k[2200],k[2100]
double max_delta_stiff=max_stiffness-200.0;//k[2200],k[200]
double initial_step_size=(max_delta_stiff-min_delta_stiff)/8;
double min_step_size =10.0;
int trial_numbers=10; // After how many trials we want pest to exit.
Pest pest(min_delta_stiff, max_delta_stiff, initial_step_size,min_step_size);
double first_stimulus=max_stiffness;//2200,200
double second_stimulus=max_stiffness-max_delta_stiff;
int bool_random=0;
double new_stimulus =max_delta_stiff;
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
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


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
	//rand() % 2+ 1;
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

    /////////////////PEST
    else if (a_key == GLFW_KEY_N)
    {
    	if (bool_random ==0)
    	{
    		stiffness=Stiffness(first_stimulus); //2200   
            std::cout<<"first "<< first_stimulus <<std::endl;
    	}
    	else{
    		stiffness=Stiffness(second_stimulus);   //200 
            std::cout<<"second: "<< second_stimulus<<std::endl;
            //std::cout<<"whaaaaaaaaaaat: "<<stiffness<<endl;
    	}
        
    }
    else if (a_key == GLFW_KEY_M)
    {
    	if (bool_random ==0)
        {
        	stiffness=Stiffness(second_stimulus);   //200 
            std::cout<<"second: "<< second_stimulus<<std::endl;
            //std::cout<<"stiff: "<<stiffness<<endl;
    		
    	}
    	else{
    		stiffness=Stiffness(first_stimulus); //2200   
            std::cout<<"first "<< first_stimulus <<std::endl;
            //std::cout<<"stiff: "<<stiffness<<endl;
    	}
    }
    else if (a_key == GLFW_KEY_J)
    {
    	if ((bool_random ==0)){
    		std::cout<<"correct!"<<std::endl;
    		new_stimulus= pest.NextStimulus(1);
    		second_stimulus = max_stiffness-new_stimulus;//max-new>max
    	    std::cout<<"next stimulus1: "<<new_stimulus<<"  "<<second_stimulus<<std::endl;
    	    //std::cout<<"first "<< first_stimulus <<std::endl;
    	    //std::cout<<"second: "<< second_stimulus<<std::endl;
    	   if (pest.flag==false) 
    	   {
    	   		std::cout<<"step"<<endl;
    	    	exit(-1);
    	    }
    	   // terminate();
    	}
    	else{
    		std::cout<<"not correct!"<<std::endl;
    		new_stimulus= pest.NextStimulus(0);
    		second_stimulus = max_stiffness-new_stimulus;
	        std::cout<<"next stimulus0: "<<new_stimulus<<"  "<<second_stimulus<<std::endl;
	        //std::cout<<"first "<< first_stimulus <<std::endl;
    	    //std::cout<<"second: "<< second_stimulus<<std::endl;
    	    if (pest.flag==false) 
    	    {
    	    	std::cout<<"step"<<endl;
    	        exit(-1);
    	    }
    	    //terminate();
    	}
    	//second_stimulus = Stiffness(max_stiffness-p0);
        //std::cout<<"next: "<<bool_random<<std::endl;
    	bool_random=rand() % 2;
    	
    }
    else if (a_key == GLFW_KEY_K)
    {
    	//bool_random=rand() % 2;    	//second_stimulus = Stiffness(max_stiffness-p1);
    	if ((bool_random ==1)){
    		std::cout<<"then correct!"<<std::endl;
    		new_stimulus= pest.NextStimulus(1);
    		second_stimulus = max_stiffness-new_stimulus;
    	    std::cout<<"next stimulus1: "<<new_stimulus<<"  "<<second_stimulus<<std::endl;
    	    //std::cout<<"first "<< first_stimulus <<std::endl;
    	    //std::cout<<"second: "<< second_stimulus<<std::endl;
    	    if (pest.flag==false) 
    	    {
    	    	std::cout<<"step"<<endl;
    	        exit(-1);
    	    }
    	    //terminate();
    	}
    	else{
    		std::cout<<"not correct!"<<std::endl;
    		new_stimulus= pest.NextStimulus(0);
    		second_stimulus = max_stiffness-new_stimulus;
	        std::cout<<"next stimulus0: "<<new_stimulus<<"  "<<second_stimulus<<std::endl;
	        //std::cout<<"first "<< first_stimulus <<std::endl;
    	    //std::cout<<"second: "<< second_stimulus<<std::endl;
    	    if (pest.flag==false) 
    	    {
    	    	std::cout<<"step"<<endl;
    	        exit(-1);
    	    }
        //std::cout<<"nextttt: "<<bool_random<<std::endl;
    	
   	 	}
    bool_random=rand() % 2;
	}

}

//------------------------------------------------------------------------------

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
    while(simulationRunning)
    {
    	//loopcount++;
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        //cVector3d position;
        hapticDevice->getPosition(position);
        
        // read orientation 
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read gripper position
        //double gripperAngle;
        //hapticDevice->getGripperAngleRad(gripperAngle);q

        // read linear velocity 
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        // read angular velocity
        //cVector3d angularVelocity;
        //hapticDevice->getAngularVelocity(angularVelocity);

        // read gripper angular velocity
        //double gripperAngularVelocity;
        //hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

        // read user-switch status (button 0)
        //bool button0, button1, button2, button3;
        //button0 = false;
        //button1 = false;
        //button2 = false;
        //button3 = false;

        //hapticDevice->getUserSwitch(0, button0);
        //hapticDevice->getUserSwitch(1, button1);
        //hapticDevice->getUserSwitch(2, button2);
        //hapticDevice->getUserSwitch(3, button3);


        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
       
        // update arrow
        velocity->m_pointA = position;
        velocity->m_pointB = cAdd(position, linearVelocity);

        // update position and orientation of cursor
        cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);

        // adjust the  color of the cursor according to the status of
        // the user-switch (ON = TRUE / OFF = FALSE)
       if (*(double *)shared_data > 10.0)
        {
            cursor->m_material->setGreenMediumAquamarine();
        }
        /*
        else if (button1)
        {
            cursor->m_material->setYellowGold();
        }
        else if (button2)
        {
            cursor->m_material->setOrangeCoral();
        }
        else if (button3)
        {
            cursor->m_material->setPurpleLavender();
        }
        else
        {
            cursor->m_material->setBlueRoyal();
        }

        // update global variable for graphic display update
        hapticDevicePosition = position;

*/
        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d desiredPosition;
        desiredPosition.set(0.0, 0.0, 0.0);

        // desired orientation
        cMatrix3d desiredRotation;
        desiredRotation.identity();
        
        // variables for forces
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;
        
        if (isSame(prev_stiffness,stiffness))    
        {
	    	//std::cout<<"add"<<std::endl;
	    	loopcount=0;
	        prev_stiffness=stiffness;
	        forceField = stiffness * (desiredPosition - position);
	        force.add(gravityCorrection);
	        force.add(forceField);
	        //prev_stiffness=stiffness;
	   		hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
		}    
	    else
	    {
	    	loopcount++;
	    	if (loopcount==1){
	    		add= (stiffness-prev_stiffness)/200.0;
	    		//std::cout<<"qqqqqqqqqqqqqqqqqqq: "<<stiffness<<"   "<<prev_stiffness<<"   "<<add<<endl;

	    	}
	    	//double add_stiff=0.0;
	    	force.add(gravityCorrection);
	    	//for (int i=0;i<5;i++){
	    		//add_stiff=add_stiff+add;
	    		forceField = (prev_stiffness+add) * (desiredPosition - position);

	    		force.add(forceField);
	    		//std::cout<<"hereeee: "<<"add" <<add <<" pre "<<prev_stiffness <<" sum "<<prev_stiffness+add<< " goal: "<<stiffness<<endl;
	    		
	    		//std::cout<<"hereeee: "<<"add" <<add<<"    "<<prev_stiffness+add<<endl; 
	    		prev_stiffness=prev_stiffness+add;
	    		//if (isSame(prev_stiffness,stiffness))  std::cout<<"yeeeeessssssss"<<std::endl;
				hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);	
	    	//}
	    }

		
        // signal frequency counter
        //if (stiffness<300){
        //	        desiredPosition.set(0.03, 0.0, 0.0);

        //}
        /*
        forceField = stiffness * (desiredPosition - position);
		force.add(forceField);
        cVector3d gravityCorrection (3.5, 0.0, -2.5);
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
*/
   		freqCounterHaptics.signal(1);
    }
    //freqCounterHaptics.signal(1);
    
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

    auto t1 = std::chrono::high_resolution_clock::now();
	while (!exitKey) {
        bird.getCoordinatesAngles( sennum, dX, dY, dZ, dAzimuth, dElevation, dRoll );
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        FT_data = Force.GetCurrentFT(numSample) ;
        *(Vector6FT*)(shared_data) = FT_data;
        myfile <<FT_data[0]<<" "<< FT_data[1]<< " "<<FT_data[2]<< " "<<FT_data[3]<<" "<<FT_data[4]<<" "<< FT_data[5]<< " "<<position(0)<< " "<< position(1)<< " "<< position(2)<< " "<<stiffness<<" "<<dX<< " "<<dY<< " "<<dZ<< " "<<dAzimuth<< " "<<dElevation<< " "<<dRoll<< " "<<duration<<" "<<new_stimulus<<" "<<first_stimulus<<" "<<second_stimulus<<"\n";
        //myfile2 <<FT_data[0]<<" "<< FT_data[1]<< " "<<FT_data[2]<< " "<<FT_data[3]<<" "<<FT_data[4]<<" "<< FT_data[5]<< " "<<position(0)<< " "<< position(1)<< " "<< position(2)<< " "<<Kp<<" "<<dX<< " "<<dY<< " "<<dZ<< " "<<dAzimuth<< " "<<dElevation<< " "<<dRoll<< " "<<duration<<"\n";
        //std::cout << "\rX: " << dX << ", \tY: " << dY << ", \tZ: " << dZ;
        //std::cout << ", \tA: " << dAzimuth << ", \tE: " << dElevation << ", \tR: " << dRoll << std::endl;
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
    camera->set( cVector3d (0.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

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