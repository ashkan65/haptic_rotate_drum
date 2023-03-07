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

#include "hapticengine.hpp"



//==============================================================================
/*
    DEMO:   01-mydevice.cpp
    This application illustrates how to program forces, torques and gripper
    forces to your haptic device.
    In this example the application opens an OpenGL window and displays a
    3D cursor for the device connected to your computer. If the user presses 
    onto the user button (if available on your haptic device), the color of 
    the cursor changes from blue to green.
    In the main haptics loop function  "updateHaptics()" , the position,
    orientation and user switch status are read at each haptic cycle. 
    Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    vector<int>  shared_data={0,0,0};
    if (argc==1){
        std::cout<<"Participant number and experiment number, and file number are not entered"<<std::endl;
        return 0;
    }
    else if (argc==4){
        shared_data[0]=atoi(argv[1]);
        shared_data[1]=atoi(argv[2]);
        shared_data[2]=atoi(argv[3]);
        std::cout<<"Par"<<shared_data[0]<<" "<<shared_data[1]<<" "<<shared_data[2]<<std::endl;
    }
    else{
       std::cout<<"Participant number and experiment number, and file number are not entered correctly"<<std::endl; 
    }
       //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 01-mydevice" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;




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
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
   // glfwSwapInterval(swapInterval);

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
    camera->set( cVector3d (0.25, 0.0, 0.25),    // camera position (eye)
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
    light->setDir(-0.7, 0.2, -1.0);

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

    cursor->setShowEnabled(false); // Default to hide
    velocity->setShowEnabled(false);

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);



    //--------------------------------------------------------------------------
    // CREATING SHAPES (Zhian Li)
    //--------------------------------------------------------------------------    

    ////////////////////////////////////////////////////////////////////////////
    // In the following lines create the scene with a sphere representing the 
    // tracker (birdSphere), three cylinders representing the plates. birdSphere 
    // represents the tracker,
    ////////////////////////////////////////////////////////////////////////////

    // get properties of haptic device
    double maxStiffness = 2000;


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - BIRD SPHERE
    ////////////////////////////////////////////////////////////////////////////

    // create a sphere
    birdSphere = new cShapeSphere(0.1);
    world->addChild(birdSphere);

    // set position
    birdSphere->setLocalPos(0.0,0.1, 0.0);
    
    // set radius
    birdSphere->setRadius(0.01);

    // set material color
    birdSphere->m_material->setRedFireBrick();

    // create haptic effect and set properties
    birdSphere->createEffectSurface();
    
    // set stiffness property
    birdSphere->m_material->setStiffness(0.4 * maxStiffness);

    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - BOX 0, BOX 1, BOX 2, BOX 3
    ////////////////////////////////////////////////////////////////////////////

    std::vector<cShapeBox*> boxes {box0, box1, box2, box3};
    std::vector<cVector3d> box_positions{
        cVector3d(0.03, 0.03, -0.025),
        cVector3d(-0.03, 0.03, -0.025),
        cVector3d(-0.03, -0.03, -0.025),
        cVector3d(0.03, -0.03, -0.025)
    };

    for (int i = 0; i < boxes.size(); i++){
        boxes[i] = new cShapeBox(0.05, 0.05, 0.05);
        world->addChild(boxes[i]);

        // set position and orientation
        boxes[i]->setLocalPos(box_positions[i]);
        boxes[i]->m_material->setYellowGold();

        boxes[i]->createEffectSurface();
        // boxes[i]->m_material->setStiffness(maxStiffness);
    }
    
    // save pointers to global variables
    box0 = boxes[0];
    box1 = boxes[1];
    box2 = boxes[2];
    box3 = boxes[3];

    /* 
    // Zhian: Set texture attempt
    // load a texture image file    // create a texture map
    box0->m_texture = cTexture2d::create();
    box0->m_texture->loadFromFile("lenna.png");
    box0->m_material->setTextureLevel(1.0);

    // enable texture mapping
    box0->setUseTexture(true);
    // assign a white material color that is modulated with the texture
    box0->m_material->setWhite();
    */

    cShapeBox* wall0 = new cShapeBox(0.11, 0.01, 0.08);
    world->addChild(wall0);

    // set position and orientation
    wall0->setLocalPos(0, 0, -0.025);
    wall0->m_material->setBlueSky();

    cShapeBox* wall1 = new cShapeBox(0.01, 0.11, 0.08);
    world->addChild(wall1);

    // set position and orientation
    wall1->setLocalPos(0, 0, -0.025);
    wall1->m_material->setBlueSky();

    cShapeBox* wall2 = new cShapeBox(0.13, 0.01, 0.08);
    world->addChild(wall2);

    // set position and orientation
    wall2->setLocalPos(0, 0.06, -0.025);
    wall2->m_material->setBlueSky();

    cShapeBox* wall3 = new cShapeBox(0.13, 0.01, 0.08);
    world->addChild(wall3);

    // set position and orientation
    wall3->setLocalPos(0, -0.06, -0.025);
    wall3->m_material->setBlueSky();

    cShapeBox* wall4 = new cShapeBox(0.01, 0.13, 0.08);
    world->addChild(wall4);

    // set position and orientation
    wall4->setLocalPos(0.06, 0, -0.025);
    wall4->m_material->setBlueSky();

    cShapeBox* wall5 = new cShapeBox(0.01, 0.13, 0.08);
    world->addChild(wall5);

    // set position and orientation
    wall5->setLocalPos(-0.06, 0, -0.025);
    wall5->m_material->setBlueSky();
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

    // create a label to display the position of haptic device
    labelExperimentDisplay = new cLabel(font);
    camera->m_frontLayer->addChild(labelExperimentDisplay);
    

    cLabel* labelBox1 = new cLabel(font);
    camera->m_frontLayer->addChild(labelBox1);
    labelBox1->setText("Box 1");
    labelBox1->setLocalPos(600, 220, 0);

    cLabel* labelBox2 = new cLabel(font);
    camera->m_frontLayer->addChild(labelBox2);
    labelBox2->setText("Box 2");
    labelBox2->setLocalPos(600, 320, 0);

    cLabel* labelBox3 = new cLabel(font);
    camera->m_frontLayer->addChild(labelBox3);
    labelBox3->setText("Box 3");
    labelBox3->setLocalPos(220, 320, 0);

    cLabel* labelBox4 = new cLabel(font);
    camera->m_frontLayer->addChild(labelBox4);
    labelBox4->setText("Box 4");
    labelBox4->setLocalPos(220, 220, 0);

    // Zhian Li: Add widgets for the blocks on the side
    using namespace chai3d;

    // create block object
    bitmapNumber1 = new cBitmap();
    // add bitmap to front layer of camera
    camera->m_frontLayer->addChild(bitmapNumber1);
    // load image file
    bitmapNumber1->loadFromFile("../texture/number1.png");
    bitmapNumber1->setZoom(0.25, 0.25);
    bitmapNumber1->setLocalPos(50, 64, 0);
    bitmapNumber1->setTransparencyLevel(0);

    // create block object
    bitmapNumber2 = new cBitmap();
    // add bitmap to front layer of camera
    camera->m_frontLayer->addChild(bitmapNumber2);
    // load image file
    bitmapNumber2->loadFromFile("../texture/number2.png");
    bitmapNumber2->setZoom(0.25, 0.25);
    bitmapNumber2->setLocalPos(50, 128, 0);
    bitmapNumber2->setTransparencyLevel(0);

    // create block object
    bitmapNumber3 = new cBitmap();
    // add bitmap to front layer of camera
    camera->m_frontLayer->addChild(bitmapNumber3);
    // load image file
    bitmapNumber3->loadFromFile("../texture/number3.png");
    bitmapNumber3->setZoom(0.25, 0.25);
    bitmapNumber3->setLocalPos(50, 192, 0);
    bitmapNumber3->setTransparencyLevel(0);

    // create block object
    bitmapNumber4 = new cBitmap();
    // add bitmap to front layer of camera
    camera->m_frontLayer->addChild(bitmapNumber4);
    // load image file
    bitmapNumber4->loadFromFile("../texture/number4.png");
    bitmapNumber4->setZoom(0.25, 0.25);
    bitmapNumber4->setLocalPos(50, 256, 0);
    bitmapNumber4->setTransparencyLevel(0);

    //--------------------------------------------------------------------------
    // START THREADS
    //--------------------------------------------------------------------------
    // Vector6FT shared_data;
    void *a;
    a = &shared_data[0];

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    forceThread = new cThread();
    EposThread = new cThread();
    //time_t ta=time(NULL);
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS, a);
    forceThread->start(readFTdata , CTHREAD_PRIORITY_HAPTICS, a);
    EposThread->start(runEpos , CTHREAD_PRIORITY_HAPTICS, a);

    //time_t tb=time(NULL);
    //std::cout << rec_count << " samples collected" << std::endl;
    //std::cout << tb-ta << " seconds elapsed" << std::endl;
    //std::cout << rec_count/(tb-ta) << " samples per second" << std::endl;
    // exit
    // setup callback when application exits
    atexit(close);

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}



