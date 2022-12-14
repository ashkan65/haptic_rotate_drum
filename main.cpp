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
{//2200 100 2000 16
    //srand(time(NULL)); 
    //max_stiffness=atof(argv[1]);
    //min_delta_stiff=atof(argv[2]);
    //max_delta_stiff=atof(argv[3]);
    //std::cout<<"1: "<<max_stiffness<<" 2: "<<min_delta_stiff<<" 3: "<<max_delta_stiff<<endl;

    //double max_stiffness=2200;
    //double min_delta_stiff=100;//k[2200],k[2100]
    //double max_delta_stiff=2000.0;//k[2200],k[200]
    //double initial_step=(max_delta_stiff-min_delta_stiff)/16.0;
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
    // SHAPE - CYLINDER 0
    ////////////////////////////////////////////////////////////////////////////

    // create a cylinder
    cylinder0 = new cShapeCylinder(0.03, 0.03, 0.05);
    world->addChild(cylinder0);

    // set position and orientation
    cylinder0->setLocalPos(0.03, -0.0, -0.05);
    // cylinder->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90);

    // set material color
    cylinder0->m_material->setBlueCornflower();

    // create haptic effect and set properties
    cylinder0->createEffectSurface();
    cylinder0->m_material->setStiffness(maxStiffness);

    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - CYLINDER 1
    ////////////////////////////////////////////////////////////////////////////

    // create a cylinder
    cylinder1 = new cShapeCylinder(0.03, 0.03, 0.05);
    world->addChild(cylinder1);

    // set position and orientation
    cylinder1->setLocalPos(-0.030, 0.034, -0.05);
    // cylinder->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90);

    // set material color
    cylinder1->m_material->setYellowGold();

    // create haptic effect and set properties
    cylinder1->createEffectSurface();
    cylinder1->m_material->setStiffness(maxStiffness);

    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - CYLINDER 2
    ////////////////////////////////////////////////////////////////////////////

    // create a cylinder
    cylinder2 = new cShapeCylinder(0.03, 0.03, 0.05);
    world->addChild(cylinder2);

    // set position and orientation
    cylinder2->setLocalPos(-0.030, -0.034, -0.05);
    // cylinder->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90);

    // set material color
    cylinder2->m_material-> setPurpleViolet();

    // create haptic effect and set properties
    cylinder2->createEffectSurface();
    cylinder2->m_material->setStiffness(maxStiffness);

    //--------------------------------------------------------------------------
    // CREATE Pentagon
    //--------------------------------------------------------------------------

    /*
    // create a virtual mesh
    object = new cMesh();

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setLocalPos(0.0, 0.0, 0.0);

    // Since we want to see our polygons from both sides, we disable culling.
    object->setUseCulling(false);

    cColorf color;
    color.setRed();

    cVector3d p0 = cVector3d(0.0, -0.07, -0.10);
    cVector3d p1 = cVector3d(0.0, -0.1, 0.03);
    cVector3d p2 = cVector3d(0.0, 0.0, 0.1);
    cVector3d p3 = cVector3d(0.0, 0.1, 0.03);
    cVector3d p4 = cVector3d(0.0, 0.07, -0.1);

    int vertex0 = object->newVertex();
    int vertex1 = object->newVertex();
    int vertex2 = object->newVertex();
    int vertex3 = object->newVertex();
    int vertex4 = object->newVertex();

    // set position of each vertex
    object->m_vertices->setLocalPos(vertex0, p0);
    object->m_vertices->setLocalPos(vertex1, p1);
    object->m_vertices->setLocalPos(vertex2, p2);
    object->m_vertices->setLocalPos(vertex3, p3);
    object->m_vertices->setLocalPos(vertex4, p4);

    // assign color to each vertex
    object->m_vertices->setColor(vertex0, color);
    object->m_vertices->setColor(vertex1, color);
    object->m_vertices->setColor(vertex2, color);
    object->m_vertices->setColor(vertex3, color);
    object->m_vertices->setColor(vertex4, color);

    // create new triangle from vertices
    object->newTriangle(vertex0, vertex1, vertex2);
    object->newTriangle(vertex0, vertex2, vertex3);
    object->newTriangle(vertex0, vertex3, vertex4);
    */
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
    //--------------------------------------------------------------------------
    // START THREADS
    //--------------------------------------------------------------------------
    Vector6FT  shared_data;
    void *a;
    a = &shared_data;
    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    forceThread = new cThread();
    //time_t ta=time(NULL);
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS, a);
    forceThread->start(readFTdata , CTHREAD_PRIORITY_HAPTICS, a);
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



