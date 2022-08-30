//==============================================================================
/*
    This library includes staircase psychophysics method.
    
    
    \author    Naghmeh Zamani
    \version   1.0 $Rev: 1 $
    \date      30/03/2021
*/
//==============================================================================
#ifndef STAIRCASE_H
#define STAIRCASE_H

#include <iostream>

//------------------------------------------------------------------------------

class Staircase{
    private:
        double new_stimulus[100];   //  creates the next stimulus based on previous stimuli in each iteration and keep it in new_stimulus[i]
        double step_size;        //Constant step size
        double max;                 // Maximum stimulus that is given by user
        double min;                 // Minimum stimmulus that is given by user
        int prev_answer;          // staircase requires to know the previous answer in order to choose the next stimuli
        double direction =-1.0;      //Direction of new stimulus: it is 1 if the next stimulus is in the same direction as theprevious one, otherwise (-1)
        int trial_number=0; // Which trial number are we in. starts from 0 to total trial numbers(that has to be defined in main).
        int n; // iteration number that is used for trial number
        void NewStimulusGenerator(); //Generates new stimulus: = previous stimulus + (direction * step size)
        void UpdateStepSize(); // Update the step size for a condition that num of steps in the same direction is = ,< , > step three.
        bool first_time;
    public:
        int GetTrialNumber(); //get trial number to be used in main: if the threshhold doesn't reach the min step size, after how many steps we want it to exit.
        Staircase(double min, double max,double __step_size); //Instructor to initialize 
        void Initialize(double _min, double _max,double _step_size); //Initialize max min and initial step size
        double NextStimulus(double answer); //next stimulus is the final function that user can use it in main to receive the next stimulus
        bool dir=false;
        int num=0;
        bool flag = true;



};
#endif

