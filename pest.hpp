//==============================================================================
/*
    This library includes an adaptive psychophysics methods called PEST
    (Parameter Estimation by Sequential Testing).
    
    The description will be in terms of stimulus differences
    for a Yes-No experiment. The forced-choice equivalent 
    substitutes "correct" for "yes" and "incorrect" for "no."
    There are 6 rules in this methods:

    1. On every reversal of step direction, halve the step size.
    2. The second step in a given direction, if called for, is
    the same size as the first.
    The size of the initial stimulus difference is set to a 
    constant multiplied by the exit criterion. 
    In order that a decision can be made to the
    response following the first stimulus presentation, it
    is necessary to assume the step size and the direction
    for two (imaginary) presentations prior to the first
    actual stimulus presentation.


    3. The fourth and subsequent steps in a given direction
    are each double their predecessor (except that, as
    noted above, large steps may be disturbing to a human
    observer and an upper limit on permissible step size
    may be needed).
    4. Whether a third successive step in a given direction
    is the same as or double the second depends on the sequence
    of steps leading to the most recent reversal. If
    the step immediately preceding that reversal resulted
    from a doubling, then the third step is not doubled,
    while if the step leading to the most recent reversal was
    not the result of a doubling, then this third step is
    double the second. 


    
    5. Exit rule. If the step size upon the next trial
    is less than an arbitrary pre-set exit criterion, the
    trial is terminated. The value of the stimulus on the
    next trial is the point estimate of its threshold.
    6. Maximum rule. The maximum size step is
    limited to a constant multiplied by the exit criterion.
    7. "Bumping into zero" rule. If the step size will
    modify the size of the stimulus difference to a difference
    below zero, the step size is successively halved
    
    \author    Naghmeh Zamani
    \version   1.0 $Rev: 1 $
    \date      13/03/2021
*/
//==============================================================================
#ifndef PEST_H
#define PEST_H

#include <iostream>

//------------------------------------------------------------------------------

class Pest{
    private:
        double new_stimulus[100];   // Pest creates the next stimulus based on previous stimuli in each iteration and keep it in new_stimulus[i]
        double min_step_size;        //Rule 5: min_step is the minimum step size that we indicate to exit when (new_stimulus[n] - new_stimulus[n-1]) reaches there
        double max_step_size;            //Rule 6: maximum step that is (max-min)
        double max;                 // Maximum stimulus that is given by user
        double min;                 // Minimum stimmulus that is given by user
        int prev_answer=0;          // Pest requires to know the previous answer in order to choose the next step size
        double direction =1.0;      //Direction of new stimulus: it is 1 if the next stimulus is in the same direction as theprevious one, otherwise (-1)
        int step_three=2; // Rule 4:  Whether a third successive step in a given direction is the same as or double the second depends on the on the sequence
                            // of steps leading to the most recent reversal. (step_three=2, since it starts from 0).
        bool doubled=false; //It is a flag for Rule 4, to decide wether the step size should be the same or doubled
        double init_step_size; // Rule 2: the first and second step size is equal to init_step which has to be a multiplication of max_step
        int trial_number=0; // Which trial number are we in. starts from 0 to total trial numbers(that has to be defined in main).
        int num_steps_samedir=0; // number of steps in one direction after each change of direction. Everytime restarts from 0.
        int n=0; // iteration number that is used for trial number
        double step; //step size
        bool first_time=true; //first time flag: If at the begining starts in one direction and reaches the third step it has to be doubled. 
        void SetInitialSteps(double answer); //Rule 2: For the first and second iterations step size is equal to initial step, and direction changes upon reversal
        void NewStimulusGenerator(); //Generates new stimulus: = previous stimulus + (direction * step size)
        void UpdateStepSize(); // Update the step size for a condition that num of steps in the same direction is = ,< , > step three.
    public:
        bool flag = true; //Rule 5: exit rule if threshold reaches minimum step.
        int GetTrialNumber(); //get trial number to be used in main: if the threshhold doesn't reach the min step size, after how many steps we want it to exit.
        Pest(double min, double max,double _init_step,double _min_step_size); //Instructor to initialize 
        void Initialize(double _min, double _max,double _init_step_size,double _min_step_size); //Initialize max min and initial step size
        double NextStimulus(double answer); //next stimulus is the final function that user can use it in main to receive the next stimulus
        

};
#endif

