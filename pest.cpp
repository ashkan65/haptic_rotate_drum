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

#include <iostream>
#include "pest.hpp"
using namespace std;
//------------------------------------------------------------------------------
Pest::Pest(double min, double max,double _init_step_size, double _min_step_size){  //Instructor to initialize 
    Initialize(min, max, _init_step_size,_min_step_size);
}

void Pest::Initialize(double _min, double _max,double _init_step_size,double _min_step_size) //Initialize max min max_step size and initial_step size
{
    min = _min;
    max = _max;
    init_step_size = _init_step_size;
    max_step_size = max-min;
    min_step_size=_min_step_size;
    
}


double Pest::NextStimulus(double answer) //next stimulus receives an answer and calculates the new stimulus
{   
    if (n==0){ // Our stimuli starts from maximum(this is arbitrary, it can also start from minimum):
        new_stimulus[0]=max;
    }
    else if (n<3){ //For the first and second iterations step size is equal to initial step, and direction changes upon reversal
        SetInitialSteps(answer);
    }
    else //After third iterarion new stimulus depends on if previous answer is equal to current answer or not
    {
        if (prev_answer != answer) 
        {
            num_steps_samedir=0;                // count how many steps goes after each reversal (change of direction)
            direction= (-1)*direction;          //direction has changed
            step = step / 2.0;                  //Rule 1: step size halves
            if (step  >max_step_size) step=max_step_size; //Rule 6
            if (step  <min_step_size) flag=false;    //Rule 5
        }
        else if (prev_answer == answer)
        {
            num_steps_samedir++;                //when previous answer =current answer that means it still goes in the same direction
            if (first_time)                     // If revious answer =current answer and it's the first time starts the program steps doubles
            {
                step = 2.0 * step;
                num_steps_samedir=num_steps_samedir+3; //+3 to make sure it doesn't consider step three rule for the first time=>
                                                        //it makes sure next time  "UpdateStepSize()" uses condition (num_steps_samedir>step_three)
                                                        //to double the step size even if it's at step three
            }
            else{
                UpdateStepSize();   //Update next step size            
            }
            if (step  >max_step_size) step=max_step_size;  //Rule 6
            if (step  <min_step_size) flag=false;     //Rule 5

        }
        NewStimulusGenerator();  //Generate the next stimulus
        prev_answer=answer;      //replace the previous answer with the current answer.
        first_time =false;        //exit the first time condition
    }
    trial_number=n; //record trial number to be used for GetTrialNumber()
    n++;
    return new_stimulus[n-1]; 
}

int Pest::GetTrialNumber(){ //Get trial number to be used in main
    return trial_number;
}

void Pest::NewStimulusGenerator(){
    double temp_stimulus = new_stimulus[n-1] + (direction) * (step);
    if (temp_stimulus>=max){ //If after summation goes above max, then it halves
        new_stimulus[n]=(new_stimulus[n-1]+max)/2.0;
        step = max-new_stimulus[n];
        std::cout<<"heelooooooo"<<endl;
    }
    else if (temp_stimulus <=min){  //Rule 7: "Bumping into zero" rule. If the step size will modify the size of the stimulus 
                                    //difference to a difference below zero, the step size is successively halved
        new_stimulus[n]=(new_stimulus[n-1]+min)/2.0;
        step = new_stimulus[n]-min;
    }
    else
    {
        new_stimulus[n]=temp_stimulus;
    }

}


void Pest::UpdateStepSize() //keep the step size to be the the same if (num_steps_samedir<step_three),                             
{
    if(num_steps_samedir==step_three) // when (num_steps_samedir=step_three), if step size doubled before reversal step size 
                                      //stays the same if step size didn't double before reversal then double the step size.
    {
            if(doubled==false)
            {
                step= 2.0 * step;
                doubled = true;
            }
            else if(doubled ==true)
            {
                doubled =false;
            }

    }
    else if(num_steps_samedir>step_three)   //double step size if (num_steps_samedir>step_three), 
    {
            step = 2.0 * step;
    }

}
                      

void Pest::SetInitialSteps(double answer){ //Rule 2: For the first and second iterations step size is equal to initial step, and direction changes upon reversal
    step = init_step_size;
    if (prev_answer != answer)
    {
        direction = -1.0 * direction;
    }
    new_stimulus[n]=new_stimulus[n-1]+ direction * (init_step_size);
    prev_answer=answer;
}
