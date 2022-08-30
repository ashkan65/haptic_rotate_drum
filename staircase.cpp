//==============================================================================
/*
      This library includes staircase psychophysics method.
    
    
    \author    Naghmeh Zamani
    \version   1.0 $Rev: 1 $
    \date      13/03/2021
*/
//==============================================================================

#include <iostream>
#include "staircase.hpp"
using namespace std;
//------------------------------------------------------------------------------
Staircase::Staircase(double min, double max,double _step_size){  //Instructor to initialize 
    Initialize(min, max, _step_size);
}

void Staircase::Initialize(double _min, double _max,double _step_size) //Initialize max min and step size
{
    srand (time(NULL));
    min = _min;
    max = _max;
    step_size = _step_size;
    first_time=true;
    new_stimulus[0]=min;
    n=0;
}


double Staircase::NextStimulus(double answer) //next stimulus receives an answer and calculates the new stimulus
{ 
    n++;
    if (first_time==true){
        prev_answer=answer;
        first_time=false;
        std::cout<<"haaaaaaaaaaaaaaaa"<<endl;
    } 

    if ((first_time==false) && (prev_answer != answer)) 
    {
        direction= (-1)*direction;          //direction has changed

    }

    NewStimulusGenerator();  //Generate the next stimulus
    prev_answer=answer;      //replace the previous answer with the current answer.
    trial_number=n; //record trial number to be used for GetTrialNumber()
    //n++;
    std::cout<<new_stimulus[n]<<" oooo"<<new_stimulus[n-1]<<endl;
    return new_stimulus[n]; 
}

int Staircase::GetTrialNumber(){ //Get trial number to be used in main
    return trial_number;
}

void Staircase::NewStimulusGenerator(){
    double temp_stimulus = new_stimulus[n-1] - (direction) * (step_size);
     std::cout<<new_stimulus[n-1]<<"  "<<temp_stimulus<<endl;
    //if (temp_stimulus>=max){ //If after summation goes above max, then it halves
     //   double temp_stimulus = new_stimulus[n-1];
     //   std::cout<<"heelooooooo"<<endl;
    //}
    if (temp_stimulus <=min){  
        std::cout<<"heelooooooo22222222222"<<endl;
        double temp_stimulus = new_stimulus[n-1];
    }
    else if (temp_stimulus > (max-step_size)){  
        std::cout<<"terminate"<<endl;
         flag=false; 
    }
    else
    {
        new_stimulus[n]=temp_stimulus;
    }

}
