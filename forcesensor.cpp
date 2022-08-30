#include "forcesensor.hpp"
#include <iostream>

ForceSensor::ForceSensor(){
    //Calibration matrix for Nano17 F/T ATI sensor (in Newton) (FT23660)
	calibration_matrix << 0.00218,   0.00777, -0.07732, -3.31152,   0.02261,  3.28962,
		-0.02655,   4.05829, - 0.06129, - 1.88839, - 0.00746, - 1.91908,
		3.79085,   0.00200,   3.81456, - 0.06195,   3.81532, - 0.07418,
		-0.77256,  24.43408,  20.89700, - 11.69060, - 21.72983, - 11.19770,
		- 23.58908, - 0.08840,  12.77264,  19.82763,  12.98916, - 20.10740,
		0.01391,  14.38601,   0.36843,  13.98652,   0.25728,  14.55357;	

	//Calibration matrix for Mini45 F/T ATI sensor (in Newton) (FT23655)
	/*
	calibration_matrix << 0.41388, 0.14592, 4.19733, -47.43412, -2.46789, 46.50125, 
		-4.25833, 54.21553, 3.63101, -27.03922, -0.56011, -27.53264,
		66.98523, 3.36221, 67.82584, 2.86437, 66.75031, 4.08293,
		-0.03263, 0.38512, -1.06996, -0.24152, 1.05840, -0.12555,
		1.24766, 0.07060, -0.65865, 0.30393, -0.62320, -0.36835,
		0.06922, -0.69457, 0.03705, -0.69582, 0.04867, -0.69297;
		*/
};
// Getting average from each "_numsample" values in the buffer and presents one averaged value for each channel instead(_avgFT)
void ForceSensor::FTAverageFilter(int _numsample, Vector6FT * _avgFT){
	double avg[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for (int i=0;i<_numsample;i++){
		ReadAdcOutput(adcbuf,data);
		for (int i=0;i<6;i++){
			avg[i] += data[i]/_numsample;
		}
	}

    (*_avgFT) << avg[0], avg[1], avg[2], avg[3], avg[4], avg[5];
};

// Creating an offset by getting an average from the first "num_initread" values of the AIN for calibration
void ForceSensor::FTSetOffset(int num_initread){
	FTAverageFilter(num_initread, &offset);
};

// Get the offset value for each channel
 Vector6FT ForceSensor::GetOffset(){
 	return offset;
 };

//Converting voltage to Force/Torque, using calibration_matrix and offset
 void ForceSensor::Voltage2FT(int _numsample, Vector6FT * _FT) {
 	FTAverageFilter(_numsample, &FTdata);
	FTdata = FTdata - offset;
	*_FT = calibration_matrix * FTdata ;
	//std::cout<<FT[2]<<std::endl;
};

 Vector6FT ForceSensor::GetCurrentFT(int _numsample){
 	Voltage2FT(_numsample, &FT); 
 	return FT;
 };