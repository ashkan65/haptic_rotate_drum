#ifndef FORCE_H
#define FORCE_H
/*****************************************************
A wrapper for ATI Force Sensor that records and calibrates data from sensoray
Ver 1.0 by Naghme Nov-2020		
*****************************************************/
#include <chrono>
#include "simple826.hpp"
#include "type.hpp"
#include <thread>  //Might be better to do the threading outside!
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 1> Vector6FT;


class ForceSensor: public Simple826{
	private:
		Matrix6f calibration_matrix; //Calibration matrix for Mini45 F/T ATI sensor (FT23655)
		Vector6FT offset;//Calibration offset for each channel
		Vector6FT FTdata;//Force/Torque 6D vector required for calibration process
		Vector6FT FT;	//Force/Torque 6D vector required for calibration process		
		int adcbuf[16]; // Pointer to a buffer that will receive ADC results for the sixteen possible slots. The buffer must be large enough to
        //accommodate sixteen values regardless of the number of active slots. Each slot is represented by a 32-bit value,
        //which is stored at buf[SlotNumber].
		double data[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0}; // data for each channel/slot

	public:
		ForceSensor();
		// ~ForceSensor();
		void FTAverageFilter(int _numsample,Vector6FT * _FT);// Getting average from each "_numsample" values in the buffer and presents one averaged value for each channel instead
		void FTSetOffset(int num_initread);// Creating an offset by getting an average from the first "num_initread" values of the AIN for calibration
		Vector6FT GetOffset();// Get the offset value for each channel
		void Voltage2FT(int _numsample,Vector6FT * _FT); 
		Vector6FT  GetCurrentFT(int _numsample);
 
};
#endif // FORCE_H