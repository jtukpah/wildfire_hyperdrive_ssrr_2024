/**
 * @file imu_filter.cpp
 *
 * @date   Feb 3, 2013
 * @author Adam Panzics
 * @brief implementation of the IMUFilter class
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<ros/ros.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<kvh_driver/imu_filter.h>
//**********************NAMESPACES*****************************//

using namespace kvh_driver;

IMUFilter::IMUFilter(const ColumnVector& sys_noise_mu, const SymmetricMatrix& sys_noise_cov, const ColumnVector& measurement_noise_mu, const SymmetricMatrix& measurement_noise_cov):
		LinearFilter(constants::IMU_STATE_SIZE(), 0, constants::IMU_STATE_SIZE(), sys_noise_mu, sys_noise_cov, measurement_noise_mu, measurement_noise_cov)
{

}

void IMUFilter::buildAB()
{
	//Build System Matrix
		Matrix A(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
		A = 0;
		for (int r = 0; r < constants::ODOM_STATE_SIZE(); ++r)
		{
			A(r,r) = 1;
		}

		//Build Input Matrix
		Matrix B(constants::IMU_STATE_SIZE(),1);
		B = 0;

		//Build System Evolution Matrix
		this->AB_[0]   = A;
		this->AB_[1]   = B;
}

void IMUFilter::buildH()
{
	//Build H Matrix
	this->H_  = new Matrix(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
	*this->H_ = 0;
	for (int r = 0; r < constants::IMU_STATE_SIZE(); ++r)
	{
		(*this->H_)(r,r) = 1;
	}
}

