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
				LinearFilter(constants::IMU_STATE_SIZE(), 0, constants::IMU_STATE_SIZE(), sys_noise_mu, sys_noise_cov, buildA(), buildB(), measurement_noise_mu, measurement_noise_cov, buildH())
{

}

IMUFilter::~IMUFilter()
{

}

const Matrix IMUFilter::buildA()
{
	//Build System Matrix
	Matrix A(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
	A = 0;
	for (int r = 1; r <= constants::ODOM_STATE_SIZE(); ++r)
	{
		A(r,r) = 1;
	}

	return A;
}

const Matrix IMUFilter::buildB()
{
	//Build Input Matrix
	Matrix B(0,0);
	B = 0;

	return B;
}

const Matrix IMUFilter::buildH()
{
	//Build H Matrix
	Matrix H(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
	H = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		H(r,r) = 1;
	}
	return H;
}

