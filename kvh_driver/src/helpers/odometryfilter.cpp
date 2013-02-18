/**
 * @file	odometryfilter.cpp
 * @date	Feb 2, 2013
 * @author	Adam Panzica
 * @brief	Implementaiton of the IMUFilter class
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/odometryfilter.hpp>
//*********************** NAMESPACES ********************//

using namespace kvh_driver;

OdometryFilter::OdometryFilter(const ColumnVector& sys_noise_mu, const SymmetricMatrix& sys_noise_cov, const ColumnVector& measurement_noise_mu, const SymmetricMatrix& measurement_noise_cov):
				LinearFilter(constants::ODOM_STATE_SIZE(), constants::ODOM_INPUT_SIZE(), constants::ODOM_MEASUREMENT_SIZE(), sys_noise_mu, sys_noise_cov, buildA(), buildB(), measurement_noise_mu, measurement_noise_cov, buildH())
{

}

OdometryFilter::~OdometryFilter()
{

}

bool OdometryFilter::update(const ColumnVector& input, const ColumnVector& measurement)
{
	if(this->isInitialized())
	{
		//Convert the measurement/input to the correct frame
		Matrix rotation(3,3);
		double phi   = this->prior_->ExpectedValueGet()(constants::ODOM_RX_STATE());
		double theta = this->prior_->ExpectedValueGet()(constants::ODOM_RY_STATE());
		double psi   = this->prior_->ExpectedValueGet()(constants::ODOM_RZ_STATE());
		rotation(1,1)= cos(theta)*sin(psi);
		rotation(1,2)= -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
		rotation(1,3)= sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
		rotation(2,1)= rotation(1,1);
		rotation(2,2)= cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
		rotation(2,3)= -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
		rotation(3,1)= -sin(theta);
		rotation(3,2)= sin(phi)*cos(theta);
		rotation(3,3)= cos(phi)*cos(theta);

		ColumnVector rot_input(input);
		rot_input = rotation*rot_input;

		ColumnVector rot_measurement(measurement);
		rot_measurement = rotation*rot_measurement;
		return LinearFilter::update(rot_input, rot_measurement);
	}
	else
	{
		ROS_ERROR("Cannot Perform Update on Uninitialized Filter!");
		return false;
	}
}

const Matrix OdometryFilter::buildA()
{
	//Build System Matrix
	Matrix A(constants::ODOM_STATE_SIZE(),constants::ODOM_STATE_SIZE());
	A = 0;
	for (int r = 1; r <= constants::ODOM_STATE_SIZE(); ++r)
	{
		A(r,r) = 1;
	}
	A(constants::ODOM_X_STATE(),  constants::ODOM_X_DOT_STATE())  = 1;
	A(constants::ODOM_Y_STATE(),  constants::ODOM_Y_DOT_STATE())  = 1;
	A(constants::ODOM_Z_STATE(),  constants::ODOM_Z_DOT_STATE())  = 1;
	A(constants::ODOM_RX_STATE(), constants::ODOM_RX_DOT_STATE()) = 1;
	A(constants::ODOM_RY_STATE(), constants::ODOM_RY_DOT_STATE()) = 1;
	A(constants::ODOM_RZ_STATE(), constants::ODOM_RZ_DOT_STATE()) = 1;
	return A;
}

const Matrix OdometryFilter::buildB()
{
	//Build Input Matrix
	Matrix B(12,3);
	B = 0;
	B(constants::ODOM_X_DOT_STATE(), constants::X_DOT_DOT_INPUT()) = 1;
	B(constants::ODOM_Y_DOT_STATE(), constants::Y_DOT_DOT_INPUT()) = 1;
	B(constants::ODOM_Z_DOT_STATE(), constants::Z_DOT_DOT_INPUT()) = 1;
	return B;

}

const Matrix OdometryFilter::buildH()
{

	//Build H Matrix
	Matrix H(constants::ODOM_STATE_SIZE(),constants::ODOM_MEASUREMENT_SIZE());
	H = 0;
	H(constants::ODOM_RX_DOT_STATE(), constants::RX_DOT_MEASUREMENT()) = 1;
	H(constants::ODOM_RY_DOT_STATE(), constants::RY_DOT_MEASUREMENT()) = 1;
	H(constants::ODOM_RZ_DOT_STATE(), constants::RZ_DOT_MEASUREMENT()) = 1;
	return H;

}
