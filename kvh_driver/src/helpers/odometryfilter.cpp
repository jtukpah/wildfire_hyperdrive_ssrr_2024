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
		LinearFilter(constants::ODOM_STATE_SIZE(), constants::INPUT_SIZE(), constants::MEASUREMENT_SIZE(), sys_noise_mu, sys_noise_cov, measurement_noise_mu, measurement_noise_cov)
{

}

void OdometryFilter::buildAB()
{
	//Build System Matrix
	Matrix A(constants::ODOM_STATE_SIZE(),constants::ODOM_STATE_SIZE());
	A = 0;
	for (int r = 0; r < constants::ODOM_STATE_SIZE(); ++r)
	{
		A(r,r) = 1;
	}
	A(constants::ODOM_X_STATE(),  constants::ODOM_X_DOT_STATE())  = 1;
	A(constants::ODOM_Y_STATE(),  constants::ODOM_Y_DOT_STATE())  = 1;
	A(constants::ODOM_Z_STATE(),  constants::ODOM_Z_DOT_STATE())  = 1;
	A(constants::ODOM_RX_STATE(), constants::ODOM_RX_DOT_STATE()) = 1;
	A(constants::ODOM_RY_STATE(), constants::ODOM_RY_DOT_STATE()) = 1;
	A(constants::ODOM_RZ_STATE(), constants::ODOM_RZ_DOT_STATE()) = 1;

	//Build Input Matrix
	Matrix B(12,3);
	B = 0;
	B(constants::ODOM_X_DOT_STATE(), constants::X_DOT_DOT_INPUT()) = 1;
	B(constants::ODOM_Y_DOT_STATE(), constants::Y_DOT_DOT_INPUT()) = 1;
	B(constants::ODOM_Z_DOT_STATE(), constants::Z_DOT_DOT_INPUT()) = 1;

	//Build System Evolution Matrix
	this->AB_[0]   = A;
	this->AB_[1]   = B;
}

void OdometryFilter::buildH()
{

	//Build H Matrix
	this->H_ = new Matrix(constants::ODOM_STATE_SIZE(),constants::MEASUREMENT_SIZE());
	*this->H_ = 0;
	(*this->H_)(constants::ODOM_RX_DOT_STATE(), constants::RX_DOT_MEASUREMENT()) = 1;
	(*this->H_)(constants::ODOM_RY_DOT_STATE(), constants::RY_DOT_MEASUREMENT()) = 1;
	(*this->H_)(constants::ODOM_RZ_DOT_STATE(), constants::RZ_DOT_MEASUREMENT()) = 1;

}
