/**
 * @file imu_filter.cpp
 *
 * @date   Feb 3, 2013
 * @author Adam Panzics
 * @brief implementation of the IMUFilter class
 */

/*
 * Copyright (c) 2013, RIVeR Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
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

