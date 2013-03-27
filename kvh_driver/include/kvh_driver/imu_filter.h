/**
 * @file imu_filter.h
 *
 * @date   Feb 3, 2013
 * @author Adam Panzica
 * @brief Class declaration for the IMU filter
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

#ifndef IMU_FILTER_H_
#define IMU_FILTER_H_

//*****************LOCAL DEPENDANCIES**************************//
#include<kvh_driver/linearfilter.h>
//**********************NAMESPACES*****************************//


namespace kvh_driver
{
using MatrixWrapper::Matrix;
using MatrixWrapper::ColumnVector;
using MatrixWrapper::SymmetricMatrix;
using namespace BFL;

class IMUFilter: public LinearFilter
{
public:
	/**
	 * @author Adam Panzica
	 * @brief Constructs a new IMU filter given an expected system noise and covariance and measurement noise and covariance
	 * @param [in] sys_noise_mu The expected system noise
	 * @param [in] sys_noise_cov The expected system covariance
	 * @param [in] measurement_noise_mu The expected measurement noise
	 * @param [in] measurement_noise_cov The expected measurement covariance
	 */
	IMUFilter(const ColumnVector& sys_noise_mu, const SymmetricMatrix& sys_noise_cov, const ColumnVector& measurement_noise_mu, const SymmetricMatrix& measurement_noise_cov);
	virtual ~IMUFilter();

private:

	const Matrix buildA();
	const Matrix buildB();
	const Matrix buildH();

};

}; /* END KVH_DRIVER */

#endif /* IMU_FILTER_H_ */
