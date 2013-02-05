/**
 * @file imu_filter.h
 *
 * @date   Feb 3, 2013
 * @author Adam Panzica
 * @brief Class declaration for the IMU filter
 */

//License File

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
