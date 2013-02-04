/**
 * @file	odometryfilter.hpp
 * @date	Feb 2, 2013
 * @author	Adam Panzica
 * @brief	EKF filter definitions for filtering KVH IMU data
 */

/*
* LICENSE FILE
*/

#ifndef FILTER_HPP_
#define FILTER_HPP_

//******************* SYSTEM DEPENDANCIES ****************//

//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/linearfilter.h>
//*********************** NAMESPACES ********************//
namespace kvh_driver
{
using MatrixWrapper::Matrix;
using MatrixWrapper::ColumnVector;
using MatrixWrapper::SymmetricMatrix;
using namespace BFL;

/**
 * @author Adam Panzica
 * Simple Extended Kalman Filter implementation based on BFL's filtering library. It is hard-coded to assume the
 * following system dynamics:
 *
 * \f[
 * \textbf{X} = \f{bmatrix}{ x \\ y \\ z \\ rx \\ ry \\ rz \\ \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{rx} \\ \dot{ry} \\ \dot{rz} \f}
 * \f]
 * \f[
 * \textbf{A} = \f{bmatrix}{ 1 0 0 0 0 0 1 0 0 0 0 0\\
 * 							 0 1 0 0 0 0 0 1 0 0 0 0\\
 * 							 0 0 1 0 0 0 0 0 1 0 0 0\\
 * 							 0 0 0 1 0 0 0 0 0 1 0 0\\
 * 							 0 0 0 0 1 0 0 0 0 0 1 0\\
 * 							 0 0 0 0 0 1 0 0 0 0 0 1\\
 * 							 0 0 0 0 0 0 1 0 0 0 0 0\\
 * 							 0 0 0 0 0 0 0 1 0 0 0 0\\
 * 							 0 0 0 0 0 0 0 0 1 0 0 0\\
 * 							 0 0 0 0 0 0 0 0 0 1 0 0\\
 * 							 0 0 0 0 0 0 0 0 0 0 1 0\\
 * 							 0 0 0 0 0 0 0 0 0 0 0 1\f}
 * \f]
 * \f[
 * \textbf{B} = \f{bmatrix}{ 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 1 0 0 \\
 * 							 0 1 0 \\
 * 							 0 0 1 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \f}
 * \f]
 * \textbf{u} = \f{bmatrix}{ \ddot{x}\\
 *                           \ddot{y}\\
 *                           \ddot{z}\f}
 * \f]
 * \textbf{H} = \f{bmatrix}{ 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 0 0 0 \\
 * 							 1 0 0 \\
 * 							 0 1 0 \\
 * 							 0 0 1 \f}
 * \f]
 * \textbf{z} = \f{bmatrix}{ \dot{rx}\\
 *                           \dot{ry}\\
 *                           \dot{rz}\f}
 * \f]
 */
class OdometryFilter: public LinearFilter
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
	OdometryFilter(const ColumnVector& sys_noise_mu, const SymmetricMatrix& sys_noise_cov, const ColumnVector& measurement_noise_mu, const SymmetricMatrix& measurement_noise_cov);
	virtual ~OdometryFilter();

protected:
	virtual void buildAB();

	virtual void buildH();
};

} /*END NAMESPACE KVH_DRIVER */


#endif /* FILTER_HPP_ */
