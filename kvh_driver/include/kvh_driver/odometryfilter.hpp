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
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/constants.h>
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
class OdometryFilter
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

	/**
	 * @author Adam Panzica
	 * @brief Initializes the filter to an initial state estimate and covariance
	 * @param [in] initial_state The initial state estimate
	 * @param [in] initial_covar The initial state estimate covariance
	 * @return True if sucessfully intialized, else false
	 *
	 * Note that the size of the state vector and covariance matrix must match the size defined by the constant
	 * SYSTEM_STATE_SIZE
	 */
	bool init(const ColumnVector& initial_state, const SymmetricMatrix& initial_covar);

	/**
	 * @author Adam Panzica
	 * @brief Updates the filter given new system input and measurement
	 * @param [in] input The new system input
	 * @param [in] measurement The new state measurement
	 * @return True if succesfully updated, else false
	 * Note that the init method must be called before performing filter updates.
	 *
	 * Note that the size of the input and measurement vectors must match the size defined by the constants
	 * INPUT_SIZE and MEASUREMENT_SIZE
	 */
	bool update(const ColumnVector& input, const ColumnVector& measurement);

	/**
	 * @author Adam Panzica
	 * @brief Gets the latest state estimate from the filter
	 * @param [out] estimate Vector to write the state estimate to
	 * @param [out] covar Matrix to write the state estimate covariance to
	 * @return True if the estimate was sucessfully retrieved, else false
	 *
	 * Note that the init method must have been called prior to getting estimates
	 */
	bool getEstimate(ColumnVector& estimate, SymmetricMatrix& covar) const;

	/**
	 * @author Adam Panzica
	 * @return true if the filter is intialized, else false
	 */
	bool isInitialized();

private:
	/**
	 * The combined system evolution dynamics
	 */
	std::vector<Matrix>                                AB_;
	/**
	 * The measurement transform Matrix
	 */
	Matrix*										       H_;
	/**
	 * The system Gaussian PDF
	 */
	LinearAnalyticConditionalGaussian*                 sys_pdf_;
	/**
	 * The system uncertanty model
	 */
	LinearAnalyticSystemModelGaussianUncertainty*      sys_model_;
	/**
	 * The measurement Gaussian PDF
	 */
	LinearAnalyticConditionalGaussian*                 mes_pdf_;
	/**
	 * The measurement uncertanty model
	 */
	LinearAnalyticMeasurementModelGaussianUncertainty* mes_model_;

	/**
	 * Flag for signalling an initialized filter
	 */
	bool                                               filter_init_;
	/**
	 * The actual BFL ExtendedKalmanFilter for the system
	 */
	ExtendedKalmanFilter*                              filter_;
	/**
	 * The current state estimate as of the last filter update, or the initial state estimate if no update has been performed
	 */
	Gaussian*                                          prior_;
};

} /*END NAMESPACE KVH_DRIVER */


#endif /* FILTER_HPP_ */
