/**
 * @file linearfilter.h
 *
 * @date   Feb 4, 2013
 * @author parallels
 * @brief \todo
 */

//License File


#ifndef LINEARFILTER_H_
#define LINEARFILTER_H_

//****************SYSTEM DEPENDANCIES**************************//
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<kvh_driver/constants.h>
//**********************NAMESPACES*****************************//

namespace kvh_driver
{
using MatrixWrapper::Matrix;
using MatrixWrapper::ColumnVector;
using MatrixWrapper::SymmetricMatrix;
using namespace BFL;

class LinearFilter
{
public:
	/**
	 * @author Adam Panzica
	 * @brief Constructs a new IMU filter given an expected system noise and covariance and measurement noise and covariance
	 * @param [in] state_size The number of states in the system vector
	 * @param [in] input_size The number of inputs in the u vector
	 * @param [in] measurement_size The number of states in the measurement vector
	 * @param [in] sys_noise_mu The expected system noise
	 * @param [in] sys_noise_cov The expected system covariance
	 * @param [in] measurement_noise_mu The expected measurement noise
	 * @param [in] measurement_noise_cov The expected measurement covariance
	 */
	LinearFilter(int state_size, int input_size, int measurment_size, const ColumnVector& sys_noise_mu, const SymmetricMatrix& sys_noise_cov, const Matrix& A, const Matrix& B, const ColumnVector& measurement_noise_mu, const SymmetricMatrix& measurement_noise_cov, const Matrix& H);
	virtual ~LinearFilter();

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
	virtual bool init(const ColumnVector& initial_state, const SymmetricMatrix& initial_covar);

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
	virtual bool update(const ColumnVector& input, const ColumnVector& measurement);

	/**
	 * @author Adam Panzica
	 * @brief Updates the filter based purely on the system model
	 * @return True if the filter successfully is updated
	 *
	 * This is intended to be used if the sensor missed an update cycle before time ran out. It moves the filter along based
	 * purely on the system model
	 */
	virtual bool update();

	/**
	 * @author Adam Panzica
	 * @brief Gets the latest state estimate from the filter
	 * @param [out] estimate Vector to write the state estimate to
	 * @param [out] covar Matrix to write the state estimate covariance to
	 * @return True if the estimate was sucessfully retrieved, else false
	 *
	 * Note that the init method must have been called prior to getting estimates
	 */
	virtual bool getEstimate(ColumnVector& estimate, SymmetricMatrix& covar) const;

	/**
	 * @author Adam Panzica
	 * @return true if the filter is intialized, else false
	 */
	virtual bool isInitialized();

protected:

	/**
	 * Number of states in the filter
	 */
	int state_size_;

	/**
	 * Number of inputs in the filter
	 */
	int input_size_;

	/**
	 * Number of measurements in the filter
	 */
	int measurement_size_;

	/**
	 * The combined system evolution dynamics
	 */
	std::vector<Matrix>                                AB_;
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

}; /*END kvh_driver*/


#endif /* LINEARFILTER_H_ */
