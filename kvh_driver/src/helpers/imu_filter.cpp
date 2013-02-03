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
								sys_model_(NULL),
								sys_pdf_(NULL),
								mes_model_(NULL),
								mes_pdf_(NULL),
								AB_(2),
								H_(NULL),
								filter_init_(false),
								filter_(NULL),
								prior_(NULL)
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

	//Build System PDF/Model
	Gaussian system_uncertainty(sys_noise_mu, sys_noise_cov);
	this->sys_pdf_   = new LinearAnalyticConditionalGaussian(this->AB_, system_uncertainty);
	this->sys_model_ = new LinearAnalyticSystemModelGaussianUncertainty(this->sys_pdf_);

	//Build H Matrix
	Matrix H(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
	H = 0;
	for (int r = 0; r < constants::ODOM_STATE_SIZE(); ++r)
	{
		H(r,r) = 1;
	}

	//Build Measurement PDF/Model
	Gaussian measurement_uncertainty(measurement_noise_mu, measurement_noise_cov);
	this->mes_pdf_   = new LinearAnalyticConditionalGaussian(H, measurement_uncertainty);
	this->mes_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(this->mes_pdf_);
}

IMUFilter::~IMUFilter()
{
	if(this->sys_model_!= NULL) delete this->sys_model_;
	if(this->sys_pdf_  != NULL) delete this->sys_pdf_;
	if(this->mes_model_!= NULL) delete this->mes_model_;
	if(this->mes_pdf_  != NULL) delete this->mes_pdf_;
	if(this->H_        != NULL) delete this->H_;
	if(this->filter_   != NULL) delete this->filter_;
	if(this->prior_    != NULL) delete this->prior_;
}

bool IMUFilter::init(const ColumnVector& initial_state, const SymmetricMatrix& initial_covar)
{
	//Check to make sure sizes match up
	if(initial_state.size() == constants::ODOM_STATE_SIZE() && initial_covar.size1()==constants::ODOM_STATE_SIZE())
	{
		this->prior_       = new Gaussian(initial_state, initial_covar);
		this->filter_      = new ExtendedKalmanFilter(this->prior_);
		this->filter_init_ = true;
		return true;
	}
	else
	{
		ROS_ERROR("Cannot Initialize IMU Filter with State/Covar size %d/%d, Expecting Size %d/%d", initial_state.size(), initial_covar.size1(), constants::ODOM_STATE_SIZE(), constants::ODOM_STATE_SIZE());
		return false;
	}
}

bool IMUFilter::update(const ColumnVector& input, const ColumnVector& measurement)
{
	if(this->filter_init_)
	{
		//Check to make sure sizes match up
		if(input.size() == constants::INPUT_SIZE() && measurement.size()==constants::MEASUREMENT_SIZE())
		{
			this->filter_->Update(this->sys_model_, input, this->mes_model_, measurement);
			this->prior_ = this->filter_->PostGet();
			return true;
		}
		else
		{
			ROS_ERROR("Cannot update IMU filter with input size %d and measurement size %d, expecting %d, %d", input.size(), measurement.size(), constants::INPUT_SIZE(), constants::MEASUREMENT_SIZE());
			return false;
		}
	}
	else
	{
		ROS_ERROR("Cannot perform IMU Filter update on uninitialized filter!!");
		return false;
	}
}

bool IMUFilter::getEstimate(ColumnVector& state_estimate, SymmetricMatrix& covar) const
{
	if(this->filter_init_)
	{
		state_estimate = this->prior_->ExpectedValueGet();
		covar          = this->prior_->CovarianceGet();
		return true;
	}
	else
	{
		ROS_ERROR("Cannot get estimate on uninitialized filter!!");
		return false;
	}
}


