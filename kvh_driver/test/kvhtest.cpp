/**
 * @file kvhtest.cpp
 *
 * @date   Feb 23, 2013
 * @author parallels
 * @brief \todo
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
#include<gtest/gtest.h>
#include<ros/ros.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<kvh_driver/imu_filter.h>
#include<kvh_driver/linearfilter.h>
#include<kvh_driver/odometryfilter.hpp>
//**********************NAMESPACES*****************************//

class FilterHarness: public ::testing::Test
{
protected:

	virtual void SetUp()
	{
		this->setUpLinearFilter();
	}

	void setUpLinearFilter()
	{
		BFL::ColumnVector    linear_sys_noise(2);
		BFL::SymmetricMatrix linear_sys_covar(2);
		linear_sys_noise(1)   = 1;
		linear_sys_noise(2)   = 1;
		linear_sys_covar      = 0;
		linear_sys_covar(1,1) = 1;
		linear_sys_covar(2,2) = 1;

		BFL::Matrix lin_A(2,2);
		BFL::Matrix lin_B(2,2);
		lin_A(1,1) = 2;
		lin_A(1,2) = 2;
		lin_A(2,1) = .5;
		lin_A(2,2) = .5;
		lin_B(1,1) = 1;
		lin_B(1,2) = 0;
		lin_B(2,1) = 0;
		lin_B(2,2) = 1;

		BFL::ColumnVector    lin_meas_noise(2);
		BFL::SymmetricMatrix lin_meas_covar(2);
		BFL::Matrix lin_H(2,2);
		lin_meas_noise(1)   = 1;
		lin_meas_noise(2)   = 1;
		lin_meas_covar      = 0;
		lin_meas_covar(1,1) = 1;
		lin_meas_covar(2,2) = 1;
		lin_H(1,1) = 1;
		lin_H(1,2) = 0;
		lin_H(2,1) = 0;
		lin_H(2,2) = 1;

		BFL::ColumnVector lin_initial_state(linear_sys_noise.size());
		BFL::SymmetricMatrix lin_initial_covar(linear_sys_covar);
		lin_initial_state = 0;

		this->lin_initial_state_ = lin_initial_state;
		this->lin_initial_covar_ = lin_initial_covar;
		this->testLF_ = new kvh_driver::LinearFilter(lin_A.size2(), lin_B.size2(), lin_H.size2(), linear_sys_noise, linear_sys_covar, lin_A, lin_B, lin_meas_noise, lin_meas_covar, lin_H);
		this->testLF_->init(lin_initial_state, lin_initial_covar);
	}

	virtual void TearDown()
	{
		delete testLF_;
		delete testIF_;
		delete testOF_;
	}

	kvh_driver::LinearFilter*   testLF_;
	BFL::ColumnVector           lin_initial_state_;
	BFL::SymmetricMatrix        lin_initial_covar_;
	kvh_driver::IMUFilter*      testIF_;
	kvh_driver::OdometryFilter* testOF_;
};

TEST(LinearFilterTest, perSamplePerSecondTest)
{
	double startValue = 1;
	ros::Duration sample_time(0.1);
	double endValue =  kvh_driver::LinearFilter::perSecToPerSample(sample_time, startValue);
	ASSERT_EQ(endValue, .1)<<"The Converted Value in units/sample did not match the expected value";
}

//TEST_F(FilterHarness, setupTest)
//{
//	ASSERT_TRUE(testLF_->isInitialized()) << "LinearFilter did not initialize properly!";
//
//	BFL::ColumnVector    test_lin_state(lin_initial_state_.size());
//	BFL::SymmetricMatrix test_lin_covar(lin_initial_covar_.size1());
//	testLF_->getEstimate(test_lin_state,test_lin_covar);
//	ASSERT_EQ(test_lin_state, lin_initial_state_);
//	ASSERT_EQ(test_lin_covar, lin_initial_covar_);
//
//}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




