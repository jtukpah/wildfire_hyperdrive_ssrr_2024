/**
 * @file kvhtest.cpp
 *
 * @date   Feb 23, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<gtest/gtest.h>
#include<ros/ros.h>
#include<ros/package.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<kvh_driver/configurations.h>
#include<kvh_driver/constants.h>
//**********************NAMESPACES*****************************//


using MatrixWrapper::ColumnVector;
using MatrixWrapper::SymmetricMatrix;

class ConfigurationManagerHarness: public ::testing::Test
{
protected:

	virtual void SetUp()
	{
		std::string package_path = ros::package::getPath("kvh_driver");
		if(package_path.empty())
			FAIL()<<"Could not load kvh_driver configuration file";
		else{
			testCM_ = new kvh_driver::ConfigurationManager(package_path+"/test/testdevices.xml");
		}

	}

	virtual void TearDown()
	{
		delete testCM_;
	}

	kvh_driver::ConfigurationManager* testCM_;

};


TEST_F(ConfigurationManagerHarness, testLoadData)
{
	ColumnVector testVector1;
	testVector1.resize(5);
	ColumnVector testVector2;
	testVector2.resize(6);
	SymmetricMatrix testMatrix1;
	testMatrix1.resize(5);
	SymmetricMatrix testMatrix2;
	testMatrix2.resize(6);

	kvh_driver::ConfigurationData dataA = testCM_->GetConfiguration("CG-5100");
	ASSERT_TRUE(dataA.first)<<"Failed to load configuration";
	ASSERT_TRUE(dataA.second)<<"Failed to load calibration";
	ASSERT_EQ(115200, dataA.first->serial_baud_rate());
	ASSERT_EQ(kvh_driver::serial_parity_none, dataA.first->serial_parity());
	ASSERT_EQ(8, dataA.first->serial_data_bits());
	ASSERT_EQ(1, dataA.first->serial_stop_bits());
	ASSERT_FALSE(dataA.first->serial_flow_control());

	ASSERT_FALSE(dataA.second->noise(testVector1));
	ASSERT_TRUE(dataA.second->noise(testVector2));
	ASSERT_EQ(0.1, testVector2(kvh_driver::constants::IMU_X_DOT_DOT_STATE()));
	ASSERT_EQ(0.2, testVector2(kvh_driver::constants::IMU_Y_DOT_DOT_STATE()));
	ASSERT_EQ(0.3, testVector2(kvh_driver::constants::IMU_Z_DOT_DOT_STATE()));
	ASSERT_EQ(1.1, testVector2(kvh_driver::constants::IMU_RX_DOT_STATE()));
	ASSERT_EQ(1.2, testVector2(kvh_driver::constants::IMU_RY_DOT_STATE()));
	ASSERT_EQ(1.3, testVector2(kvh_driver::constants::IMU_RZ_DOT_STATE()));

	ASSERT_FALSE(dataA.second->covar(testMatrix1));
	ASSERT_TRUE(dataA.second->covar(testMatrix2));
	ASSERT_EQ(0.4, testMatrix2(kvh_driver::constants::IMU_X_DOT_DOT_STATE(), kvh_driver::constants::IMU_X_DOT_DOT_STATE()));
	ASSERT_EQ(0.5, testMatrix2(kvh_driver::constants::IMU_Y_DOT_DOT_STATE(), kvh_driver::constants::IMU_Y_DOT_DOT_STATE()));
	ASSERT_EQ(0.6, testMatrix2(kvh_driver::constants::IMU_Z_DOT_DOT_STATE(), kvh_driver::constants::IMU_Z_DOT_DOT_STATE()));
	ASSERT_EQ(1.4, testMatrix2(kvh_driver::constants::IMU_RX_DOT_STATE(), kvh_driver::constants::IMU_RX_DOT_STATE()));
	ASSERT_EQ(1.5, testMatrix2(kvh_driver::constants::IMU_RY_DOT_STATE(), kvh_driver::constants::IMU_RY_DOT_STATE()));
	ASSERT_EQ(1.6, testMatrix2(kvh_driver::constants::IMU_RZ_DOT_STATE(), kvh_driver::constants::IMU_RZ_DOT_STATE()));
	for(int i = 1; i<=kvh_driver::constants::IMU_STATE_SIZE(); ++i)
		for(int j = 1; j<=kvh_driver::constants::IMU_STATE_SIZE(); ++j)
			if(i!=j)
				ASSERT_EQ(0, testMatrix2(i, j));


	ColumnVector testVector3;
	testVector3.resize(0);
	ColumnVector testVector4;
	testVector4.resize(6);
	SymmetricMatrix testMatrix3;
	testMatrix3.resize(1);
	SymmetricMatrix testMatrix4;
	testMatrix4.resize(6);

	kvh_driver::ConfigurationData dataB = testCM_->GetConfiguration("DSP-1750");
	ASSERT_TRUE(dataB.first)<<"Failed to load configuration";
	ASSERT_TRUE(dataB.second)<<"Failed to load calibration";
	ASSERT_EQ(9600, dataB.first->serial_baud_rate());
	ASSERT_EQ(kvh_driver::serial_parity_odd, dataB.first->serial_parity());
	ASSERT_EQ(7, dataB.first->serial_data_bits());
	ASSERT_EQ(0, dataB.first->serial_stop_bits());
	ASSERT_TRUE(dataB.first->serial_flow_control());


	ASSERT_FALSE(dataB.second->noise(testVector3));
	ASSERT_TRUE(dataB.second->noise(testVector4));
	ASSERT_EQ(2.1, testVector4(kvh_driver::constants::IMU_X_DOT_DOT_STATE()));
	ASSERT_EQ(2.2, testVector4(kvh_driver::constants::IMU_Y_DOT_DOT_STATE()));
	ASSERT_EQ(2.3, testVector4(kvh_driver::constants::IMU_Z_DOT_DOT_STATE()));
	ASSERT_EQ(3.1, testVector4(kvh_driver::constants::IMU_RX_DOT_STATE()));
	ASSERT_EQ(3.2, testVector4(kvh_driver::constants::IMU_RY_DOT_STATE()));
	ASSERT_EQ(3.3, testVector4(kvh_driver::constants::IMU_RZ_DOT_STATE()));

	ASSERT_FALSE(dataB.second->covar(testMatrix3));
	ASSERT_TRUE(dataB.second->covar(testMatrix4));
	ASSERT_EQ(2.4, testMatrix4(kvh_driver::constants::IMU_X_DOT_DOT_STATE(), kvh_driver::constants::IMU_X_DOT_DOT_STATE()));
	ASSERT_EQ(2.5, testMatrix4(kvh_driver::constants::IMU_Y_DOT_DOT_STATE(), kvh_driver::constants::IMU_Y_DOT_DOT_STATE()));
	ASSERT_EQ(2.6, testMatrix4(kvh_driver::constants::IMU_Z_DOT_DOT_STATE(), kvh_driver::constants::IMU_Z_DOT_DOT_STATE()));
	ASSERT_EQ(3.4, testMatrix4(kvh_driver::constants::IMU_RX_DOT_STATE(), kvh_driver::constants::IMU_RX_DOT_STATE()));
	ASSERT_EQ(3.5, testMatrix4(kvh_driver::constants::IMU_RY_DOT_STATE(), kvh_driver::constants::IMU_RY_DOT_STATE()));
	ASSERT_EQ(3.6, testMatrix4(kvh_driver::constants::IMU_RZ_DOT_STATE(), kvh_driver::constants::IMU_RZ_DOT_STATE()));
	for(int i = 1; i<=kvh_driver::constants::IMU_STATE_SIZE(); ++i)
		for(int j = 1; j<=kvh_driver::constants::IMU_STATE_SIZE(); ++j)
			if(i!=j)
				ASSERT_EQ(0, testMatrix4(i, j));
}

TEST_F(ConfigurationManagerHarness, testBadConfigurationName)
{
	kvh_driver::ConfigurationData data = testCM_->GetConfiguration("ABadName");
	ASSERT_FALSE(data.first);
	ASSERT_FALSE(data.second);
}




