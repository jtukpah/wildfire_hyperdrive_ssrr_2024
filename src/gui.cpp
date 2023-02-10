#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/core.hpp>
// #include <opencv2/imgcodes.hpp>
#include "imec_driver/GUI.h"


namespace imec_driver
{
GUI::GUI( QWidget* parent )
  : rviz::Panel( parent )
{
  channel_listener = nh.subscribe("/hsi_gui/channel_img", 1000, &GUI::updateChannel, this);
  histogram_listener = nh.subscribe("/hsi_gui/hist_img", 1000, &GUI::updateHistogram, this);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &GUI::timerCallback);
  timer_->start(10); // 1 second interval
  layout = new QGridLayout;

  title = new QLabel("<P><b><FONT SIZE = 5>Hyperspectral Cube Viewer</b></P></br>");
  layout->addWidget(title, 0, 0, 1, -1, Qt::AlignCenter);
  // Create hbox for images
  images = new QWidget();
  // Create placeholder for channel image
  image_layout = new QHBoxLayout(images);
  QImage image_channel(300, 300, QImage::Format_RGB888);
  image_channel.fill(Qt::white);
  label_channel = new QLabel();
  label_channel->setPixmap(QPixmap::fromImage(image_channel));
  image_layout->addWidget(label_channel);

  // Create placeholder for histogram image
  QImage image_hist(300, 300, QImage::Format_RGB888);
  image_hist.fill(Qt::black);
  label_hist = new QLabel();
  label_hist->setPixmap(QPixmap::fromImage(image_hist));
  image_layout->addWidget(label_hist);

  layout->addWidget(images);
  // Create hbox for controls
  controls = new QWidget();
  controls_layout = new QHBoxLayout(controls);
  /* 
  ========================================
  COMBOBOX FOR CAMERA SELECTION
  ========================================
  */
  topicSelector = new QComboBox();
  QStringList topics;
  topics << "[Camera]" << "ximea" << "imec" << "combined";
  topicSelector->addItems(topics);
  currentCube = new QLabel(tr("Current Cube:"));
  controls_layout->addWidget(currentCube);
  controls_layout->addWidget(topicSelector);
  /* 
  ========================================
  SLIDER FOR HSI CUBE CHANNEL SELECTION
  ========================================
  */
  channel_slider = new QSlider(Qt::Horizontal);
  channel_slider->setMinimum(0);
  channel_slider->setMaximum(24);
  channel_slider->setTickInterval(1);
  // Add Handler for Slider Change
  currentChannelValue = new QLabel(tr("Current \u039B:"));
  controls_layout->addWidget(currentChannelValue);
  controls_layout->addWidget(channel_slider);
  // Add the horizontal box
  layout->addWidget(controls);
  setLayout(layout);
  // Initiate publishers
  channel_publisher = nh.advertise<std_msgs::Int8>("/hsi_gui/channel", 10);
  camera_publisher = nh.advertise<std_msgs::String>("/hsi_gui/camera", 10);
  // Initiate event listeners
  QObject::connect(channel_slider, &QSlider::valueChanged, this, &GUI::handleSliderEvent);
  QObject::connect(topicSelector, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                    this, &GUI::handleTopicChange);
}

void GUI::handleSliderEvent(int value) {
  qDebug() << "Slider value changed to " << value;
  // Publish the numeric index to ROS
  if( ros::ok() && channel_publisher)
  {
    std_msgs::Int8 msg;
    msg.data = value;
    channel_publisher.publish(msg);
  }
}

void GUI::handleTopicChange(int index) {
  qDebug() << "Topic Index Change To" << index;
  // Publish the name of the currently selected camera to ROS
  if( ros::ok() && camera_publisher)
  {
    std_msgs::String msg;
    msg.data = topicSelector->currentText().toStdString();
    camera_publisher.publish(msg);
  }
}

void GUI::timerCallback()
{
  ros::spinOnce();
}

void GUI::updateHistogram(sensor_msgs::Image msg)
{
  qDebug() << "Histogram CALLBACK RECEIVED: ";
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Update the image in the UI
  label_hist->setPixmap(QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888)));
}

void GUI::updateChannel(sensor_msgs::Image msg)
{
  qDebug() << "Channel CALLBACK RECEIVED: ";
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  label_channel->setPixmap(QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888)));
}

void GUI::save( rviz::Config config ) const
{
  rviz::Panel::save(config);
}


void GUI::load( const rviz::Config& config )
{
  rviz::Panel::load(config);
}


} // end namespace imec_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(imec_driver::GUI, rviz::Panel)