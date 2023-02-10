#ifndef GUI_H
#define GUI_H

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <rviz/panel.h>
#endif

#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <Qt>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QObject>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QDebug>
#include <string>
#include <thread>
#include <QTimer>

namespace imec_driver
{

class GUI: public rviz::Panel
{
Q_OBJECT
public:
  GUI( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
  QGridLayout *layout;
  QLabel *title;
  QLabel *label_channel;
  QLabel *label_hist;
  QLabel *currentCube;
  QLabel *currentChannelValue;
  QHBoxLayout *image_layout;
  QHBoxLayout *controls_layout;
  QWidget* images;
  QWidget *controls;
  QComboBox *topicSelector;
  QSlider *channel_slider;
  QTimer* timer_;
  ros::Publisher channel_publisher;
  ros::Publisher camera_publisher;
  ros::Subscriber channel_listener;
  ros::Subscriber histogram_listener;
  void updateChannel(sensor_msgs::Image);
  void updateHistogram(sensor_msgs::Image);


public Q_SLOTS:
  void handleSliderEvent(int);
  void handleTopicChange(int);
  void test(std_msgs::Int8);
  void timerCallback();



Q_SIGNALS:


protected:
  ros::NodeHandle nh;

  ros::AsyncSpinner* spinner_ptr;


  
  // Text to inform the user the status of the planner
  QLabel* status_update_;

};

} // end namespace imec_driver

#endif // GUI_H