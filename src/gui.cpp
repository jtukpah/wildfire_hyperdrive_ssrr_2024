//TODO: Tested with multiple waypoints, did not plan for all waypoints, it also moved the waypoints on the multiple waypoint plan but not on the single waypoint plan.
#ifndef GUI_H_
#define GUI_H_

#include <vector>
#include <time.h>
#include <chrono>
#include <thread>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "imec_driver/GUI.h"
#include <pluginlib/class_list_macros.h>

namespace imec_driver
{
GUI::GUI( QWidget* parent )
  : rviz::Panel( parent )
{

  spinner_ptr = new ros::AsyncSpinner(2);
  spinner_ptr->start();

  QGridLayout *layout = new QGridLayout;

  QLabel *title = new QLabel("<P><b><FONT SIZE = 5>Hyper Spectral Image</b></P></br>");
  layout->addWidget(title, 0, 0, 1, -1, Qt::AlignCenter);

  setLayout(layout);

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

PLUGINLIB_EXPORT_CLASS(imec_driver::GUI, rviz::Panel)