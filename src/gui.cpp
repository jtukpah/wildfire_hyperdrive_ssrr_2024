//TODO: Tested with multiple waypoints, did not plan for all waypoints, it also moved the waypoints on the multiple waypoint plan but not on the single waypoint plan.

#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
#include "imec_driver/GUI.h"


namespace imec_driver
{
GUI::GUI( QWidget* parent )
  : rviz::Panel( parent )
{
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(imec_driver::GUI, rviz::Panel)