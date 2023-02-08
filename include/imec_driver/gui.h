#define GUI_H

#ifndef Q_MOC_RUN
  #include <ros/ros.h> //Comes from roscpp
  #include <rviz/panel.h>
#endif

#include <QGridLayout> //USed for the actual design of the panel
#include <QLabel> //Used for the title
#include <QSlider> //Used for changing the channel of the image
#include <QHBoxLayout> //Handle the orientation of the layout
#include <QVBoxLayout> //Handle the orientation of the layout
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QComboBox>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <thread>

namespace imec_driver
{

class GUI: public rviz::Panel
{
Q_OBJECT
public:
  GUI( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  
Q_SIGNALS:

protected:
  ros::NodeHandle nh_;

  ros::Publisher current_interactive_marker_pub_;
  ros::Publisher display_trajectory_pub_;
  ros::Publisher display_replay_pub_;
  ros::Publisher data_pub_;
  ros::Publisher execute_trajectory_pub_;
  ros::Publisher ghost_robot_visibility_pub_;
  ros::Publisher manipulation_pub_;
  ros::Publisher mid_manipulation_gripper_control_pub_;
  ros::Publisher moveit_path_pub;
  
  ros::Subscriber changed_interacive_marker_sub_;
  ros::Subscriber current_control_marker_request_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber moveit_path_sub;
  ros::Subscriber navigation_sub_;

  ros::AsyncSpinner* spinner_ptr;
};

} // end namespace fetch_rviz_interface

#endif
