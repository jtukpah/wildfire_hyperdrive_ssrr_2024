#ifndef GUI_H
#define GUI_H

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <rviz/panel.h>
#endif

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QObject>
#include <QPushButton>
#include <QRadioButton>

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

  ros::AsyncSpinner* spinner_ptr;


  
  // Text to inform the user the status of the planner
  QLabel* status_update_;

};

} // end namespace imec_driver

#endif // GUI_H