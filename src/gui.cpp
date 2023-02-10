#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
// #include <opencv2/core.hpp>
// #include <opencv2/imgcodes.hpp>
#include "imec_driver/GUI.h"


namespace imec_driver
{
GUI::GUI( QWidget* parent )
  : rviz::Panel( parent )
{
  QGridLayout *layout = new QGridLayout;

  QLabel *title = new QLabel("<P><b><FONT SIZE = 5>Hyperspectral Cube Viewer</b></P></br>");
  layout->addWidget(title, 0, 0, 1, -1, Qt::AlignCenter);
  // Create hbox for images
  // Add channel image
  // QPixmap *channel_pix = new QPixmap(200,200);
  // channel_pix->fill();
  // QLabel *channelLabel = new QLabel();
  // channelLabel->setPixmap(channel_pix);
  // // Add histogram image
  // QPixmap *hist_pix = new QPixmap(200,200);
  // hist_pix->fill();
  // layout->addWidget(*channelLabel);
  // graphics_view->add
  // Create hbox for controls
  QWidget *controls = new QWidget;
  QHBoxLayout *controls_layout = new QHBoxLayout(controls);
  // Create and add topic selector
  QComboBox *topicSelector = new QComboBox();
  QStringList topics;
  topics << "[Camera]" << "ximea" << "imec" << "combined";
  topicSelector->addItems(topics);
  QLabel * currentCube = new QLabel(tr("Current Cube:"));
  controls_layout->addWidget(currentCube);
  controls_layout->addWidget(topicSelector);
  // Create the value slider
  QSlider* channel_slider = new QSlider(Qt::Horizontal);
  channel_slider->setMinimum(0);
  channel_slider->setMaximum(24);
  channel_slider->setTickInterval(1);
  // TODO Add Handler for Slider Change
  // connect(channel_slider, SIGNAL(valueChanged(int)), , SLOT(setValue(int)));
  QLabel * currentChannelValue = new QLabel(tr("Current \u039B:"));
  controls_layout->addWidget(currentChannelValue);
  controls_layout->addWidget(channel_slider);
  // Add the horizontal box
  layout->addWidget(controls);
  setLayout(layout);

}

// void GUI:handle_slider(QSlider:: slider, int value) {
// //   QObject::connect(&channel_slider, &QSlider::sliderMoved,
// // [&](int value) {
// // #if 0 // not so nice -> delayed
// //       qSlider.setToolTip(QString("%1").arg(value));
// // #else // better
// //       QToolTip::showText(QCursor::pos(), QString("%1").arg(value), nullptr);
// // #endif // 0
// //     });
// }

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