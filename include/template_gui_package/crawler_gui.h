#ifndef CRAWLER_GUI_H
#define CRAWLER_GUI_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QProgressBar>
#include <ros/ros.h>
#include <qtimer.h>
#include <sensor_msgs/Image.h>
#include <template_gui_package/MotorPosition.h>
#include <template_gui_package/MotorSpeed.h>
#include <template_gui_package/Buttons.h>
#include <template_gui_package/GoToInfo.h>
#include <template_gui_package/MotorVoltage.h>
#include <template_gui_package/MotorPower.h>
#include <template_gui_package/MotorCurrent.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "include/template_gui_package/backup_controller.h"
#include "include/template_gui_package/omni_gui.h"


namespace Ui {
class CrawlerGui;
}

class CrawlerGui : public QWidget
{
  Q_OBJECT

public:
  explicit CrawlerGui(QWidget *parent = nullptr);
  ~CrawlerGui();
  void onBladeLengthEntered();
  void onGoToEntered();
  void onGoToThresholdEntered();
  void onWheelDiameterEntered();
  void onCcSpeedEntered();
  void onJogIncrementEntered();
  void posCallBack(const template_gui_package::MotorPosition::ConstPtr& msg);
  // void posCallBack(const nav_msgs::Odometry::ConstPtr& msg);
  void speedCallBack(const template_gui_package::MotorSpeed::ConstPtr& msg);
  void imageCallBack(const sensor_msgs::Image::ConstPtr& msg);
  void pwmCallBack(const std_msgs::String::ConstPtr& msg);
  void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
  void voltageCallBack(const template_gui_package::MotorVoltage::ConstPtr& msg);
  void powerCallBack(const template_gui_package::MotorPower::ConstPtr& msg);
  void currentCallBack(const template_gui_package::MotorCurrent::ConstPtr& msg);
  void onSliderValueChanged(int value);

public slots:
  void spinOnce();
  void onZeroButtonPress();
  void onStopGoToButtonPress();
  void onControllerButtonPress();
  void onOmniControllerButtonPress();

private:
  Ui::CrawlerGui *ui;

  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;
  ros::Subscriber pos_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber image_sub_;
  ros::Publisher zero_pub_;
  ros::Subscriber pwm_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber volt_sub_;
  ros::Subscriber pow_sub_;
  ros::Subscriber cur_sub_;
  ros::Publisher pwm_pub_;
  ros::Publisher goto_pub_;

  QGraphicsScene *scene_;
  QGraphicsScene *roll_scene_;
  QGraphicsScene *pitch_scene_;
  QGraphicsScene *plan_scene_;

  QGraphicsPixmapItem *rollItem_;
  QGraphicsPixmapItem *pitchItem_;
  QGraphicsPixmapItem *planItem_;

  double roll_angle = 0.0;
  double pitch_angle = 0.0;

  double previous_smoothed_roll;
  double previous_smoothed_pitch;

  QProgressBar *progressBar_;
  QProgressBar *progressBarTwo_;
  QProgressBar *progressBarFour_;
  QProgressBar *battery_level_;

  BackupController *backupController;

  OmniGui *omniController;
};

#endif // CRAWLER_GUI_H
