#ifndef BACKUP_CONTROLLER_H
#define BACKUP_CONTROLLER_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <template_gui_package/GoToInfo.h>
#include <QSlider>
#include <QCloseEvent>

namespace Ui {
class BackupController;
}

class BackupController : public QWidget
{
  Q_OBJECT

public:
  explicit BackupController(QWidget *parent = nullptr);
  void closeEvent(QCloseEvent*);
  ~BackupController();
  void gotoCallBack(const template_gui_package::GoToInfo::ConstPtr& msg);

public slots:
  void spinOnce();

  void onForwardValueChanged();
  void onForwardSliderReleased();

  void onReverseValueChanged();
  void onReverseSliderReleased();

  void onLeftValueChanged();
  void onLeftSliderReleased();

  void onRightValueChanged();
  void onRightSliderReleased();

  void onLightsButtonPress();
  void onJogButtonPress();
  void onForwardCcButtonPress();
  void onReverseCcButtonPress();

private:
  Ui::BackupController *ui;
  QTimer *ros_timer;
  ros::NodeHandlePtr nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher pwm_pub_;
  ros::Subscriber goto_sub_;
};

#endif // BACKUP_CONTROLLER_H
