#ifndef OMNI_GUI_H
#define OMNI_GUI_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <QCloseEvent>

namespace Ui {
class OmniGui;
}

class OmniGui : public QWidget
{
  Q_OBJECT

public:
  explicit OmniGui(QWidget *parent = nullptr);
  void changeGraphic(QString, int);
  std::string convertSpeed(int);
  void closeEvent(QCloseEvent*);
  ~OmniGui();
  void onCommandEntered();
  void onSpeedEntered();

  void onFrontRightChecked();
  void onFrontLeftChecked();
  void onBackRightChecked();
  void onBackLeftChecked();

public slots:
  void spinOnce();

  void onForwardButtonPress();
  void onBackwardButtonPress();
  void onLeftButtonPress();
  void onRightButtonPress();

  void onForwardRightButtonPress();
  void onForwardLeftButtonPress();
  void onBackwardRightButtonPress();
  void onBackwardLeftButtonPress();

  void onTurningRightButtonPress();
  void onTurningLeftButtonPress();
  void onCurvedTrajectoryRightButtonPress();
  void onCurvedTrajectoryLeftButtonPress();
  void onLateralArcRightButtonPress();
  void onLateralArcLeftButtonPress();

  void onSendCommandButtonPress();

private:
  Ui::OmniGui *ui;

  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;

  QGraphicsScene *scene_;
  QGraphicsPixmapItem *scene_item_;

  ros::Publisher command_pub_;
};

#endif // OMNI_GUI_H
