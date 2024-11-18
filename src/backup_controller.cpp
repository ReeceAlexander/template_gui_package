#include "include/template_gui_package/backup_controller.h"
#include "ui_backup_controller.h"

bool forward_cc_on = false;
bool reverse_cc_on = false;
bool led_on = false;
bool goto_in_progress = false;
bool jog_mode = false;
bool jog_start = false;
double jog_target = 0.0;

BackupController::BackupController(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::BackupController)
{
  ui->setupUi(this); // Set up the UI
  setWindowTitle("Virtual Controller GUI");

  nh_.reset(new ros::NodeHandle("~"));

  // Setup the timer that will signal ROS stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));

  // Set the rate to 100ms
  ros_timer->start(100);

  // Connect sliders to slots
  connect(ui->forward, &QSlider::valueChanged, this, &BackupController::onForwardValueChanged);
  connect(ui->forward, &QSlider::sliderReleased, this, &BackupController::onForwardSliderReleased);

  connect(ui->reverse, &QSlider::valueChanged, this, &BackupController::onReverseValueChanged);
  connect(ui->reverse, &QSlider::sliderReleased, this, &BackupController::onReverseSliderReleased);

  connect(ui->left, &QSlider::valueChanged, this, &BackupController::onLeftValueChanged);
  connect(ui->left, &QSlider::sliderReleased, this, &BackupController::onLeftSliderReleased);

  connect(ui->right, &QSlider::valueChanged, this, &BackupController::onRightValueChanged);
  connect(ui->right, &QSlider::sliderReleased, this, &BackupController::onRightSliderReleased);

  // Connect buttons to slots
  connect(ui->forward_cc, &QPushButton::clicked, this, &BackupController::onForwardCcButtonPress);
  connect(ui->reverse_cc, &QPushButton::clicked, this, &BackupController::onReverseCcButtonPress);
  connect(ui->lights, &QPushButton::clicked, this, &BackupController::onLightsButtonPress);
  connect(ui->jog_mode, &QPushButton::clicked, this, &BackupController::onJogButtonPress);

  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pwm_pub_ = nh_->advertise<std_msgs::String>("/led_pwm", 2);

  std::string goto_topic;
  nh_->param<std::string>("goto_topic", goto_topic, "/goto_status");
  goto_sub_ = nh_->subscribe<template_gui_package::GoToInfo>(goto_topic, 2, &BackupController::gotoCallBack, this);
}

BackupController::~BackupController()
{
  ros::param::set("/backup_controller_active", false);
  delete ui;
}

void BackupController::closeEvent(QCloseEvent *event) {
  // Perform actions before the GUI is destroyed
  ros::param::set("/backup_controller_active", false);

  // Allow the window to close
  event->accept();
}

// Define the button press slot functions
void BackupController::onForwardValueChanged()
{
  if (!forward_cc_on && !reverse_cc_on)
  {
    double value = ui->forward->value();

    geometry_msgs::Twist msg;
    msg.linear.x = value / 100;

    cmd_vel_pub_.publish(msg);
  }
}

void BackupController::onForwardSliderReleased()
{
  ui->forward->setValue(0);
}

void BackupController::onReverseValueChanged()
{
  if (!forward_cc_on && !reverse_cc_on)
  {
    double value = ui->reverse->value();

    geometry_msgs::Twist msg;
    msg.linear.x = (value / 100) * -1;

    cmd_vel_pub_.publish(msg);
  }
}

void BackupController::onReverseSliderReleased()
{
  ui->reverse->setValue(0);
}

void BackupController::onLeftValueChanged()
{
  double value = ui->left->value();

  geometry_msgs::Twist msg;
  msg.angular.z = (value / 100) * -1;

  cmd_vel_pub_.publish(msg);
}

void BackupController::onLeftSliderReleased()
{
  ui->left->setValue(0);

  if (forward_cc_on)
  {
    geometry_msgs::Twist msg;
    double value;
    ros::param::get("/cc_speed", value);
    msg.linear.x = value / 90;

    cmd_vel_pub_.publish(msg);
  }
  else if (reverse_cc_on)
  {
    geometry_msgs::Twist msg;
    double value;
    ros::param::get("/cc_speed", value);
    msg.linear.x = (value / 90) * -1;

    cmd_vel_pub_.publish(msg);
  }
}

void BackupController::onRightValueChanged()
{
  double value = ui->right->value();

  geometry_msgs::Twist msg;
  msg.angular.z = value / 100;

  cmd_vel_pub_.publish(msg);
}

void BackupController::onRightSliderReleased()
{
  ui->right->setValue(0);

  if (forward_cc_on)
  {
    geometry_msgs::Twist msg;
    double value;
    ros::param::get("/cc_speed", value);
    msg.linear.x = value / 90;

    cmd_vel_pub_.publish(msg);
  }
  else if (reverse_cc_on)
  {
    geometry_msgs::Twist msg;
    double value;
    ros::param::get("/cc_speed", value);
    msg.linear.x = (value / 90) * -1;

    cmd_vel_pub_.publish(msg);
  }
}

void BackupController::onForwardCcButtonPress()
{
  if (forward_cc_on)
  {
    forward_cc_on = false;

    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;

    cmd_vel_pub_.publish(msg);
  }
  else
  {
    forward_cc_on = true;

    geometry_msgs::Twist msg;
    double value;
    ros::param::get("/cc_speed", value);
    msg.linear.x = value / 90;

    cmd_vel_pub_.publish(msg);
  }
}

void BackupController::onReverseCcButtonPress()
{
  if (reverse_cc_on)
  {
    reverse_cc_on = false;

    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;

    cmd_vel_pub_.publish(msg);
  }
  else
  {
    reverse_cc_on = true;

    geometry_msgs::Twist msg;
    double value;
    ros::param::get("/cc_speed", value);
    msg.linear.x = (value / 90) * -1;

    cmd_vel_pub_.publish(msg);
  }
}

void BackupController::onLightsButtonPress()
{
  if (led_on)
  {
    led_on = false;

    std_msgs::String msg;
    msg.data = "0,0,0";

    pwm_pub_.publish(msg);
  }
  else
  {
    led_on = true;

    std_msgs::String msg;
    msg.data = "150,150,150";

    pwm_pub_.publish(msg);
  }
}

void BackupController::gotoCallBack(const template_gui_package::GoToInfo::ConstPtr &msg)
{
  bool value;
  ros::param::get("/goto_enabled", value);

  if (value)
  {
    goto_in_progress = true;
    double proximity = msg->current_pos - msg->target_pos;

    if (abs(proximity) <= msg->target_pos_threshold)
    {
      if (forward_cc_on)
      {
        forward_cc_on = false;
      }
      else if (reverse_cc_on)
      {
        reverse_cc_on = false;
      }

      ros::param::set("goto_enabled", false);
      goto_in_progress = false;

      geometry_msgs::Twist msg;
      msg.linear.x = 0.0;

      cmd_vel_pub_.publish(msg);
    }
    else if (msg->target_pos >= msg->current_pos)
    {
      forward_cc_on = true;

      geometry_msgs::Twist msg;
      double value;
      ros::param::get("/cc_speed", value);
      msg.linear.x = value / 90;

      cmd_vel_pub_.publish(msg);
    }
    else if (msg->target_pos <= msg->current_pos)
    {
      reverse_cc_on = true;

      geometry_msgs::Twist msg;
      double value;
      ros::param::get("/cc_speed", value);
      msg.linear.x = (value / 90) * -1;

      cmd_vel_pub_.publish(msg);
    }
  }
  else
  {
    if (goto_in_progress)
    {
      if (forward_cc_on)
      {
        forward_cc_on = false;
      }
      else if (reverse_cc_on)
      {
        reverse_cc_on = false;
      }

      geometry_msgs::Twist msg;
      msg.linear.x = 0.0;
      cmd_vel_pub_.publish(msg);

      goto_in_progress = false;
    }
  }

  if (jog_mode)
  {
    if (!jog_start)
    {
     jog_start = true;
     double value;
     ros::param::get("/jog_increment", value);
     jog_target = msg->current_pos + value;
    }

    double jog_proximity = msg->current_pos - jog_target;

    if (abs(jog_proximity) <= msg->target_pos_threshold)
    {
      if (forward_cc_on)
      {
        forward_cc_on = false;
      }
      else if (reverse_cc_on)
      {
        reverse_cc_on = false;
      }

      jog_mode = false;
      jog_start = false;

      geometry_msgs::Twist msg;
      msg.linear.x = 0.0;
      cmd_vel_pub_.publish(msg);
    }
    else if (jog_target >= msg->current_pos)
    {
      forward_cc_on = true;

      geometry_msgs::Twist msg;
      double value;
      ros::param::get("/cc_speed", value);
      msg.linear.x = value / 90;

      cmd_vel_pub_.publish(msg);
    }
    else if (jog_target <= msg->current_pos)
    {
      reverse_cc_on = true;

      geometry_msgs::Twist msg;
      double value;
      ros::param::get("/cc_speed", value);
      msg.linear.x = (value / 90) * -1;

      cmd_vel_pub_.publish(msg);
    }
  }
}

void BackupController::onJogButtonPress()
{
  if (jog_mode)
  {
    if (forward_cc_on)
    {
      forward_cc_on = false;
    }
    else if (reverse_cc_on)
    {
      reverse_cc_on = false;
    }

    jog_mode = false;
    jog_start = false;

    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    cmd_vel_pub_.publish(msg);
  }
  else
  {
    jog_mode = true;
  }
}

void BackupController::spinOnce()
{
  if (ros::ok())
  {
    ros::spinOnce();
  }
  else
    QApplication::quit();
}
