#include "crawler_gui.h"
#include "ui_crawler_gui.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <tf/tf.h>
#include <cmath>

bool updating = false;
double motorPos[4] = {0.0, 0.0, 0.0, 0.0};
double wheelDiameterValue = 0.1005;
double goToValue = 0.0;
double bladeLengthValue = 75.0;
double distanceTraveled = 0.0;
int ccSpeedValue = 45.0;
double jogIncrementValue = 0.5;
double goToThresholdValue = 0.1;

CrawlerGui::CrawlerGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CrawlerGui),
  backupController(nullptr),
  omniController(nullptr),
  scene_(new QGraphicsScene(this)),
  roll_scene_(new QGraphicsScene(this)),
  pitch_scene_(new QGraphicsScene(this)),
  plan_scene_(new QGraphicsScene(this))
{
  ui->setupUi(this);

  nh_.reset(new ros::NodeHandle("~"));

  // Setup the timer that will signal ros stuff to happen,
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));

  // Set the rate to 100ms,
  ros_timer->start(100);

  std::string position_topic;
  nh_->param<std::string>("position_topic", position_topic, "/motor_position");
  pos_sub_ = nh_->subscribe<template_gui_package::MotorPosition>(position_topic, 8, &CrawlerGui::posCallBack, this);

  // std::string position_topic;
  // nh_->param<std::string>("position_topic", position_topic, "/odom");
  // pos_sub_ = nh_->subscribe<nav_msgs::Odometry>(position_topic, 4, &CrawlerGui::posCallBack, this);

  std::string speed_topic;
  nh_->param<std::string>("speed_topic", speed_topic, "/motor_speed");
  speed_sub_ = nh_->subscribe<template_gui_package::MotorSpeed>(speed_topic, 4, &CrawlerGui::speedCallBack, this);

  std::string camera_topic;
  nh_->param<std::string>("camera_topic", camera_topic, "/d400_camera/camera/color/image_raw");
  image_sub_ = nh_->subscribe<sensor_msgs::Image>(camera_topic, 100, &CrawlerGui::imageCallBack, this);

  std::string led_topic;
  nh_->param<std::string>("led_topic", led_topic, "/led_pwm");
  pwm_sub_ = nh_->subscribe<std_msgs::String>(led_topic, 2, &CrawlerGui::pwmCallBack, this);

  std::string odom_topic;
  nh_->param<std::string>("odom_topic", odom_topic, "/t265_camera/camera/odom/sample");
  odom_sub_ = nh_->subscribe<nav_msgs::Odometry>(odom_topic, 2, &CrawlerGui::odomCallBack, this);

  std::string voltage_topic;
  nh_->param<std::string>("voltage_topic", voltage_topic, "/motor_voltage");
  volt_sub_ = nh_->subscribe<template_gui_package::MotorVoltage>(voltage_topic, 4, &CrawlerGui::voltageCallBack, this);

  std::string power_topic;
  nh_->param<std::string>("power_topic", power_topic, "/motor_power");
  pow_sub_ = nh_->subscribe<template_gui_package::MotorPower>(power_topic, 4, &CrawlerGui::powerCallBack, this);

  std::string current_topic;
  nh_->param<std::string>("current_topic", current_topic, "/motor_current");
  cur_sub_ = nh_->subscribe<template_gui_package::MotorCurrent>(current_topic, 4, &CrawlerGui::currentCallBack, this);

  zero_pub_ = nh_->advertise<template_gui_package::Buttons>("/gui_input", 1);
  connect(ui->zero, &QPushButton::clicked, this, &CrawlerGui::onZeroButtonPress);

  connect(ui->stop_go_to, &QPushButton::clicked, this, &CrawlerGui::onStopGoToButtonPress);

  connect(ui->go_to, &QLineEdit::editingFinished, this, &CrawlerGui::onGoToEntered);
  connect(ui->go_to_threshold, &QLineEdit::editingFinished, this, &CrawlerGui::onGoToThresholdEntered);
  connect(ui->wheel_diameter, &QLineEdit::editingFinished, this, &CrawlerGui::onWheelDiameterEntered);
  connect(ui->blade_length, &QLineEdit::editingFinished, this, &CrawlerGui::onBladeLengthEntered);
  connect(ui->cc_speed, &QLineEdit::editingFinished, this, &CrawlerGui::onCcSpeedEntered);
  connect(ui->jog_increment, &QLineEdit::editingFinished, this, &CrawlerGui::onJogIncrementEntered);

  ui->go_to_value->setText(QString::number(goToValue));
  ros::param::set("goto_enabled", false);
  goto_pub_ = nh_->advertise<template_gui_package::GoToInfo>("/goto_status", 1);

  ui->go_to_threshold_value->setText(QString::number(goToThresholdValue));

  ui->wheel_diameter_value->setText(QString::number(wheelDiameterValue));
  ui->blade_length_value->setText(QString::number(bladeLengthValue));

  ui->cc_speed_value->setText(QString::number(ccSpeedValue));
  ros::param::set("cc_speed", ccSpeedValue);

  ui->jog_increment_value->setText(QString::number(jogIncrementValue));
  ros::param::set("jog_increment", jogIncrementValue);

  // ui->position_value->setText(QString::number(distanceTraveled));

  ui->graphicsView->setScene(scene_);

  // Setup the Roll Visualiser -----------------------------------------------------------------------------------,
  QPixmap rollPixmap("/home/reece/catkin_ws/src/template_gui_package/Images/Blade Runner - Rear View 1.jpg");

  rollItem_ = roll_scene_->addPixmap(rollPixmap);
  rollItem_->setTransformOriginPoint(rollPixmap.width() / 2, rollPixmap.height() / 2);

  // Set the scene to roll_view
  ui->roll_view->setScene(roll_scene_);

  // Fit the image in the view, keeping the aspect ratio
  ui->roll_view->fitInView(rollItem_, Qt::KeepAspectRatio);
  rollItem_->setScale(rollItem_->scale() * 6);  // Increase scale by 1.5x
  ui->roll_view->show();
  ui->roll_view->update();

  // Setup the Pitch Visualiser -----------------------------------------------------------------------------------,
  QPixmap pitchPixmap("/home/reece/catkin_ws/src/template_gui_package/Images/Blade Runner - Side View 1.jpg");

  pitchItem_ = pitch_scene_->addPixmap(pitchPixmap);
  pitchItem_->setTransformOriginPoint(pitchPixmap.width() / 2, pitchPixmap.height() / 2);

  // Set the scene to pitch_view
  ui->pitch_view->setScene(pitch_scene_);

  // Fit the image in the view, keeping the aspect ratio
  ui->pitch_view->fitInView(pitchItem_, Qt::KeepAspectRatio);
  pitchItem_->setScale(pitchItem_->scale() * 6);  // Increase scale by 1.5x
  ui->pitch_view->show();
  ui->pitch_view->update();

  // Setup the Plan Visualiser ------------------------------------------------------------------------------------,
  QPixmap planPixmap("/home/reece/catkin_ws/src/template_gui_package/Images/Blade Runner - Plan View.jpg");

  planItem_ = plan_scene_->addPixmap(planPixmap);
  planItem_->setTransformOriginPoint(planPixmap.width() / 2, planPixmap.height() / 2);

  // Set the scene to plan_view
  ui->plan_view->setScene(plan_scene_);

  // Fit the image in the view, keeping the aspect ratio
  ui->plan_view->fitInView(planItem_, Qt::KeepAspectRatio);
  planItem_->setScale(planItem_->scale() * 5);  // Increase scale by 1.5x
  planItem_->setRotation(90);
  ui->plan_view->show();
  ui->plan_view->update();

  progressBar_ = ui->progressBar;
  progressBarTwo_ = ui->progressBar_2;
  progressBarFour_ = ui->progressBar_4;
  battery_level_ = ui->battery_level;

  pwm_pub_ = nh_->advertise<std_msgs::String>("/led_pwm", 2);
  connect(ui->frontLedSlider, &QSlider::valueChanged, this, &CrawlerGui::onSliderValueChanged);
  connect(ui->rightLedSlider, &QSlider::valueChanged, this, &CrawlerGui::onSliderValueChanged);
  connect(ui->leftLedSlider, &QSlider::valueChanged, this, &CrawlerGui::onSliderValueChanged);

  connect(ui->backup_controller, &QPushButton::clicked, this, &CrawlerGui::onControllerButtonPress);
  ros::param::set("/backup_controller_active", false);

  connect(ui->omni_controller, &QPushButton::clicked, this, &CrawlerGui::onOmniControllerButtonPress);
  ros::param::set("/omni_controller_active", false);
}

CrawlerGui::~CrawlerGui()
{
  delete ui;
  delete backupController;
  delete omniController;
}

void CrawlerGui::onBladeLengthEntered()
{
  QString bladeLengthText = ui->blade_length->text();

  if (!bladeLengthText.isEmpty()) {
      // Process the input from the go_to field
      bladeLengthValue = bladeLengthText.toDouble();

      // Perform any necessary action with goToValue
      progressBarTwo_->setMaximum(bladeLengthValue);
      progressBarFour_->setMaximum(bladeLengthValue);
      ui->blade_length_value->setText(QString::number(bladeLengthValue));
      ui->blade_length->clear();
  }
}

void CrawlerGui::onGoToEntered()
{
  QString goToText = ui->go_to->text();

  if (!goToText.isEmpty()) {
      // Process the input from the go_to field
      goToValue = goToText.toDouble();
      ros::param::set("/goto_enabled", true);

      // Perform any necessary action with goToValue
      ui->progressBar_4->setValue(goToValue);
      ui->go_to_value->setText(QString::number(goToValue));
      ui->go_to->clear();
  }
}

void CrawlerGui::onGoToThresholdEntered()
{
  QString goToThresholdText = ui->go_to_threshold->text();

  if (!goToThresholdText.isEmpty()) {
      // Process the input from the go_to field
      goToThresholdValue = goToThresholdText.toDouble();

      // Perform any necessary action with goToValue
      ui->go_to_threshold_value->setText(QString::number(goToThresholdValue));
      ui->go_to_threshold->clear();
  }
}

void CrawlerGui::onWheelDiameterEntered()
{
  QString wheelDiameterText = ui->wheel_diameter->text();

  if (!wheelDiameterText.isEmpty()) {
      // Process the input from the wheel_diameter field
      wheelDiameterValue = wheelDiameterText.toDouble();
      ui->wheel_diameter_value->setText(QString::number(wheelDiameterValue));
      ui->wheel_diameter->clear();
  }
}

void CrawlerGui::onCcSpeedEntered()
{
  QString ccSpeedText = ui->cc_speed->text();

  if (!ccSpeedText.isEmpty()) {
      // Process the input from the wheel_diameter field
      ccSpeedValue = ccSpeedText.toInt();
      ros::param::set("cc_speed", ccSpeedValue);
      ui->cc_speed_value->setText(QString::number(ccSpeedValue));
      ui->cc_speed->clear();
  }
}

void CrawlerGui::onJogIncrementEntered()
{
  QString jogIncrementText = ui->jog_increment->text();

  if (!jogIncrementText.isEmpty()) {
      // Process the input from the wheel_diameter field
      jogIncrementValue = jogIncrementText.toDouble();
      ros::param::set("/jog_increment", jogIncrementValue);
      ui->jog_increment_value->setText(QString::number(jogIncrementValue));
      ui->jog_increment->clear();
  }
}

void CrawlerGui::spinOnce()
{
  if (ros::ok())
  {
    ros::spinOnce();
  }
  else
    QApplication::quit();
}

void CrawlerGui::posCallBack(const template_gui_package::MotorPosition::ConstPtr &msg)
{
  std::string label_text =
                           std::string("CANID: ") + std::to_string(msg->motor_id) + "\n" +
                           std::string("Position: ") + std::to_string(msg->motor_pos) + " deg";

  if (msg->motor_id == 141)
  {
    ui->front_right_pos->setText(QString::fromStdString(label_text));
    motorPos[0] = (double)msg->motor_pos;
    //printf("motor pos")
  }
  else if (msg->motor_id == 142)
  {
    ui->back_right_pos->setText(QString::fromStdString(label_text));
    motorPos[1] = (double)msg->motor_pos;
  }
  else if (msg->motor_id == 143)
  {
    ui->back_left_pos->setText(QString::fromStdString(label_text));
    motorPos[2] = (double)msg->motor_pos;
  }
  else if (msg->motor_id == 144)
  {
    ui->front_left_pos->setText(QString::fromStdString(label_text));
    motorPos[3] = (double)msg->motor_pos;
  }

  double averageDistance = (motorPos[0] + motorPos[1] + motorPos[2] + motorPos[3]) / 4.0;
  distanceTraveled = ((averageDistance / 360.0) * (M_PI * wheelDiameterValue));
  //printf("Average Distance: %.2f\n", averageDistance);


  progressBarTwo_->setValue(distanceTraveled);
  ui->position_value->setText(QString::number(distanceTraveled, 'f', 2));

  template_gui_package::GoToInfo info;
  info.target_pos = goToValue;
  info.current_pos = distanceTraveled;
  info.target_pos_threshold = goToThresholdValue;
  goto_pub_.publish(info);
}

// void CrawlerGui::posCallBack(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   double averageDistance = msg->pose.pose.position.x;
//   distanceTraveled = averageDistance;

//   progressBarTwo_->setValue(distanceTraveled);
//   ui->position_value->setText(QString::number(distanceTraveled, 'f', 2));

//   template_gui_package::GoToInfo info;
//   info.target_pos = goToValue;
//   info.current_pos = distanceTraveled;
//   info.target_pos_threshold = goToThresholdValue;
//   goto_pub_.publish(info);
// }

void CrawlerGui::speedCallBack(const template_gui_package::MotorSpeed::ConstPtr &msg)
{
  std::string label_text_A = std::string("Speed: ") + std::to_string(msg->front_right_speed) + " RPM";
  std::string label_text_B = std::string("Speed: ") + std::to_string(msg->back_right_speed) + " RPM";
  std::string label_text_C = std::string("Speed: ") + std::to_string(msg->back_left_speed) + " RPM";
  std::string label_text_D = std::string("Speed: ") + std::to_string(msg->front_left_speed) + " RPM";

  ui->front_right_speed->setText(QString::fromStdString(label_text_A));
  ui->back_right_speed->setText(QString::fromStdString(label_text_B));
  ui->back_left_speed->setText(QString::fromStdString(label_text_C));
  ui->front_left_speed->setText(QString::fromStdString(label_text_D));

  int averageSpeed = (msg->front_right_speed + msg->back_right_speed + msg->back_left_speed + msg->front_left_speed) / 4;
  progressBar_->setValue(abs(averageSpeed));
}

void CrawlerGui::imageCallBack(const sensor_msgs::Image::ConstPtr &msg)
{
    try
    {
        // Convert the ROS image message to an OpenCV image (RGB format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // Convert the OpenCV image to QImage
        QImage qimage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);

        // Clear the scene and add the new image
        scene_->clear();
        scene_->addPixmap(QPixmap::fromImage(qimage));

        // Fit the image within the view
        ui->graphicsView->fitInView(scene_->itemsBoundingRect(), Qt::KeepAspectRatio);

        // Force the view to update
        ui->graphicsView->update();
        ui->graphicsView->show();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void CrawlerGui::pwmCallBack(const std_msgs::String::ConstPtr &msg)
{
  updating = true;

  // Get the incoming message as a QString
  QString pwmData = QString::fromStdString(msg->data);

  // Split the string by commas
  QStringList pwmValues = pwmData.split(',');

  // Ensure we have exactly 3 values (for left, right, and front LEDs)
  if (pwmValues.size() == 3) {
      // Convert the split segments to integers
      int leftValue = pwmValues[0].toInt();
      int rightValue = pwmValues[1].toInt();
      int frontValue = pwmValues[2].toInt();

      // Set the values for the respective QSliders
      ui->leftLedSlider->setValue(leftValue);
      ui->rightLedSlider->setValue(rightValue);
      ui->frontLedSlider->setValue(frontValue);
  }
  updating = false;
}

void CrawlerGui::odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Extract the angular velocities from the odometry message
  double angular_x = msg->pose.pose.orientation.x; // Roll rate
  double angular_y = msg->pose.pose.orientation.y; // Pitch rate

  // Scaling factor for better visualization
  double scaling_factor = 10.0; // Adjust this factor as necessary

  // Convert angular rates from radians to degrees
  double roll_angle = (angular_x * scaling_factor * (180.0 / M_PI)) * (-1);
  double pitch_angle = (angular_y * scaling_factor * (180.0 / M_PI)) * (-1);

  double corrected_roll_angle = roll_angle * 0.2222222222;
  double corrected_pitch_angle = pitch_angle * 0.2267002519;

  // Rotate the images
  rollItem_->setRotation(corrected_roll_angle);
  pitchItem_->setRotation(corrected_pitch_angle);

  // Update the QLabel text to display the roll and pitch angles with " deg"
  ui->roll_value->setText(QString::number(corrected_roll_angle, 'f', 2) + " deg");
  ui->pitch_value->setText(QString::number(corrected_pitch_angle, 'f', 2) + " deg");

  // Update the graphics views
  ui->roll_view->update();
  ui->pitch_view->update();
}

void CrawlerGui::voltageCallBack(const template_gui_package::MotorVoltage::ConstPtr &msg)
{
    if (msg->motor_id == 141)
    {
      ui->front_right_voltage->setText(QString::number(msg->motor_v) + " V");
      ros::param::set("/front_right_voltage", msg->motor_v);
    }
    else if (msg->motor_id == 142)
    {
      ui->back_right_voltage->setText(QString::number(msg->motor_v) + " V");
      ros::param::set("/back_right_voltage", msg->motor_v);
    }
    else if (msg->motor_id == 143)
    {
      ui->back_left_voltage->setText(QString::number(msg->motor_v) + " V");
      ros::param::set("/back_left_voltage", msg->motor_v);
    }
    else if (msg->motor_id == 144)
    {
      ui->front_left_voltage->setText(QString::number(msg->motor_v) + " V");
      ros::param::set("/front_left_voltage", msg->motor_v);
    }

    float fr_v_value;
    ros::param::get("/front_right_voltage", fr_v_value);

    float br_v_value;
    ros::param::get("/back_right_voltage", br_v_value);

    float fl_v_value;
    ros::param::get("/front_left_voltage", fl_v_value);

    float bl_v_value;
    ros::param::get("/back_right_voltage", bl_v_value);

    float average_v;
    average_v = (fr_v_value + br_v_value + fl_v_value + bl_v_value) / 4;
    ui->label_4->setText("Battery Voltage: " + QString::number(average_v) + " V");

    battery_level_->setValue((int)(average_v * 10));
}

void CrawlerGui::powerCallBack(const template_gui_package::MotorPower::ConstPtr &msg)
{
  if (msg->motor_id == 141)
    {
      ui->front_right_power->setText(QString::number(msg->motor_p) + " W");

    }
    else if (msg->motor_id == 142)
    {
      ui->back_right_power->setText(QString::number(msg->motor_p) + " W");

    }
    else if (msg->motor_id == 143)
    {
      ui->back_left_power->setText(QString::number(msg->motor_p) + " W");

    }
    else if (msg->motor_id == 144)
    {
      ui->front_left_power->setText(QString::number(msg->motor_p) + " W");

    }
}

void CrawlerGui::currentCallBack(const template_gui_package::MotorCurrent::ConstPtr &msg)
{
  if (msg->motor_id == 141)
    {
      ui->front_right_current->setText(QString::number(msg->motor_c, 'f', 2) + " A");

    }
    else if (msg->motor_id == 142)
    {
      ui->back_right_current->setText(QString::number(msg->motor_c, 'f', 2) + " A");

    }
    else if (msg->motor_id == 143)
    {
      ui->back_left_current->setText(QString::number(msg->motor_c, 'f', 2) + " A");

    }
    else if (msg->motor_id == 144)
    {
      ui->front_left_current->setText(QString::number(msg->motor_c, 'f', 2) + " A");

    }
}

void CrawlerGui::onZeroButtonPress()
{
    template_gui_package::Buttons msg;
    ui->position_value->setText("0");
    msg.zero = 1.0;
    zero_pub_.publish(msg);  // Publish the message when the button is pressed
}

void CrawlerGui::onStopGoToButtonPress()
{
  ros::param::set("goto_enabled", false);
  ui->progressBar_4->setValue(0);
}

void CrawlerGui::onSliderValueChanged(int value)
{
  if (updating != true)
  {
    // Read the current values from all three sliders
    int leftValue = ui->leftLedSlider->value();
    int rightValue = ui->rightLedSlider->value();
    int frontValue = ui->frontLedSlider->value();

    // Format the string as "Red,Green,Blue"
    std::string pwmString = std::to_string(leftValue) + "," +
                            std::to_string(rightValue) + "," +
                            std::to_string(frontValue);

    // Create a ROS String message
    std_msgs::String msg;
    msg.data = pwmString;

    // Publish the PWM command
    pwm_pub_.publish(msg);
  }
}

void CrawlerGui::onControllerButtonPress()
{
  ros::param::set("/backup_controller_active", true);

  // Create a new instance of BackupController
  backupController = new BackupController();

  // Show the BackupController window
  backupController->show();
}

void CrawlerGui::onOmniControllerButtonPress()
{
  ros::param::set("/omni_controller_active", true);

  // Create a new instance of BackupController
  omniController = new OmniGui();

  // Show the BackupController window
  omniController->show();
}
