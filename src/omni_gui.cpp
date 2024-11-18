#include "include/template_gui_package/omni_gui.h"
#include "ui_omni_gui.h"

bool sendToFrontRight = false;
bool sendToFrontLeft = false;
bool sendToBackRight = false;
bool sendToBackLeft = false;

QString command;

int movement_speed = 45;

bool forward = false;
bool backward = false;
bool right = false;
bool left = false;

bool forward_right = false;
bool forward_left = false;
bool backward_right = false;
bool backward_left = false;

bool turning_right = false;
bool turning_left = false;
bool curved_trajectory_right = false;
bool curved_trajectory_left = false;
bool lateral_arc_right = false;
bool lateral_arc_left = false;

/*
144---141
    |
    |
143---142
*/

OmniGui::OmniGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::OmniGui),
  scene_(new QGraphicsScene(this))
{
  ui->setupUi(this);
  setWindowTitle("Omnidirectional Wheel Controller GUI");

  nh_.reset(new ros::NodeHandle("~"));

  // Setup the timer that will signal ros stuff to happen,
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));

  // Set the rate to 100ms,
  ros_timer->start(100);

  connect(ui->forwardButton, &QPushButton::clicked, this, &OmniGui::onForwardButtonPress);
  connect(ui->backwardButton, &QPushButton::clicked, this, &OmniGui::onBackwardButtonPress);
  connect(ui->leftButton, &QPushButton::clicked, this, &OmniGui::onLeftButtonPress);
  connect(ui->rightButton, &QPushButton::clicked, this, &OmniGui::onRightButtonPress);

  connect(ui->forwardRightButton, &QPushButton::clicked, this, &OmniGui::onForwardRightButtonPress);
  connect(ui->forwardLeftButton, &QPushButton::clicked, this, &OmniGui::onForwardLeftButtonPress);
  connect(ui->backwardRightButton, &QPushButton::clicked, this, &OmniGui::onBackwardRightButtonPress);
  connect(ui->backwardLeftButton, &QPushButton::clicked, this, &OmniGui::onBackwardLeftButtonPress);

  connect(ui->turningRightButton, &QPushButton::clicked, this, &OmniGui::onTurningRightButtonPress);
  connect(ui->turningLeftButton, &QPushButton::clicked, this, &OmniGui::onTurningLeftButtonPress);
  connect(ui->curvedTrajectoryRightButton, &QPushButton::clicked, this, &OmniGui::onCurvedTrajectoryRightButtonPress);
  connect(ui->curvedTrajectoryLeftButton, &QPushButton::clicked, this, &OmniGui::onCurvedTrajectoryLeftButtonPress);
  connect(ui->lateralArcRightButton, &QPushButton::clicked, this, &OmniGui::onLateralArcRightButtonPress);
  connect(ui->lateralArcLeftButton, &QPushButton::clicked, this, &OmniGui::onLateralArcLeftButtonPress);

  connect(ui->sendCommandButton, &QPushButton::clicked, this, &OmniGui::onSendCommandButtonPress);

  connect(ui->frontRightCheck, &QCheckBox::clicked, this, &OmniGui::onFrontRightChecked);
  connect(ui->frontLeftCheck, &QCheckBox::clicked, this, &OmniGui::onFrontLeftChecked);
  connect(ui->backRightCheck, &QCheckBox::clicked, this, &OmniGui::onBackRightChecked);
  connect(ui->backLeftCheck, &QCheckBox::clicked, this, &OmniGui::onBackLeftChecked);

  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/forward.png", 8);

  command_pub_ = nh_->advertise<std_msgs::String>("/motor_commands", 4);

  connect(ui->commandEnter, &QLineEdit::editingFinished, this, &OmniGui::onCommandEntered);

  connect(ui->speedEnter, &QLineEdit::editingFinished, this, &OmniGui::onSpeedEntered);
  ui->speedValue->setText(QString::number(movement_speed));
}

void OmniGui::changeGraphic(QString url, int scale)
{
  scene_->clear();

  QPixmap scenePixmap(url);

  scene_item_ = scene_->addPixmap(scenePixmap);
  scene_item_->setTransformOriginPoint(scenePixmap.width() / 2, scenePixmap.height() / 2);

  // Set the scene to plan_view
  ui->commandView->setScene(scene_);

  // Fit the image in the view, keeping the aspect ratio
  ui->commandView->fitInView(scene_item_, Qt::KeepAspectRatio);
  scene_item_->setScale(scene_item_->scale() * scale);
  ui->commandView->show();
  ui->commandView->update();
}

std::string OmniGui::convertSpeed(int rpm)
{
  // Step 1: Convert RPM to degrees per second (dps)
  int dps = rpm * 6;

  // Step 2: Convert dps to "0.01 dps/LSB" units (speedControl)
  int32_t speedControl = static_cast<int32_t>(dps / 0.01);

  // Step 3: Break speedControl into bytes for the data field
  uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00,
                     static_cast<uint8_t>(speedControl),
                     static_cast<uint8_t>(speedControl >> 8),
                     static_cast<uint8_t>(speedControl >> 16),
                     static_cast<uint8_t>(speedControl >> 24)};

  // Step 4: Convert data array to a hex string
  std::ostringstream oss;
  for (int i = 0; i < 8; i++)
  {
      oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
  }

  return oss.str();
}

OmniGui::~OmniGui()
{
  delete ui;
}

void OmniGui::closeEvent(QCloseEvent *event) {
  // Perform actions before the GUI is destroyed
  ros::param::set("/omni_controller_active", false);

  // Allow the window to close
  event->accept();
}


void OmniGui::onForwardButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/forward.png", 1);

  std_msgs::String msg;

  if (forward)
  {
    forward = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    forward = true;

    msg.data = "cansend can0 141#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
  }
}

void OmniGui::onBackwardButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/backward.png", 1);

  std_msgs::String msg;

  if (backward)
  {
    backward = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    backward = true;

    msg.data = "cansend can0 141#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
  }
}

void OmniGui::onLeftButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/left.png", 1);

  std_msgs::String msg;

  if (left)
  {
    left = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    left = true;

    msg.data = "cansend can0 141#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
  }
}

void OmniGui::onRightButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/right.png", 1);

  std_msgs::String msg;

  if (right)
  {
    right = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    right = true;

    msg.data = "cansend can0 141#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
  }
}

void OmniGui::onForwardRightButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/forward-right.png", 1);

  std_msgs::String msg;

  if (forward_right)
  {
    forward_right = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    forward_right = true;

    msg.data = "cansend can0 141#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
  }
}

void OmniGui::onForwardLeftButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/forward-left.png", 1);

  std_msgs::String msg;

  if (forward_left)
  {
    forward_left = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    forward_left = true;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
}

void OmniGui::onBackwardRightButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/backward-right.png", 1);

  std_msgs::String msg;

  if (backward_right)
  {
    backward_right = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    backward_right = true;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
}

void OmniGui::onBackwardLeftButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/backward-left.png", 1);

  std_msgs::String msg;

  if (backward_left)
  {
    backward_left = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    backward_left = true;

    msg.data = "cansend can0 141#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
  }
}

void OmniGui::onTurningRightButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/turning-right.png", 1);

  std_msgs::String msg;

  if (turning_right)
  {
    turning_right = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    turning_right = true;

    msg.data = "cansend can0 141#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
  }
}

void OmniGui::onTurningLeftButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/turning-left.png", 1);

  std_msgs::String msg;

  if (turning_left)
  {
    turning_left = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    turning_left = true;

    msg.data = "cansend can0 141#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
  }
}

void OmniGui::onCurvedTrajectoryRightButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/curved-trajectory.png", 1);

  std_msgs::String msg;

  if (curved_trajectory_right)
  {
    curved_trajectory_right = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    curved_trajectory_right = true;

    msg.data = "cansend can0 141#" + convertSpeed(static_cast<int>(movement_speed / 2));
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed(static_cast<int>(movement_speed / 2));
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
  }
}

void OmniGui::onCurvedTrajectoryLeftButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/curved-trajectory-inv.png", 1);

  std_msgs::String msg;

  if (curved_trajectory_left)
  {
    curved_trajectory_left = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    curved_trajectory_left = true;

    msg.data = "cansend can0 141#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(static_cast<int>(movement_speed / 2));
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#" + convertSpeed(static_cast<int>(movement_speed / 2));
    command_pub_.publish(msg);
  }
}

void OmniGui::onLateralArcRightButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/lateral-arc-inv.png", 1);

  std_msgs::String msg;

  if (lateral_arc_right)
  {
    lateral_arc_right = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    lateral_arc_right = true;

    msg.data = "cansend can0 141#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
}

void OmniGui::onLateralArcLeftButtonPress()
{
  OmniGui::changeGraphic("/home/reece/catkin_ws/src/template_gui_package/Images/lateral-arc.png", 1);

  std_msgs::String msg;

  if (lateral_arc_left)
  {
    lateral_arc_left = false;

    msg.data = "cansend can0 141#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
  else
  {
    lateral_arc_left = true;

    msg.data = "cansend can0 141#" + convertSpeed((movement_speed * -1));
    command_pub_.publish(msg);
    msg.data = "cansend can0 144#" + convertSpeed(movement_speed);
    command_pub_.publish(msg);
    msg.data = "cansend can0 142#8100000000000000";
    command_pub_.publish(msg);
    msg.data = "cansend can0 143#8100000000000000";
    command_pub_.publish(msg);
  }
}

void OmniGui::onCommandEntered()
{
  command = ui->commandEnter->text();
}

void OmniGui::onSpeedEntered()
{
  QString speed = ui->speedEnter->text();
  movement_speed = speed.toInt();
  ui->speedValue->setText(QString::number(movement_speed));
}

void OmniGui::onSendCommandButtonPress()
{
  std_msgs::String msg;

  if (!command.isEmpty())
  {
    if (sendToFrontRight)
    {
      msg.data = "cansend can0 141#" + command.toStdString();
      command_pub_.publish(msg);
    }
    if (sendToFrontLeft)
    {
      msg.data = "cansend can0 144#" + command.toStdString();
      command_pub_.publish(msg);
    }
    if (sendToBackRight)
    {
      msg.data = "cansend can0 142#" + command.toStdString();
      command_pub_.publish(msg);
    }
    if (sendToBackLeft)
    {
      msg.data = "cansend can0 143#" + command.toStdString();
      command_pub_.publish(msg);
    }
  }
}

void OmniGui::onFrontRightChecked()
{
  if (sendToFrontRight)
  {
    sendToFrontRight = false;
  }
  else
  {
    sendToFrontRight = true;
  }
}

void OmniGui::onFrontLeftChecked()
{
  if (sendToFrontLeft)
  {
    sendToFrontLeft = false;
  }
  else
  {
    sendToFrontLeft = true;
  }
}

void OmniGui::onBackRightChecked()
{
  if (sendToBackRight)
  {
    sendToBackRight = false;
  }
  else
  {
    sendToBackRight = true;
  }
}

void OmniGui::onBackLeftChecked()
{
  if (sendToBackLeft)
  {
    sendToBackLeft = false;
  }
  else
  {
    sendToBackLeft = true;
  }
}

void OmniGui::spinOnce()
{
  if (ros::ok())
  {
    ros::spinOnce();
  }
  else
    QApplication::quit();
}
