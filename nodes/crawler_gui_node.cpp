#include <QApplication>
#include "crawler_gui.h"


int main(int argc, char *argv[])
{

  ros::init(argc, argv, "crawler_gui_node");
  QApplication a(argc, argv);

  CrawlerGui w;
  w.setWindowTitle(QString::fromStdString(
                     ros::this_node::getName()));
  w.show();
  return a.exec();
}
