/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/rocon_qorchestra/main_window.hpp"
#include <ros/ros.h>
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);

  /*
   * Start splash screen to show off a bit
   */
  QPixmap pixmap(":/images/rocon-logo-blue-large.png");
  QSplashScreen splash(pixmap);
  splash.show();
  app.processEvents();
  ros::Time::init();
  ros::Duration(1.5).sleep(); // let's enjoy it a while

  rocon_qorchestra::MainWindow w(argc,argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  splash.finish(&w); // enough, let's close it again

  int result = app.exec();

	return result;
}
