#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc,char *argv[]){
	ros::init(argc, argv, "gui_motor");
	ros::NodeHandle n;

	QApplication a(argc,argv);
	MainWindow w(n);
	TimerHandler tHandler(100);
	w.show();	 
	//ros::spin();
	return a.exec();
}


