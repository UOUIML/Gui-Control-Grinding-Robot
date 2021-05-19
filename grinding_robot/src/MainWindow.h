#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <qdebug.h>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include "MotorCmd.h"
#include "PIDControl.h"
#include "imageview.h"

#include "ros/ros.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Int32.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QTcpServer>
#include <QTcpSocket>
#include <QDebug>
#include <QtGui/QImage>
#include <QBuffer>
#include <QtGui/QImageWriter>

#include <mypcl/scanAction.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/publisher.h>
// //#include <pcl/point_cloud.h>
// //#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>




using namespace mypcl;
typedef actionlib::SimpleActionClient<mypcl::scanAction> scanClient;

#define DIRMOTOR1 -1
#define DIRMOTOR2 -1
#define DIRMOTOR3 -1
#define DIRSTEER -1
#define IDMOTOR2  5

#define LEFTSTEER -30
#define RIGHTSTREER 30

#define ROTATESTEER -60


enum CMD {SCAN_STATUS=1,
	SET_SCANROI=2,
	SET_POINTCLOUD_MOVE_XY=3,
	SET_CUR_PEAK_LASER=4,
	DRIVE_MOTOR_SPEED=5,
	STEER_MOTOR_SPEED=6,
	SET_LASER_TRACKING=7,
	LASER_ID_CODE=8,
	MOTOR_ID_CODE=9,
        STEER_MOTOR_ID=10,
        DRIVE_MOTOR_ID=11,
        STEER_MOTOR_WAIT=12};


class TimerHandler : public QObject{
	Q_OBJECT

	public:
		TimerHandler(int msec = 100){
			timer = new QTimer(this);

			connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));

			timer->start(msec);
		}
		QTimer *timer;

		public slots:
			void timerSlot(){
				ros::spinOnce();
			}
};

class MainWindow : public QMainWindow
{
	Q_OBJECT

	public:
		MainWindow(ros::NodeHandle &n, QWidget *parent = Q_NULLPTR);
		~MainWindow();

	protected:
		bool eventFilter(QObject *object, QEvent *event);
		QPoint _startPoint;
		QPoint _endPoint;

		public slots:

		void on_btn_forward_pressed();
		void on_btn_backward_pressed();
		void on_btn_left_pressed();
		void on_btn_right_pressed();
		void on_btn_stop_pressed();
                void on_btn_rotateLeft_pressed();
                void on_btn_rotateRight_pressed();
		void on_btn_DriveMotorStateOn_clicked();
		void on_btn_DriveMotorStateOff_clicked();
                void on_btn_SteerMotorSetOffEncode_clicked();


		void on_btn_forward_released();
		void on_btn_backward_released();
		void on_btn_left_released();
		void on_btn_right_released();
                void on_btn_rotateLeft_released();
                void on_btn_rotateRight_released();

		void on_btn_ConnectNetwork_clicked();
		void on_btn_NetworkCam_clicked();
		void on_btn_startTimer_clicked();
		void on_btn_stopTimer_clicked();

		// laser control
		void on_btn_SetCurPeakLaser_clicked();
		// drive motor control
		void on_driveMotorSpeed_valueChanged();
		void on_btn_DriveMotorSpeed_clicked();
		void on_btn_DriveMotorStop_clicked();
		// steer motor control
		void on_steerMotorAngle_valueChanged();
		void on_btn_SteerMotorAngle_clicked();
		void on_btn_SteerMotorSpeed_clicked();
		void on_steerMotorSpeed_valueChanged();
		void on_steerAngleTyping_valueChanged(int value);
		void on_btn_SteerAngleStop_clicked();
		// front scan
		void on_btn_FrontScanning_clicked();
		void on_btn_TrackingStart_clicked();
		void on_btn_TrackingStop_clicked();
		void on_btn_FrontJobForward_pressed();
		void on_btn_FrontJobBackward_pressed();
		void on_btn_FrontJobForward_released();
		void on_btn_FrontJobBackward_released();

		// back scan
		void on_btn_BackScanning_clicked();
		void on_btn_GrindingStart_clicked();
		void on_btn_GrindingStop_clicked();
		void on_btn_BackJobForward_pressed();
		void on_btn_BackJobForward_released();
		void on_btn_BackJobBackward_pressed();
		void on_btn_BackJobBackward_released();

		void run();
	private:
		void getListMotorData(int id,double value, MotorData& motorData,
			              std::vector<std::vector<byte>>& listMotordata);
		std::queue<QString> _sendData;
		void InitCanNetwork();
	private:
		Ui::MainWindowClass ui;
		ros::NodeHandle _nh;

		MotorCmd _motorCmd;
		QTimer *timer;

		// control laser tracking
                bool _isSendLaserTracking = false;
                bool _isSteerMotorReady = false;
		//bool _isSendMotor = false;
		int _oldPeakLaser = 9999;
		int _newPeakLaser = 0;
		// PID
		PIDControl *_pidControl;
		// Client
	public: 
		// Action for Laser
		void doneActionLaserScanCb(const actionlib::SimpleClientGoalState& state,
				           const scanResultConstPtr& result);
		void activeActionLaserScanCb();
		void feedbackActionLaserScanCb(const scanFeedbackConstPtr& feedback);

		// Action for Steer Motor	
		void doneActionSteerMotorCb(const actionlib::SimpleClientGoalState& state,
				            const scanResultConstPtr& result);
		void activeActionSteerMotorCb();
		void feedbackActionSteerMotorCb(const scanFeedbackConstPtr& feedback);


		// Action for Drive Motor	
		void doneActionDriveMotorCb(const actionlib::SimpleClientGoalState& state,
				            const scanResultConstPtr& result);
		void activeActionDriveMotorCb();
		void feedbackActionDriveMotorCb(const scanFeedbackConstPtr& feedback);

	private:
		scanClient _actionScan;
		std::queue<std::vector<byte>> actionLaserGoals_;

		scanClient _actionSteerMotor;
		std::queue<std::vector<byte>> actionSteerMotorGoals_;


		scanClient _actionDriveMotor;
		std::queue<std::vector<byte>> actionDriveMotorGoals_;
	private:
		image_transport::Subscriber imgSub_;
		image_transport::ImageTransport _imgNh;
                ros::Subscriber subIdPeak_;
                // 
                ros::Subscriber subPointCloud_;
                ros::Subscriber subfrontPointCloud_;
                ros::Subscriber subbackPointCloud_;

                ros::Publisher pubPointCloud_;
                sensor_msgs::PointCloud2 mergedCloud_;

		int frontScanCounter_ = 0;
		int backScanCounter_  = 0;
                
	public:
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
                void peakIdCallback(const std_msgs::Int32::ConstPtr& msg);
                void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

                void frontScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
                void backScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

	private:

		void sendActionLibLaserScan(std::vector<std::vector<byte>>& listMotorData);
		void sendActionLibDriveMotor(std::vector<std::vector<byte>>& listMotorData);
		void sendActionLibSteerMotor(std::vector<std::vector<byte>>& listMotorData);


};
