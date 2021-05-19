#include "MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle &n,QWidget *parent)
	: _nh(n),_imgNh(_nh),QMainWindow(parent),
	_actionScan("do_laser",true),
	_actionSteerMotor("do_steer_motor",true),
	_actionDriveMotor("do_drive_motor",true)
{
	ui.setupUi(this);
	ui.imgView->installEventFilter(this);

	_pidControl = new PIDControl(0.1, 90, -90, 0.09, 0.001, 0.001);

	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &MainWindow::run);

	// image subscribe
	imgSub_ = _imgNh.subscribe("laser/image",1, &MainWindow::imageCallback,this);
	subIdPeak_ = _nh.subscribe("laser/peak",1, &MainWindow::peakIdCallback,this);

	// point cloid sub
	subPointCloud_ = _nh.subscribe("points3", 1,&MainWindow::cloudCallback,this);
	pubPointCloud_ = _nh.advertise<sensor_msgs::PointCloud2> ("points4", 1);


	// front Scan callback
	subfrontPointCloud_ = _nh.subscribe("frontScanData", 1,&MainWindow::frontScanCallback,this);
	// back Scan callback	
	subbackPointCloud_ = _nh.subscribe("backScanData", 1,&MainWindow::backScanCallback,this);

	mergedCloud_.header.frame_id ="base_link";

}

void MainWindow::InitCanNetwork(){

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ENABLEMOSBUS;

	std::vector<std::vector<byte>> listMotordata;

	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, 0, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, 0, motorData, listMotordata);
	}
	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, 0, motorData, listMotordata);
	}

	// send action command steer motor
	this->sendActionLibSteerMotor(listMotordata);

	qDebug() << "Steer Motor Enable Can Network ";

}
// send action Laser Scan

void MainWindow::sendActionLibLaserScan(std::vector<std::vector<byte>>& listMotordata){

	// Clear old data
	actionLaserGoals_ = std::queue<std::vector<byte>>();
	// this is Laser Scan
	for (auto p : listMotordata) {
		QByteArray frame(reinterpret_cast<const char*>(p.data()), p.size()-1);
		ui.textBrowser->append(frame.toHex());
		// action client
		actionLaserGoals_.push(p);
	}
	// clear motor data
	listMotordata.clear();

mypcl:scanGoal goal;
      // Laser
      if(actionLaserGoals_.size() >0){

	      // clear old data
	      goal.motorCommand.clear();
	      goal.laserCommand.clear();
	      // get front goal
	      goal.IdCode = CMD::MOTOR_ID_CODE;
	      std::vector<byte> dataFrame =  actionLaserGoals_.front();
	      for(auto p : dataFrame){
		      goal.motorCommand.push_back(p);
	      }
	      _actionScan.sendGoal(goal,
			      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
			      scanClient::SimpleActiveCallback(),
			      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
	      // pop back
	      actionLaserGoals_.pop();
      }

}


// Send action Drive Motor
void MainWindow::sendActionLibDriveMotor(std::vector<std::vector<byte>>& listMotordata){

	// clear all data
	actionDriveMotorGoals_ = std::queue<std::vector<byte>>();

	// this is drive motor
	for (auto p : listMotordata) {
		QByteArray frame(reinterpret_cast<const char*>(p.data()), p.size()-1);
		ui.textBrowser->append(frame.toHex());
		// action client
		actionDriveMotorGoals_.push(p);
	}
	// clear motor data
	listMotordata.clear();

mypcl:scanGoal goal;
      // Laser
      if(actionDriveMotorGoals_.size() >0){

	      // clear old data
	      goal.motorCommand.clear();
	      goal.laserCommand.clear();
	      // get front goal
	      goal.IdCode = CMD::MOTOR_ID_CODE;
	      std::vector<byte> dataFrame =  actionDriveMotorGoals_.front();
	      for(auto p : dataFrame){
		      goal.motorCommand.push_back(p);
	      }
	      _actionDriveMotor.sendGoal(goal,
			      boost::bind(&MainWindow::doneActionDriveMotorCb,this,_1,_2),
			      scanClient::SimpleActiveCallback(),
			      boost::bind(&MainWindow::feedbackActionDriveMotorCb,this,_1));
	      // pop back
	      actionDriveMotorGoals_.pop();
      }

}

void MainWindow::sendActionLibSteerMotor(std::vector<std::vector<byte>>& listMotordata){

	// clear all data
	actionSteerMotorGoals_ = std::queue<std::vector<byte>>();

	// this is steer motor
	for (auto p : listMotordata) {
		QByteArray frame(reinterpret_cast<const char*>(p.data()), p.size()-1);
		ui.textBrowser->append(frame.toHex());
		// action client
		actionSteerMotorGoals_.push(p);
	}
	// clear motor data
	listMotordata.clear();
mypcl:scanGoal goal;

      // Steer Motor

      if(actionSteerMotorGoals_.size() >0){
	      // clear old data
	      goal.motorCommand.clear();
	      goal.laserCommand.clear();
	      // get front goal                     
	      goal.IdCode = CMD::MOTOR_ID_CODE;
	      std::vector<byte> dataFrame =  actionSteerMotorGoals_.front();
	      for(auto p : dataFrame){
		      goal.motorCommand.push_back(p);
	      }
	      _actionSteerMotor.sendGoal(goal,
			      boost::bind(&MainWindow::doneActionSteerMotorCb,this,_1,_2),
			      scanClient::SimpleActiveCallback(),
			      boost::bind(&MainWindow::feedbackActionSteerMotorCb,this,_1));
	      // pop back
	      actionSteerMotorGoals_.pop();
      }

}
void MainWindow::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

	//pcl::concatenatePointCloud(mergedCloud_,*msg,mergedCloud_);
	//pubPointCloud_.publish(mergedCloud_);
}

void MainWindow::frontScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

	std::cout<<"front scan Data rev"<<std::endl;
	std::string fileName = "frontscan_" + std::to_string(frontScanCounter_) + ".pcd";
	pcl::io::savePCDFile(fileName,*msg);

	frontScanCounter_ +=1;

	//pcl::concatenatePointCloud(mergedCloud_,*msg,mergedCloud_);
	//pubPointCloud_.publish(mergedCloud_);
}

void MainWindow::backScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

	std::cout<<"back scan Data rev"<<std::endl;

	std::string fileName = "backscan_" + std::to_string(backScanCounter_) + ".pcd";
	pcl::io::savePCDFile(fileName,*msg);

	backScanCounter_ +=1;

	//pcl::concatenatePointCloud(mergedCloud_,*msg,mergedCloud_);
	//pubPointCloud_.publish(mergedCloud_);
}

void MainWindow::peakIdCallback(const std_msgs::Int32::ConstPtr& msg){
	this-> _newPeakLaser = msg->data;

	// this is first peak

	if(_oldPeakLaser == 9999){
		_oldPeakLaser = msg->data;
	}

	if(this->_isSteerMotorReady == true){

		// reset flag motor
		this->_isSteerMotorReady = false;

		// insert negative direction for PID control

		double DIR_PID = 1;
		double currError = _pidControl->pidCalculate(_oldPeakLaser,_newPeakLaser)*DIR_PID;	
		// motor data convert
		ROS_INFO("Peak Id: %d, Error: %f",msg->data,currError);

		MotorData motorData;
		motorData.motor_type = MOTOR_TYPE::M1010;
		motorData.state_cmd = WRITE_CMD;
		motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


		std::vector<std::vector<byte>> listMotordata;

		// control only one steer motor
		if (ui.isSteerMotor1->isChecked()) {
			getListMotorData(1, currError, motorData, listMotordata);
		}

		// send action steer command
		this->sendActionLibSteerMotor(listMotordata);

	}

}

void MainWindow::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try
	{
		cv::Mat img;
		img = cv_bridge::toCvShare(msg, "mono8")->image;

		if(!img.empty()){
			ui.imgView->setImageMat(img);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


void MainWindow::getListMotorData(int id, double value, MotorData & motorData, std::vector<std::vector<byte>>& listMotordata)
{
	motorData.idMotor = id;
	motorData.value = value;
	int len = 0;
	std::vector<byte> dataFrame;
	bool status = _motorCmd.getFrameData(dataFrame, len, motorData);
	dataFrame.push_back(len);
	qDebug()<<"len: "<<len;
	// push last frame data	
	if(motorData.motor_type == MOTOR_TYPE::DRIVE_MOTOR){
		dataFrame.push_back(CMD::DRIVE_MOTOR_ID);
	}
	else{
		dataFrame.push_back(CMD::STEER_MOTOR_ID);
	}

	// save to list motor data
	listMotordata.push_back(dataFrame);
}

MainWindow::~MainWindow()
{
	//_cams->destroyCam();
}

bool MainWindow::eventFilter(QObject * object, QEvent * event)
{
	if (object != ui.imgView) return false;

	if (event->type() == QEvent::MouseMove) {
		//QMouseEvent* mouseEvent = static_cast<QMouseEvent *>(event);
		//qDebug()<<mouseEvent->pos();

	}
	else if (event->type() == QEvent::MouseButtonPress) {
		QMouseEvent* mouseEvent = static_cast<QMouseEvent *>(event);

		this->_startPoint = mouseEvent->pos();


	}
	else if (event->type() == QEvent::MouseButtonRelease) {
		QMouseEvent* mouseEvent = static_cast<QMouseEvent *>(event);
		this->_endPoint = mouseEvent->pos();

		if (_startPoint != _endPoint) {
			ui.imgView->DrawRect(_startPoint, _endPoint);

			// add to Roi Laser
			cv::Mat img;
			cv::Rect roi;
			if (ui.imgView->GetMatandRoi(img, roi)) {
			}
		}
	}

	return false;
}


void MainWindow::on_btn_forward_pressed()
{

	qDebug() << "forward pressed";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// this is steer motor
	for (auto p : listMotordata) {
		QByteArray frame(reinterpret_cast<const char*>(p.data()), p.size()-1);
		ui.textBrowser->append(frame.toHex());
		// action client
		actionSteerMotorGoals_.push(p);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = (double)ui.driveMotorSpeed->value();

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_backward_pressed()
{
	qDebug() << "backward pressed";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;



	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = -(double)ui.driveMotorSpeed->value();

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// convert to action messesge
	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_left_pressed()
{

	qDebug() << "left";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = LEFTSTEER;



	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = (double)ui.driveMotorSpeed->value();

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);
}

void MainWindow::on_btn_right_pressed()
{

	qDebug() << "right";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = RIGHTSTREER;



	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = (double)ui.driveMotorSpeed->value();

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_stop_pressed()
{

	qDebug() << "Stop motor";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;



	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::STOPMOTOR;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, angle*DIRSTEER, motorData, listMotordata);
	}
	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// convert to action messesge
	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);
}

void MainWindow::on_btn_rotateLeft_pressed()
{


	qDebug()<<"rotate left";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = ROTATESTEER;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, 60, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, -45, motorData, listMotordata);
	}
	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, 30, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);
	// control drive motor
	double speed = -(double)ui.driveMotorSpeed->value();

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, -speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, -speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, 1*speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_rotateRight_pressed()
{


	qDebug()<<"rotate right";
	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = ROTATESTEER;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, 60, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, -45, motorData, listMotordata);
	}
	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, 30, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = (double)ui.driveMotorSpeed->value();

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, -speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, -speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3,speed*DIRMOTOR3, motorData, listMotordata);
	}

	// convert to action messesge
	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);
}


void MainWindow::on_btn_DriveMotorStateOn_clicked(){

	qDebug()<<"Motor drive state on";

	ui.textBrowser->append("Motor drive state on");
	int IdCode = CMD::DRIVE_MOTOR_SPEED;
	double value = 0;

	//send data
	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::RUNMOTOR;

	std::vector<std::vector<byte>> listMotordata;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, value, motorData, listMotordata);
	}

	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, value, motorData, listMotordata);

	}


	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, value, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);
}

void MainWindow::on_btn_DriveMotorStateOff_clicked(){


	ui.textBrowser->append("Motor drive state off");
	int IdCode = CMD::DRIVE_MOTOR_SPEED;
	double value = 1;

	//send data
	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::STOPMOTOR;

	std::vector<std::vector<byte>> listMotordata;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, value, motorData, listMotordata);
	}

	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, value, motorData, listMotordata);

	}


	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, value, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_forward_released()
{

	qDebug() << "forward release";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;;

	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);
	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);


}

void MainWindow::on_btn_backward_released()
{


	qDebug() << "backward release";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;


	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_left_released()
{

	qDebug() << "left release";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// this is steer motor
	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);
	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// convert to action messesge
	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);
}

void MainWindow::on_btn_right_released()
{

	qDebug() << "right release";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;



	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);
}


void MainWindow::on_btn_rotateLeft_released()
{


	qDebug()<<"rotate left stop";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;


	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, angle*DIRSTEER, motorData, listMotordata);
	}

	// this is steer motor
	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);
	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// convert to action messesge
	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_rotateRight_released()
{

	qDebug()<<"rotate right stop";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;



	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, angle*DIRSTEER, motorData, listMotordata);
	}

	// this is steer motor
	// send action steer command
	this->sendActionLibSteerMotor(listMotordata);

	// control drive motor
	double speed = 0;

	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}
	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);
	}
	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// convert to action messesge
	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

}

void MainWindow::on_btn_SteerMotorSetOffEncode_clicked(){


	qDebug()<<"Steer Motor encoder offet";

	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ENCODER;


	std::vector<std::vector<byte>> listMotordata;

	// control only one steer motor
	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, angle*DIRSTEER, motorData, listMotordata);
	}
	// send action command steer motor
	this->sendActionLibSteerMotor(listMotordata);

}
void MainWindow::on_btn_ConnectNetwork_clicked()
{
	//ui.textBrowser->append("Waiting for action scan server to start");
	_actionScan.waitForServer();
	ui.textBrowser->append("Action scan server started,sending goal");

	// connection action drive motor
	ui.textBrowser->append("Waiting for action steer motor to start");
	_actionSteerMotor.waitForServer();
	ui.textBrowser->append("Action Steer Motor server started,sending goal");
	InitCanNetwork();


	// connection action drive motor
	ui.textBrowser->append("Waiting for action drive motor to start");
	_actionDriveMotor.waitForServer();
	ui.textBrowser->append("Action Drive Motor server started,sending goal");

}

void MainWindow::on_btn_NetworkCam_clicked()
{
	qDebug() << "Image";
}

void MainWindow::on_btn_startTimer_clicked()
{
	timer->start();
}

void MainWindow::on_btn_stopTimer_clicked()
{
	timer->stop();
}

void MainWindow::on_btn_SteerMotorAngle_clicked(){


	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = ui.steerMotorAngle->value();
	qDebug()<<"angle: "<<angle;
	// send data

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::ANGLEOFFSET;


	std::vector<std::vector<byte>> listMotordata;

	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, angle*DIRSTEER, motorData, listMotordata);
	}
	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, angle*DIRSTEER, motorData, listMotordata);
	}

	// send action command steer motor
	this->sendActionLibSteerMotor(listMotordata);

	qDebug() << "Steer angle ";


}


void MainWindow::on_btn_SteerMotorSpeed_clicked(){


	int IdCode = CMD::STEER_MOTOR_SPEED;
	int speed = ui.steerMotorSpeed->value();
	qDebug()<<"speed: "<<speed;
	// send data

	MotorData motorData;

	motorData.maxSpeed = speed;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;


	std::vector<std::vector<byte>> listMotordata;

	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, speed, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, speed, motorData, listMotordata);
	}
	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, speed, motorData, listMotordata);
	}

	// send action command steer motor
	this->sendActionLibSteerMotor(listMotordata);

	qDebug() << "Steer speed ";


}

void MainWindow::on_steerAngleTyping_valueChanged(int value)
{
	ui.steerMotorAngle->setValue(value);
}

void MainWindow::on_btn_SteerAngleStop_clicked()
{


	int IdCode = CMD::STEER_MOTOR_SPEED;
	int angle = 0;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::M1010;
	motorData.state_cmd = WRITE_CMD;
	motorData.command_type = CONTROL_CMD::STOPMOTOR;


	std::vector<std::vector<byte>> listMotordata;

	if (ui.isSteerMotor1->isChecked()) {
		getListMotorData(1, angle*DIRSTEER, motorData, listMotordata);
	}

	if (ui.isSteerMotor2->isChecked()) {
		getListMotorData(2, angle*DIRSTEER, motorData, listMotordata);
	}
	if (ui.isSteerMotor3->isChecked()) {
		getListMotorData(3, angle*DIRSTEER, motorData, listMotordata);
	}


	// send action command steer motor
	this->sendActionLibSteerMotor(listMotordata);

	qDebug() << "Steer angle  Stop";


}


void MainWindow::on_btn_SetCurPeakLaser_clicked()
{
	int IdCode = CMD::SET_CUR_PEAK_LASER;
	bool flag = true;

	qDebug() << "Set Current Peak Laser";

	this-> _oldPeakLaser = _newPeakLaser;

}

void MainWindow::on_driveMotorSpeed_valueChanged()
{
	int speedPercent = ui.driveMotorDirection->isChecked() ? -ui.driveMotorSpeed->value() : ui.driveMotorSpeed->value();
	ui.drivemotorSpeed->setText("Speed: " + QString::number(speedPercent) + "%");
}


void MainWindow::on_steerMotorSpeed_valueChanged()
{
	int speedPercent = ui.steerMotorSpeed->value();
	ui.steermotorSpeed->setText("Speed: " + QString::number(speedPercent) + "%");
}

void MainWindow::on_btn_DriveMotorSpeed_clicked()
{

	int IdCode = CMD::DRIVE_MOTOR_SPEED;
	bool flag = ui.driveMotorDirection->isChecked() ? true : false;
	double value = (double)ui.driveMotorSpeed->value();
	double speed = flag ? - value : value;

	//send data
	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;

	std::vector<std::vector<byte>> listMotordata;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}

	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);

	}


	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

	qDebug() << "Speed Drive Motor: " << speed;

}

void MainWindow::on_btn_DriveMotorStop_clicked()
{


	int IdCode = CMD::DRIVE_MOTOR_SPEED;
	int speed = 0;

	MotorData motorData;
	motorData.motor_type = MOTOR_TYPE::DRIVE_MOTOR;
	motorData.state_cmd = STATE_CMD::WRITE_CMD;
	motorData.command_type = CONTROL_CMD::SPEED;

	std::vector<std::vector<byte>> listMotordata;
	if (ui.isDriveMotor1->isChecked()) {
		getListMotorData(1, speed*DIRMOTOR1, motorData, listMotordata);
	}

	if (ui.isDriveMotor2->isChecked()) {
		getListMotorData(2, speed*DIRMOTOR2, motorData, listMotordata);

	}


	if (ui.isDriveMotor3->isChecked()) {
		getListMotorData(3, speed*DIRMOTOR3, motorData, listMotordata);
	}

	// send action command drive
	this->sendActionLibDriveMotor(listMotordata);

	qDebug() << "Drive Motor Stop";


}

void MainWindow::on_steerMotorAngle_valueChanged()
{
	int angle = ui.steerMotorAngle->value();
	ui.steerMotorAnglelb->setText("Angle: " + QString::number(angle));

	ui.steerAngleTyping->setValue(angle);

}

void MainWindow::doneActionLaserScanCb(const actionlib::SimpleClientGoalState& state,
		const scanResultConstPtr& result){

	ROS_INFO(" Action Scan Finished in state [%s]", state.toString().c_str());

	if(result->IdCode == CMD::LASER_ID_CODE){
		qDebug()<<"Laser Answer: ";
		for(auto p : result->laserCommand){
			if(p == 88) _isSendLaserTracking = true;
			qDebug()<< p;
		}
	}
	else if(result->IdCode == CMD::MOTOR_ID_CODE){
		for(auto p : result->motorCommand){
			if(p == 77) {
				qDebug()<<"Motor Steer Answer";
			}
			else if(p == 66) qDebug()<<"Motor Drive Answer";
		}
	}
	else if(result->IdCode == CMD::STEER_MOTOR_WAIT){
		qDebug()<<"wait for steer answer";
		return;
	}


	//for(int i=0;i<100000;i++){
	//}

	// check list action goals
	if(actionLaserGoals_.size() >0){
		// get front goal

mypcl:scanGoal goal;	       
      goal.IdCode = CMD::MOTOR_ID_CODE;
      std::vector<byte> dataFrame =  actionLaserGoals_.front();

      for(auto p : dataFrame){
	      goal.motorCommand.push_back(p);
      }

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));

      // pop back
      actionLaserGoals_.pop();
	}
}


void MainWindow::activeActionLaserScanCb(){
	ROS_INFO("Goal just went active");
}
void MainWindow::feedbackActionLaserScanCb(const scanFeedbackConstPtr& feedback){
	if(feedback->IdCode == CMD::LASER_ID_CODE){
		qDebug()<<"Feedback Laser Answer: ";
		for(auto p : feedback->laserCommand){
			if(p == 88) _isSendLaserTracking = true;
			qDebug()<< p;
		}
		// display message
		if(feedback->motorCommand.size()>0){
			QByteArray frame(reinterpret_cast<const char*>(feedback->motorCommand.data()), feedback->motorCommand.size());
			qDebug()<<QString(frame);
		}
	}
	else if(feedback->IdCode == CMD::MOTOR_ID_CODE){
		for(auto p : feedback->laserCommand){
			if(p == 77) {
				qDebug()<<"Feedback Motor Steer Answer";
				_isSteerMotorReady = true;
			}
			else if(p == 66) qDebug()<<"Motor Drive Answer";
		}

		// display message
		if(feedback->motorCommand.size()>0){
			QByteArray frame(reinterpret_cast<const char*>(feedback->motorCommand.data()), feedback->motorCommand.size());
			qDebug()<<frame.toHex();
		}
	}


	//for(int i=0;i<100000;i++){
	//}

	// check list action goals
	if(actionLaserGoals_.size() >0){
		// get front goal

mypcl:scanGoal goal;	       
      goal.IdCode = CMD::MOTOR_ID_CODE;
      std::vector<byte> dataFrame =  actionLaserGoals_.front();

      for(auto p : dataFrame){
	      goal.motorCommand.push_back(p);
      }

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));

      // pop back
      actionLaserGoals_.pop();
	}

}
// Action for Drive Motor
void MainWindow::activeActionSteerMotorCb(){
	ROS_INFO("Goal just went active");
}

void MainWindow::feedbackActionSteerMotorCb(const scanFeedbackConstPtr& feedback){

	if(feedback->IdCode == CMD::MOTOR_ID_CODE){
		for(auto p : feedback->laserCommand){
			if(p == 77){
				qDebug()<<"Feedback Motor Steer Answer";
				_isSteerMotorReady = true;
			}
			else if(p == 66) qDebug()<<"Motor Drive Answer";
		}
		// display message
		if(feedback->motorCommand.size()>0){
			QByteArray frame(reinterpret_cast<const char*>(feedback->motorCommand.data()), feedback->motorCommand.size());
			qDebug()<<frame.toHex();
		}
	}

	// check list availible
	if(actionSteerMotorGoals_.size() >0){
mypcl:scanGoal goal;
      goal.IdCode = CMD::MOTOR_ID_CODE;
      std::vector<byte> dataFrame =  actionSteerMotorGoals_.front();
      for(auto p : dataFrame){
	      goal.motorCommand.push_back(p);
      }
      _actionSteerMotor.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionSteerMotorCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionSteerMotorCb,this,_1));

      // pop back
      actionSteerMotorGoals_.pop();

	}

}
void MainWindow::doneActionSteerMotorCb(const actionlib::SimpleClientGoalState& state,
		const scanResultConstPtr& result){
	if(result->IdCode == CMD::MOTOR_ID_CODE){
		for(auto p : result->laserCommand){
			if(p == 77){
				qDebug()<<"Feedback Motor Steer Answer";
			}
			else if(p == 66) qDebug()<<"Motor Drive Answer";
		}
	}
	else if(result->IdCode == CMD::STEER_MOTOR_WAIT){
		qDebug()<<"this is steer server wait for steer answer";
		return;
	}

	// check list availible
	if(actionSteerMotorGoals_.size() >0){
mypcl:scanGoal goal;
      goal.IdCode = CMD::MOTOR_ID_CODE;
      std::vector<byte> dataFrame =  actionSteerMotorGoals_.front();
      for(auto p : dataFrame){
	      goal.motorCommand.push_back(p);
      }
      _actionSteerMotor.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionSteerMotorCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionSteerMotorCb,this,_1));

      // pop back
      actionSteerMotorGoals_.pop();

	}
}

// Action for Drive Motor
void MainWindow::activeActionDriveMotorCb(){
	ROS_INFO("Goal just went active");
}

void MainWindow::feedbackActionDriveMotorCb(const scanFeedbackConstPtr& feedback){

	if(feedback->IdCode == CMD::MOTOR_ID_CODE){
		for(auto p : feedback->laserCommand){
			if(p == 77){
				qDebug()<<"Feedback Motor Steer Answer";
				_isSteerMotorReady = true;
			}
			else if(p == 66) qDebug()<<"Motor Drive Answer";
		}
		// display message
		if(feedback->motorCommand.size()>0){
			QByteArray frame(reinterpret_cast<const char*>(feedback->motorCommand.data()), feedback->motorCommand.size());
			qDebug()<<frame.toHex();
		}
	}

	// check list availible
	if(actionDriveMotorGoals_.size() >0){
mypcl:scanGoal goal;
      goal.IdCode = CMD::MOTOR_ID_CODE;
      std::vector<byte> dataFrame =  actionDriveMotorGoals_.front();
      for(auto p : dataFrame){
	      goal.motorCommand.push_back(p);
      }
      _actionDriveMotor.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionDriveMotorCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionDriveMotorCb,this,_1));

      // pop back
      actionDriveMotorGoals_.pop();

	}

}
void MainWindow::doneActionDriveMotorCb(const actionlib::SimpleClientGoalState& state,
		const scanResultConstPtr& result){
	if(result->IdCode == CMD::MOTOR_ID_CODE){
		for(auto p : result->laserCommand){
			if(p == 77){
				qDebug()<<"Feedback Motor Steer Answer";
			}
			else if(p == 66) qDebug()<<"Motor Drive Answer";
		}
	}
	else if(result->IdCode == CMD::STEER_MOTOR_WAIT){
		qDebug()<<"this is drive server wait for steer answer";
		return;
	}

	// check list availible
	if(actionDriveMotorGoals_.size() >0){
mypcl:scanGoal goal;
      goal.IdCode = CMD::MOTOR_ID_CODE;
      std::vector<byte> dataFrame =  actionDriveMotorGoals_.front();
      for(auto p : dataFrame){
	      goal.motorCommand.push_back(p);
      }
      _actionDriveMotor.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionDriveMotorCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionDriveMotorCb,this,_1));

      // pop back
      actionDriveMotorGoals_.pop();

	}
}


void MainWindow::run()
{

}

// Front Scan
void MainWindow::on_btn_FrontScanning_clicked(){
	qDebug()<<"Front Scan";

	int IdCode = 15;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}

void MainWindow::on_btn_TrackingStart_clicked(){
	qDebug()<<"Tracking Start";

	int IdCode = 16;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
      // set enable steering motor
      this->_isSteerMotorReady = true;
}


void MainWindow::on_btn_TrackingStop_clicked(){
	qDebug()<<"Tracking Stop";

	int IdCode = 16;
	bool flag = 0;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));

      // set disable steering motor
      this->_isSteerMotorReady = false;
}


void MainWindow::on_btn_FrontJobForward_pressed(){
	qDebug()<<"front job forward";

	int IdCode = 17;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}


void MainWindow::on_btn_FrontJobBackward_pressed(){
	qDebug()<<"front job backward";

	int IdCode = 18;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}


void MainWindow::on_btn_FrontJobForward_released(){
	qDebug()<<"front job forward released";


	int IdCode = 17;
	bool flag = 0;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}


void MainWindow::on_btn_FrontJobBackward_released(){
	qDebug()<<"front job backward released";

	int IdCode = 18;
	bool flag = 0;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 13;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));

}
// Back scan


void MainWindow::on_btn_BackScanning_clicked(){
	qDebug()<<"Back Scan";
	int IdCode = 15;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}



void MainWindow::on_btn_GrindingStart_clicked(){
	qDebug()<<"Grinding Start";

	int IdCode = 16;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}

void MainWindow::on_btn_GrindingStop_clicked(){
	qDebug()<<"Grinding Stop";

	int IdCode = 16;
	bool flag = 0;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}
void MainWindow::on_btn_BackJobForward_pressed(){
	qDebug()<<"back job forward";

	int IdCode = 17;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}


void MainWindow::on_btn_BackJobBackward_pressed(){
	qDebug()<<"back job backward";

	int IdCode = 18;
	bool flag = 1;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));

}

void MainWindow::on_btn_BackJobForward_released(){
	qDebug()<<"back job forward released";

	int IdCode = 17;
	bool flag = 0;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}


void MainWindow::on_btn_BackJobBackward_released(){
	qDebug()<<"back job backward released";

	int IdCode = 18;
	bool flag = 0;

	// convert to laserCommand
mypcl:scanGoal goal;
      goal.IdCode = 14;
      goal.laserCommand.push_back(IdCode);
      goal.laserCommand.push_back(flag);

      _actionScan.sendGoal(goal,
		      boost::bind(&MainWindow::doneActionLaserScanCb,this,_1,_2),
		      scanClient::SimpleActiveCallback(),
		      boost::bind(&MainWindow::feedbackActionLaserScanCb,this,_1));
}


