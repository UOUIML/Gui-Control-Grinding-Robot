# Grinding_robot
Install Package:\
OpenCv\
PCL: Point Cloud Lib\
QT
# Action File for Command Line
mypcl/action/scan.action
# Build the package
Catkin_make
# Call the GUI grinding robot 
rosrun grinding_robot gui_motor\
(read grinding_robot/CMakeLists.txt)
# Call the node on Raspberry
ssh to raspberry: ssh ubuntu@192.168.0.15\
password: thocao2019\
rosrun mypcl mylaser\
rosrun mypcl myDriveMotor\
rosrun mypcl mySteerMotor\
# Call all of the nodes in one time
On the Laptop PC\
roslaunch grinding_robot robot.launch



