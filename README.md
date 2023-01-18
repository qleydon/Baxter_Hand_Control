# Baxter_Hand_Control
Quinn Leydon Final Project

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
To Set Up: 
Move leydon_project to baxter workspace.
open camera.py or depth_camera.py in pycharm or IDE
install required packages. Conda is optional. 
connect baxter, Linux machine, and windows machine to same network with same subnet
setup Leydon_UDP_Publisher so that the address is the same as the windows address. 
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Run Using Webcam (Prefered):
open camera.py in IDE (Pycharm used)
follow setup at top of page. setup z_type, orientation, and RPY type. 
Run code. It will wait for UDP client. 

open linux machine
setup and enable baxter or gazebo, either works. 
rosrun Leydon_UDP_Publisher.py
setup Baxter vairable in main. 
	NOTE: RPY_type MUST BE CONSISTANT
rosrun Leydon_IK.py
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
to use realsense depth camera:
install required drivers for windows. follow instructions here: https://www.intelrealsense.com/get-started-depth-camera/
Proceed same as webcam but on Distance. 
Get some measurements then scale the z so range of motion is between 0.3 and 1. 
NOTE: Intelsence used had noisy image which cause poor edge detection. This resulted in a worse mediapipe detection. Distance was noisy as well. 
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Things to Note:
I was unable to give baxter Roll Pitch and Yaw together. using RPY_Type = 0 uses yaw as negative roll. RPY_Type = 1 gives raw quaternion yet strangly does not consider roll. 
If there is a connection error between Publisher and camera.py, try running reconnecting to baxter and running Leydon_IK.py. This typicaly fixes the issue. 
If messages are published too quick, it creates a lag. to test move hand off camera and watch the time until rostopic to stabilize. 
