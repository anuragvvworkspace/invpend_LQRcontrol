# Inverted Pendulum LQR control

## Launch gazebo model using 
roslaunch invpend_control load_invpend.launch

## PID
### PID controller parameter tuning 
Operate rqt_reconfigure to modify PID values
launch rqt_reconfigure with the following command

`rosrun rqt_reconfigure rqt_reconfigure`

### Demo: Inbuilt PID controller
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Mm0ADOcttg8/0.jpg)](https://youtu.be/Mm0ADOcttg8)

## LQR control
'controllqr.py' in 'invpend_control/src' is to be run to execute LQR control.

Goal is set to

`[[1],[0],[0],[0]]`

### Demo of LQR control on inverted pendulum. Goal : `[[1],[0],[0],[0]]`
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/5TH1VlKVSfs/0.jpg)](https://youtu.be/5TH1VlKVSfs)

## Plot of cart position, velocity and Pole angular position and angular velocity
![alt txt](invpend_control/scripts/plot1.png "Plot 1")

