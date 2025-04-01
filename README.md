# RINS-Cali
This is the code base for paper *"RINS-Cali"*
## MATLAB simulation
The MATLAB codes are used for RINS calibration RTLS simulation, to bring up the simulation, make sure all the files are in current working directory, then open **auto_imu_cali.mlx** and simply run, you can change the mounting errors manually, and the calibration results will displayed as a plot.
![example](https://github.com/user-attachments/assets/ec156ab6-58a1-4b28-9397-fcad554d69db)
## Python ROS node for real-world test
The python ROS node is for identification of the RINS mounting errors in real world implementations. First bring up your own IMU interfacing node, then bring up the **imu-ros-node.py** node, make sure the node subscribe to the correct IMU topic, in my case it is called "/IMU_data". The results will print to the console and save to a file called "errors_log.txt", where each row represents the batch results from a recursion, and each column stands for *alpha*, *beta*, *y*, *z*, respectively.
