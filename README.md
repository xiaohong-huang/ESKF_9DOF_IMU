# ESKF_9DOF_IMU
  Use accelerometer, magnetometer, gyroscope data, use ESKF to estimate attitude.

  Run "main.py"(python main.py), the program will automatically call the "IMU.txt" data in the directory, and then execute the ESKF algorithm. 
  
    "IMU.txt" has acceleration data, gyroscope data, angle data, and magnetic force data. The angle data is the result of a chip's own calculation. 
    
     The algorithm compares its estimated angle with the chip's own calculation results.
