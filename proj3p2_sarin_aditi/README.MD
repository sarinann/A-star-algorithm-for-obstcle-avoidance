ENPM661 - PROJECT3 - PHASE 2
Planning for Autonomous Robots

File Name: proj3_phase2_sarin_aditi.zip
Team Members:
Sarin Ann Mathew (sarinann/119390382)
Aditi Arun Bhoir (abhoir/119197257)

Libraries used: 
Queue
Time
Copy
Matplotlib
Numpy
OpenCV
Math
rospy
tf.transformations
geometry_msgs.msg
nav_msgs.msg

Download the zip into your ROS workspace/src and Unzip it

Part 01

- Run the file a_star_sarin_aditi.py to generate the nodes and paths through A-star Algorithm

- Input the clearance (preferrably 50mm)

- Input the start and goal points, you can preferably use one of the following combinations
    start = (0.5, 1, 0), (0.3, 1.5, 0) , (0.2, 1.2, 0), (0.4, 0.9, 0)
    goal = (4.7, 1.5, 0), (5.5, 1.5, 0), (4.8, 0.7, 0), (5.1, 0.5, 0)

- Input left velocity between 60-70 rpm and right wheel velocity 120-140 rpm

- There will one output plot showing all the explored nodes and the shortest path

- The shortest path will also be printed in the terminal

- The final map will refelect the plot and the axes limits as per the gazebo world

Part 02

- Commands to launch the turtlebot3 burger model in gazebo 
                   
        • Spawn robot in gazebo world
            $ roslaunch ros_package project3_phase2_world.launch

        • Program to move the robot through the way points
            - change the path of the file 'generated_path.txt' inside "ros_package/src/move_robot.py" according to your system
            - In another terminal
                -roslaunch ros_package project3_phase2_code.launch


GitHub Link: https://github.com/sarinann/https---github.com-sarinann-A-star-algorithm-for-obstcle-avoidance.git


(Both the 2D and 3D visualization videos can be found here)
Video Submission Link: https://drive.google.com/drive/folders/1ePd85D5xu_zfnBQGCmzUT0KYUw8MOgVs?usp=share_link



