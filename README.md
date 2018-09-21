# CompRobo Warmup Project

Anil Patel and Nina Tchirkova
September 21, 2018

## Project Overview
For those readers who may be completely unfamiliar with this class and this project, we were tasked with developing a variety of robotic behaviors with a Neato botvac as our platform. We began this project with minimal robotics experience, and no formal training in ROS. This project aligns itself well with the Olin mantra of “do learn.” By the end, we felt confident in our ability to analyze challenges, incrementally develop algorithms, and implement robust solutions.

We were tasked to build the following behaviors:
- Teleop - control the robot using keyboard commands.
- Drive square - pilot the robot to drive along the perimeter of a 1m side length square.
- Wall following - the robot must locate and drive parallel to a wall within its view.
- Person following - the robot tracks a person in front of it and follows it around.
- Obstacle avoidance - descriptive title, the neato avoids obstacles in its path.
- Finite-state control - some environmental input changes the robot between behaviors.

## Drive Square
The algorithm for driving a square is fairly simple, but we used it to develop some basic functions that would assist future, more advanced development. After experimentally matching robot commands to real angular and linear velocities. We then created a very rudimentary, timing based loop that would drive a square based on successive 90° turns and one meter forward movements. This was done by expressing to the robot how long we want it to drive for and how long we want it to turn for.

## Wall Following
There are a few new challenges associated with the wall-following behavior. First, we need to be able to recognize walls based on laser-scan data. Second, we need to be able to identify the orientation of the robot relative to the wall. Our algorithm interprets a laser scan and defines the slope of a wall relative to the robots “odom” frame. If the slope is negative, the robot turns left. If the slope is positive the robot turns right. If the slope is about undefined (parallel to the robot) then the robot moves forward. The robot repeats this decision making process every time it processes a scan, which is how it follows a wall.

image here

### Handling Laser Scan Data
We knew developing a solid callback function to deal with laser scan data cleanly would facilitate future behavior development. The laser has an angular resolution of 1°, meaning that the message received from the /scan topic has 360 values, each of which denotes a distance in meters and corresponds to an angle that matches the list index of the value. The data is received where the first value corresponds to an angle in  line with the neato’s heading. 

When a scan is received, we iterate through the data and convert it to cartesian coordinates, using the index to determine the angle (in degrees) and the value as the radius of polar coordinates. These values were stored as attributes of the class so that they could be accessed in later functions. We opted to not include any driving behavior in our callback function so that we decoupled our controls from the frequency of the laser topic.

### RANSAC
To find the slope of the wall in respect to the Neato we implemented a version of the Random Sample Consensus algorithm (RANSAC). Given the coordinates of all the points in a sample, the algorithm chooses two points pseudo randomly and then defines a line using them. Then, the rest of the points are tested to see if they lie on the line. The slope, intercept and how many points are on the line are saved. The process is then repeated as many times as there are points, if a line is found with a higher count of points on it than the one previously saved, it is replaced as the saved line. At the end, the line with the most points on it is returned. 

## Person Following
To identify a person in the neato’s laser scan data we implemented a function that found the center of mass of the points in its view. We chose to constrict the algorithm to only process points that were in front of the robot.

### Center of Mass
To find the effective position of a person in the Neato’s view, we took the “center of mass” of the laser scan points. The “center of mass” is the essentially the average value of all the x points and the average value of all the y points. We abstracted the problem to assume the Neato is in a room with no walls, and the only points in the view are from a person in front of the neato that it is tracking. This meant that the average value of the points from the laser scan as the center of the object. The view was limited to points within an angular range and distance in front of the Neato.

picture here

### Proportional Control
Once we determined a desired target point, the neato’s angular and linear velocities were updated using proportional control. A desired distance away from the person we were tracking was defined as 0.5 meters, to account for the fact that the laser scanner is offset from the frontmost plane of the neato. The desired angle was set to 0, so that the neato was constantly trying to align itself with the object it was tracking. Kp values for angular and linear velocities were experimentally tuned so that the Neato followed the person it was tracking without overshooting and causing underdamped oscillation.

## Obstacle Avoidance
At this point, we had built up functions that can parse laser scan data and proportionally control the Neato. To avoid objects we created a potential field, where every scan point was a repulsive force. Resolving the potential field at the neato’s position gave us a vector that pointed in the direction in the line that most avoided all physical objects.

### Potential Field
In order to create a potential field from the laser scan, every point that has a non-zero value is made into a repulsive vector in the Neato’s frame. The sum of these vectors is then calculated and then the robot is directed in the direction of the resultant vector. 

image here

### Drive Controls
With a orientation vector was determined, the driving controls only needed an adjustment to the our person-following proportional controller. The vector was treated as a point in 2D space that acted as the neato’s target position to avoid obstacles. The only changes we need to implement in our previously developed code was to tune Kp values so that the robot turned sufficiently quickly and moved slowly enough to avoid dynamic obstacles.

## Finite-State Control
We combined wall-following and obstacle avoidance in our finite-state control. When the program starts, the robot is avoiding obstacles, and when it finds a wall it changes state to follow it. Should any obstacles present themselves, the robot will change state to avoid them. The key addition here was the criteria for changing state.

### State Change
Everytime a laser scan is processed, if a wall is discovered with more points than a certain threshold then the robot switches into wall following mode. Once a wall is not seen (points on line are less than the threshold, or the majority of points detected are of objects not walls), then the robot switches back into obstacle avoidance.

## Code Structure
Each behavior was encapsulated within a ROS node. The node was defined as a class with methods carrying out the key processes each node needed to perform. Nodes had methods, to listen to topics, publish to topics, intermediate functions to process the data, and then a final run method that put everything together. To run the robot, all that needed to be done was to instantiate the class and then call the run method. Many of our nodes shared processing publishing and listening methods.

## Challenges Faced
Most of the challenges we encountered were having almost no experience with ROS before. It took us a while to learn how to receive and publish messages. However, once we familiarized ourselves with the platform we developed with much more ease and speed. Other issues we experienced were due to our environments. Rviz did not really work for either of us at first, so we had to debug using methods like Numpy instead.

## Opportunity for Improvements
There are many ways to improve this project. In terms of code structure, there could be a generalized node class that other nodes for specific behaviors could inherit from. This would have prevented a lot of redundancy in our code base. Since we needed to accomplish many different behaviors, we generally picked one algorithm per a behavior and tuned it until it worked to our satisfaction. However, it would have been really interesting to compare the effectiveness of different algorithms for the same behavior. Our algorithms are also not very well tuned for edge cases, for example, encountering a corner when following a wall. To improve our project it would be important to look into these cases and account for them in our code.
