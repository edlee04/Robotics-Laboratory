# Robotics-Laboratory ME301
This project course explored introductory robotics in a laboratory environment. The robot used throughout the course is a hexapod robot shown below. Both software (sensor processing, SLAM) and hardware (sensors, actuators, kinematics) were explored through open-loop control, feedback control, reactive control, motion planning, and SLAM. All code is written in Python.

<p align="center">
  <img width="250" height="250" src="https://github.com/edlee04/Robotics-Laboratory/blob/main/Hexapod.png">
</p>

## How the Robot Moves:
The hexapod robot has 6 legs and 3 servos on each leg, resulting in 18 total DoF. The robot is equipped with sonar sensors, an IMU, and a Raspberry Pi. All programs and algorithms were coded using Python and a provided sdk (ros_robot_controller_sdk).

## Project 1: Gait Development for Hexapod Robot
The hexapod robot has a provided SDK with functions to control the joint servos and other electrical components on the robot. By using servo functions, each joint of the servo was characterized with home positions, directionality, and angle limits. Furthermore, the sonar sensor was characterized with readings from various distances.

With each joint tested, three common hexapod gaits were developed: wave, ripple, tripod. Wave moves one leg at time. Ripple gaits move a pair of legs (one of each side) at a time. Tripod gaits move an alternating group of three legs at one time. For each of the gaits, while a leg moves forward, the others should be grounded and pushing back on the ground. This motion maximizes the forward displacement of the body.

[Project 1 Code](https://github.com/edlee04/Robotics-Laboratory/blob/main/asn0_D.py)

## Project 2: Turning Gaits, Reactive Control, and Wall Following Feedback Control for Hexapod Robot
With the gaits created from the previous project, the walks must now be optimized. Each type of gait results in drift of the hexapod. This was achieved with closed-loop wall following algorithm was implemented. Specifically, a P controller used the error (desired distance from wall - actual distance from wall) to maintain a set distance from the wall. 

Furthermore, reactive control algorithms were designed for case scenarios. For example, if a wall was detected to the left and the front of the robot, the robot will turn to the right.

[Project 2 Code](https://github.com/edlee04/Robotics-Laboratory/blob/main/asn1_D.py)

## Project 3: Localization, Planning, SLAM using Hexapod Robot
The hexapod must navigate a maze with localization, path planning, and mapping algorithms. The maze consists of a grid space where one cell is a 2x2 tile on the ground. (i, j, k) denotes the position of the hexapod in the grid space. i is in the North-South direction, while j is in the West-East direction. A positive i is in the South direction, while a positive j is in the East direction. k is the heading of the robot: 1 is North, 2 is East, 3 is South, 4 is West. The origin of the maze is at (0, 0, 1) at the top left cell of the grid.

Localization is the abiliy to determine position in the maze. The robot is given an initial position and end position. The hexapod will traverse until it reaches the end configuration while continuously updating its internal state. Path planning takes an already existing map with walls/open spaces, and uses wavefront propagation to determine the best path to reach the goal. Wavefront propagation assigns cost values to cells and counts from highest to lowest cost to determine the optimal path. Finally, mapping involves wandering an unknown maze while updating an internal map with walls/open spaces. The hexapod will be placed in a random square and wander around with a depth-first search to explore the entire maze. A sonar sensor is utilized to detect surrounding walls and spaces and updates an internal map to eventually map out the entire maze.

[Project 3 Code]()

## Project 4: Machine Learning and Reinforcement Learning for Pet Following Robot
The robot will use ML/RL to learn to follow someone as a pet. There are state spaces that describe the distance from the robot to the desired target. The states are determined through sonar readings. Possible actions involve walking forward, walking backward, stopping, and searching. A Q-table is continuously updated based on random actions the robot takes at any state.

As the robot continues to perform actions and reward values are updated, the robot will learn the optimal actions to take to appropriately follow a person. For example, through performing several training trials, the robot will learn that if it is too far from the target, it will take a forward step to get closer.

[Project 4 Code](https://github.com/edlee04/Robotics-Laboratory/blob/main/asn3_d.py)
