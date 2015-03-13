# Benchmark definition for automatic navigation
============
Date: 

Location: Rose BV, Horsten 1, Eindhoven

Tester: 

Introduction
------------

In order to benchmark the automatic navigation software a set of well defined test situations is required.
This document intends to provide these situations.

How to grade these benchmarks?

* Reaching goal (yes/no)
* # collisions
* Time (seconds)
* Smoothness (subjective 0 - 10)

## Abbreviations
IR - inscribe radius of the robot
CR - circumscribed radius of the robot

Simple Goal Benchmarks
----------

These benchmarks test the ability of the automatic navigation to navigate to a simple goal pose. 
A goal pose is a position and orientation. The ground is assumed to be flat and easy for the robot to drive on.
Obstacles can all, in theory be seen by the robot. (What to do in situations where the robot has to drive backwards and has no kinect for example?)

These test assume that in all situations the localization of the robot, aka its pose estimate in the world, is reasonably good.

### Simple
#### Simple 1
##### Enviroment setup
Move to the goal in a test area without any obstacles.

![Alt text](images/simple-1.png "Simple 1")

Situation with both the robot and the goal at orientation zero degrees.

The goal is given 4 meters away from the robot. The test is repeated with varying orientations of the robot start pose and the goal pose.

| Orientation Robot | Orientation Goal | Reached | # collisions | Time | Smoothness |
| ----------------- | ---------------- | ------- | ------------ | ---- | ---------- |
| 0 				| 0 			   |         |              |      |            |
| 90 				| 0 			   |         |              |      |            |
| 180 				| 0 			   |         |              |      |            |
| 0 				| 90 			   |         |              |      |            |
| 90 				| 90 			   |         |              |      |            |
| 180 				| 90 			   |         |              |      |            |
| 0 				| 180 			   |         |              |      |            |
| 90 				| 180 			   |         |              |      |            |
| 180 				| 180 			   |         |              |      |            |

#### Simple 2
Move to the goal in a test area with one obstacle extruding from the left side of the area.

![Alt text](images/simple-2.png "Simple 2")

Situation with both the robot and the goal at orientation zero degrees. The extrusion obstacle is placed half way from the robot to the goal. The gap on the right side of the testing area is twice the CR.

The goal is given 4 meters away from the robot. The test is repeated with varying orientations of the robot start pose and the goal pose.

| Orientation Robot | Orientation Goal | Reached | # collisions | Time | Smoothness |
| ----------------- | ---------------- | ------- | ------------ | ---- | ---------- |
| 0 				| 0 			   |         |              |      |            |
| 90 				| 0 			   |         |              |      |            |
| 180 				| 0 			   |         |              |      |            |
| 0 				| 90 			   |         |              |      |            |
| 90 				| 90 			   |         |              |      |            |
| 180 				| 90 			   |         |              |      |            |
| 0 				| 180 			   |         |              |      |            |
| 90 				| 180 			   |         |              |      |            |
| 180 				| 180 			   |         |              |      |            |

### Replanning
These benchmarks will test the replanning behaviour.
#### Replanning 1
The test area must be setup like depicted in the following picture:

![Alt text](images/replanning-1.png "Replanning 1")

The blue obstacle is hidden from sight by the red obstacle, and thus unkown to the robot at the beginning of the test.
The gap on the right and left side of the red obstacle is twice the CR.
The path left from the robot will be the preferred initial path of the robot due to the lower distance. When the obstacle around the corner is detected however the robot should replan and take the path to the right of the red obstacle.

This test can be tested with variable width of the red obstacle. A larger width will increase the 'detour' the robot has to take in order to reach the goal

| Red obstacle width | Reached | # collisions | Time | Smoothness |
| ------------------ | ------- | ------------ | ---- | ---------- |
| 4 CR 				 |         |              |      |            |
| 8 CR 				 |         |              |      |            |
| 16 CR 			 |         |              |      |            |

### Unreachable goal
### Narrow passages
### L-shaped passages
### Narrow corridors
### Tight corners
### Large open spaces
### Open space with a single obstacle
### Slalom 
### U-turns
### Dead ends
### Dynamic enviroments
