# DomBot
DomBot group code for Assignment 2 - Industrial Robotics 41013

## Group Members:
- Koda Enoka - 12580748
- Nathanael Gandhi - 12966382
- Travis Goodshaw - 12912770

## Introduction
### Scope
SafeCo wants us to investigate the possible applications for robot arms in the homes of consumers. Therefore, our project task is to design a GUI and robot arm functionality which allows the robot arm to pick and place dominoes from a stack into a user defined path. We plan to design the GUI to allow the user to easily input their desired domino path. We also intend to incorporate a camera sensor to determine the position of the dominos from the stack.

### Robot Arm
For the project task, we need to use a robot arm which can grip and would be suitable in the homes of consumers. Therefore, my group decided to use the robotic arm MyCobot and a gripper end effector by Elephant Robotics. The main reasons that this collaborative robot arm is ideal for our project because it has 6 degrees of freedom and detailed documentation that has the joint descriptions and dimensions which would allow us to model the MyCobot DH parameters. This robot arm is also relatively small and light weight (850g) when compared to most robotic arms, so it would be safer to use in homes.

## Dependancies 
- robot-9.10Small_Modified_20220202_WithVisionTB
- log4matlab - Gavin (2022). log4matlab (https://www.mathworks.com/matlabcentral/fileexchange/33532-log4matlab), MATLAB Central File Exchange.

## Design Conisderations
- Use classes
- APP (GUI) acts as "main"

## Program Structure
- APP (GUI)
	- DomBot.mlapp
		- Create Log, Simulation objects, set logging flags, simulation start trigger
- @Log < handle
	- Log.m	
		- Class to log messages to command window
		- Log to command window based on log level
		- Log to file (using log4matlab) based on log level 
			- Diluting log level to DEBUG for DEBUG & INFO
			- Diluting log level to ERROR for ERROR & FATAL
- @Simulation < handle
	- Simulation.m
		- Create Domino, Extinguisher, MyCobot, StopButton, Table objects
		- Create environment (floor, sidewalls)
- @EnvironmentObject
	- EnvironmentObject.m
		- Class for objects to inherit from when generated in the environment
		- Object, type, id, pose parameters
		- Checks the type passed in, if it doesnt have a type a type of 'generic' is set
- @Domino < @EnvironmentObject
	- Domino.m
		- Object(s) to be picked up by robot
		- Objects loaded into environment as EnvironmentObject
		- Domino requires +0.025m z-axis intial offset as middle of model is placed at "pose"
		- Offset needs to be negative (-) when picking up the object to get the middle of the object pose
		- Has update pose function available to be called and "move" the domino to the new pose from the old one
- @Extinguisher < @EnvironmentObject
	- Extinguisher.m
		- Safety equipment to be located in the environment
		- Objects loaded into environment as EnvironmentObject
- @MyCobot < @EnvironmentObject
	- MyCobot.m
		- Chosen robot
		- https://www.elephantrobotics.com/en/mycobot-en/
		- workspace generated from robot base pose
		- DH params set in GetMyCobotRobot()
		- Objects loaded into environment as EnvironmentObject
- @StopButton < @EnvironmentObject
	- StopButton.m
		- Safety equipment to be located in the environment
		- Objects loaded into environment as EnvironmentObject
- @Table < @EnvironmentObject
	- Table.m
		- Equipment to be located in the environment
		- Robot & dominos to be placed on table
		- Objects loaded into environment as EnvironmentObject
