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

##Design conisderations
- Use classes
- APP (GUI) acts as "main"

- APP (gui)
	- DomBot.mlapp
		- Create Log, Simulation objects
- @Log
	- Log.m	
		- Class to log messages to command window
		- Accessable as superclass
- @Simulation < @Log
	- Simulation.m
		- Create Domino, Extinguisher, MyCobot, StopButton, Table objects
		- Create environment (floor, sidewalls)
- @Domino < @Log
	- Domino.m
		- Object(s) to be picked up by robot
- @Extinguisher < @Log
	- Extinguisher.m
		- Safety equipment to be located in the environment
- @MyCobot < @Log
	- MyCobot.m
		- Chosen robot
		- https://www.elephantrobotics.com/en/mycobot-en/
- @StopButton < @Log
	- StopButton.m
		- Safety equipment to be located in the environment
- @Table < @Log
	- Table.m
		- Equipment to be located in the environment
		- Robot & dominos to be placed on table
