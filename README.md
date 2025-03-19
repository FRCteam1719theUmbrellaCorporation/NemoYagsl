# 1719Code2025
Code for 1719's 2025 robot: Nessie!

## New things and Updates
YASGL - We implemented YAGSL onto our robot. From faulty configuration files, to a horrible grinding sound, the beginning of the season was filled with challenges implementing our swerve drive.

Commands - We knew it was critical to have a command that moved the robot to the reef to score consistently. We began using PathPlanner's "On the Fly" path planning to implement such a command with the readings from our limelight. Unfortunately, Pathplanner and our own custom commands could not produce a precise enough command to move the robot to the nearest April Tag. With a correctly functioning gyroscope, we used MegaTag 2's April Tag localizer on our Limelight 4's in coordination with AdvantageScope to precisely use poseEstimate to find our robot's location on the game field. With this, we were able to build accurate paths to the reef.

Field-Oriented Driving - We also found an issue in our robot's field-oriented driving. Our Pigeon 2 was experiencing an issue called the Gimble Lock, which was due to a rotation in our gyro causing internal confusion to its internal Yaw and Roll readings. With this small fix, our field-oriented drive and our issues with precision were solved.


## CAN IDs

| Motor Label   | Motor Location | CAN ID |
| ------------- | -------------- | ------ |
| Motor #1      | Left Front Drive     | 11      |
| Motor #2      | Left Front Rotation Motor    | 12      |
| Motor #3      | Left Front CANCoder    | 10      |
| Motor #4      | Right Front Drive    | 21      |
| Motor #5      | Right Front Rotation Motor       | 22     |
| Motor #6      | Right Front CANCoder       | 20      |
| Motor #7      | Left Back Drive    | 31      |
| Motor #8      | Left Back Rotation Motor | 32      |
| Motor #9      | Left Back CANCoder | 30    |
| Motor #10     | Right Back Drive |36      |
| Motor #11     | Right Back Rotation Motor |37      |
| Motor #12     | Right Back CANCoder |35      |
| Gyro #13      | Gyro Port |2      |
| Motor #14     | Intake/Outtake Motors |9      |
| Motor #15     | Elevator |6     |
| Motor #16     | End Effector Motor |24      |
| Motor #17     | Intake Pivot Motor |23      |



## Button Mappings:
[Consult this document](https://docs.google.com/document/d/1tnhtNMz3D61-BR17y95493GGfkCHpqT0dgg2AZGWeoQ/edit?tab=t.0)
      


## Instructions

### Useful pages
   Setup: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/offline-installation-preparations.html
   
   WPILib Java Examples: https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/wpilib-examples.html
   

### How to add a subsytem and commands
   1. Define a subsystem class
         1. Define components of the subsystem in the subsystem class - motors, motor groups, sensors
         1. Define functions - at a basic level what can this subsystem do - shoot, drive, etc.
         1. Publish telementry through NetworkTables
   1. Define command classes
         1. There can be more than one - collect balls, eject balls
         2. Pass subsystem to the constructor
   1. Wire up subsystems, commands and buttons in RobotContainer
         1. Instance of subsystem
         2. Wireup code

### Git commands    

Note: Just use the built in visual studio code git stuff, it's easiar. I'm leaving this here in case.

```git clone https://github.com/FRCTeam1719/2022Robot_3``` command to get brand new repository

```git pull``` pull down any updates from github

```git add . ``` add all files that were added or modified to the local repo

```git commit -m "<message goes here>" ``` stage files to commit

```git push``` push your commits to github

```git checkout (-b) <branch goes here>``` Creates / pulls a branch from github. Use -b when checking out a new branch!



# Team 1719 is a Student-Run Team

### Programmers:

_______________________________________________________
Neel Shrestha @realrealneel(Lead programmer 1)
_______________________________________________________
Harrison Green @Hbg1010(Merge victim :( )
_______________________________________________________
Ian Borden @Ianborden
_______________________________________________________
Owen Rubin @owarr(Starving writer)
_______________________________________________________
Perry Sahnow @perrys25
_______________________________________________________



### Mentors

------------------------------------------------------
Ben Sugarman @bsugarman
_______________________________________________________
Matthew Green @matthewdgreen
