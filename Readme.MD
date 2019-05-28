# Test Trajectory Library Project

## Preface
This is a project I've been working on the past 2 weeks (started 5/16/19) in order to give the student members of FRC Team 3230 an example of FRC programming in Java.
At the next meeting, I'll explain the documentation here, as well as basics of each class in the library and the function of each element in the code in a basic way.
The goal of this project is to begin to get students thinking about Java programming and it's essential to have a sample to look towards.

## Code Synopsis
This project assumes a robot with a Limelight subsystem and a drivetrain currently coded for Talon SRX motor controllers (easily switchable to Neos when needed) where it is able to follow a designated path with Jaci's Pathfinder, move towards a vision target using (really quite rudimentary) Limelight vision tracking, and show the vital elements of data logging of subsystems without the use of SmartDashboard (this is an element easily added in the future)

## Setup Instructions

### General
- Clone this repo
- Run `./gradlew` to download gradle and needed FRC libraries
- Run `./gradlew tasks` to see available build options
- Enjoy!

#### Eclipse
- Run `./gradlew eclipse`
- Open Eclipse and go to "File > Open Projects" from "File System..."
- Set the import source to the `TestTrajLib` folder then click finish
- NOT RECOMMENDED

#### IntelliJ
- Run `./gradlew idea`
- Open the `TestTrajLib.ipr` file with IntelliJ
- NOT RECOMMENDED

#### Visual Studio Code
- Open repository in your Visual Studio Code install with the WPILib extension enabled
- Test build the project using the command palette and selecting `WPILib: Build Robot Code`

### Building/Deploying to the Robot through Console
- Run `./gradlew build` to build the code. Use the `--info` flag for more details
- Run `./gradlew deploy` to deploy to the robot in Terminal (Mac) or Powershell (Windows)

### Deploying the Robot through Visual Studio Code
- On the command palette, select `Deploy Robot Code` (or press Shift+F5) when connected to the RoboRio

### Testing the Robot through Visual Studio Code's Simulation
- On the command palette, select `Simulate Robot Code on Desktop`
- Wait for a dropdown menu and select `halsim_ds_socket.dll`
- Open DriverStation on your computer and Enjoy!


## Variable Naming Conventions
- General Camel Case (i.e. `leftMaster` instead of another variant)
- k*** (i.e. `kLeftMasterID`): Final constants
- m*** (i.e. `mInstance`): Private instance variables
