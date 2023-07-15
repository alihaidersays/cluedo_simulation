# Cluedo Simulation using ROS

## Experimental Robotics Laboratory - Assignment I
Muhammad Ali Haider Dar, _[5046263@studenti.unige.it](mailto:5046263@studenti.unige.it)_

MSc Robotics Engineering, University of Genoa, Italy

Instructor: [Prof. Carmine Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)

## Overview

The repository provided contains a ROS package that simulates a version of the Cluedo game, where a robot explores an environment in search of hints to identify the killer. The project's environment is an apartment consisting of three rooms that the robot enters one by one to uncover clues. The robot utilizes the discovered hints to formulate hypotheses about the identity of the killer. These hypotheses must be both "consistent" and "correct", meaning they are based on three different types of hints and belong to a predefined set of correct hypotheses.

The hints obtained by the robot fall into three categories:

- Who: The robot can find the name of a person, which serves as a potential hint about the killer, such as "Col. Mustard."
- What: The robot can find the name of a weapon, indicating a possible weapon used by the killer, for example, "Revolver."
- Where: The robot can discover the name of a location where the crime might have taken place, like "Conservatory."

An example of a consistent and correct hypothesis statement would be: "Prof. Plum with the Dagger in the Hall." If the deduced hypothesis is incorrect, the robot will revisit the three rooms to search for new hints until it formulates a consistent hypothesis.

To deduce hypotheses, the robot utilizes the ARMOR package service, developed by researchers at the University of Genova. ARMOR is a versatile management system designed to handle single or multiple-ontology architectures within the ROS framework. For more information about ARMOR, visit this [link](https://github.com/EmaroLab/armor).

## Installation

To ensure the proper functioning of this project, it is necessary to have ROS with the ARMOR package installed on your system. Please ensure that you have already installed them before proceeding with the instructions. If you haven't installed the ARMOR package yet, please follow the instructions available in this repository: https://github.com/EmaroLab/armor

Here are the steps to set up the project:

1. Download the code available in the Main branch, which is a ROS package. Place it in your ROS workspace at `{ros_ws}/src` after downloading.

2. To deploy and build the package successfully, execute the following commands:
```
catkin_make
cd devel/
source setup.bash
```

3. To use the Python modules contained in the `armor_py_api` package, add the path of the ARMOR Python modules to your `PYTHONPATH` environmental variable by running the following command:
```
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```
  Note: Replace /root/ros_ws with the correct path to your ROS workspace.

4. Download the 'cluedo_ontology.owl' file provided in this repository and place it in the '/root/Desktop/' directory.

Make sure to follow these steps to ensure the successful setup and usage of the project with the ARMOR package.


## Running the Project
1. Once you have successfully installed the Python package and other dependencies, open a command window and start the ROS master by running the following command:
```
roscore &
```

2. Next, start the ARMOR service by executing the following command:
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```

3. Open a new tab in the command terminal and launch the ROS package's launch file to initiate the simulation. Use the following command:
```
roslaunch exprob_a1 exprob_a1.launch
```

Wait for the system to load all the necessary files. Once all the nodes have finished loading, the `user_interface` node will prompt you to press 1. Upon pressing 1, the simulation will begin.

