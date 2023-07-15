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

## Running the Project

## Acknowledgements
