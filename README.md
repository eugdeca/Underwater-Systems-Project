# Autonomous Underwater Vehicle (AUV) Navigation Stack

## ⚠️ Note on Project Execution & Proprietary Dependencies
The core simulation environment, including the AUV dynamic model and the 3D underwater rendering, is based on a proprietary package provided by the university and cannot be published due to intellectual property restrictions. However, this repository contains the complete autonomous navigation stack developed by our team.

## About the Project
This repository contains the ROS Melodic navigation stack for an AUV named Zeno. The primary goal of the system is to autonomously explore an unknown underwater environment to build a bathymetric map. Subsequently, based entirely on the generated 2D map, the vehicle plans and executes a collision-free path to a designated target. 

This project was developed for the "Sistemi Subacquei" (Underwater Systems) course at the University of Pisa.

## Key Features

### 1. Lawn-Mower Exploration
For the initial mapping phase, the AUV executes a systematic "lawn-mower" (serpentine) trajectory.
* **High Configurability:** parameters such as line spacing, the GPS bounding box of the exploration area, and the scan direction can be easily adjusted via a YAML configuration file.
* **NED Coordinate System:** geographical coordinates are converted into the local NED (North-East-Down) frame for accurate waypoint generation.
* **Optimal Sensor Coverage:** the spacing between parallel lines is mathematically derived from the multibeam sonar's 60° aperture and the minimum depth of the basin to guarantee zero blind spots.

> 

https://github.com/user-attachments/assets/7e071b24-c9c7-4717-b99d-0c4e9b974f64

> *Simulation speed: 16x. Map environment: Laghetti di Campo (Pisa).*

### 2. Bathymetry-Based Motion Planning (RRT)
Once the exploration is complete and the OccupancyGrid map is published, the obstacle avoidance task begins.
* **RRT Algorithm:** path planning is handled by the Rapidly-exploring Random Tree (RRT) algorithm, chosen for its computational efficiency.
* **Obstacle Inflation:** to ensure maximum safety, all detected obstacles are inflated by 1 meter in all directions, safely accommodating the AUV's physical dimensions.
* **Path Smoothing:** the resulting raw path is optimized using B-spline curves, allowing for fluid navigation without sharp turns that could destabilize the vehicle.

> 

https://github.com/user-attachments/assets/9903e0c9-a60d-4d81-a7eb-2f4bfb97709d

 > *Simulation speed: 7x. Simulated on a fictitious test map.*

## Authors
* **Eugenio Delli Carri**
* **Luigi Bianco Esposito**
* **Artur Vagapov**
