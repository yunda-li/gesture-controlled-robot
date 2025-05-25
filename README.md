# gesture-controlled-robot
This project implements a robotics system on the Duckietown DB21J Robot that allows it to follow a human subject and respond to gesture commands. Making heavy use of the Duckietown software and robotics platform, this system aims to demonstrate intuitive human-robot interaction through pose estimation and gesture recognition.This project makes heavy use of the Duckietown software and robotics platform.

## Features

* Human subject following
* Gesture command recognition
* Integration with Duckietown DB21J robot and software
* Utilizes Google MediaPipe for 2D pose estimation

## Key Technologies & Concepts

* **Robot Operating System (ROS Noetic):** Core framework for inter-process communication, sensor integration, and robot control.
* **Google MediaPipe Pose Landmarker:** Employed for highly accurate, real-time 2D human pose estimation.
* **Docker:** Ensures a consistent and reproducible development and deployment environment.
* **Python:** Primary programming language for implementing robotics logic, computer vision, and gesture recognition algorithms.
* **Computer Vision:** Applied for pose estimation, image processing, and potential future object detection.
* **Human-Robot Interaction (HRI):** Focus on designing natural and intuitive control paradigms.
* **Robot Control & Navigation:** Implementation of reactive behaviors and basic path planning for following and direction.

## Requirements

### Hardware
* [Duckietown DB21J Robot](https://get.duckietown.com/products/duckiebot-db21)
* WiFi network with administrative privileges (for robot-computer connection)
* **Recommended:** Nvidia GPU (greatly improves pose estimation performance)

### Software
* Ubuntu 20.04 LTS
* ROS Noetic
* Docker 
* [Google MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker) (Dependencies handled by Docker build)

## Setup

Assuming you have already:
* Completed the initial setup of your DB21J robot according to the [Duckietown Operations Manual](https://docs.duckietown.com/daffy/opmanual-duckiebot/intro.html).
* Connected your DB21J to your computer via WiFi.

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/yunda-li/gesture-controlled-robot
    cd gesture-controlled-robot
    ```

2.  **Build the Duckietown package (and Docker image):**
    ```bash
    dts devel build -f
    ```
    This command builds the necessary Docker image for the project, handles dependency installation, and compiles any code as defined in the Dockerfile and package structure. 

## Running the Demo

To run the full human following and gesture reading demo, use the command:

```bash
dts devel run -R <robot_name> -L final_launcher -X --runtime nvidia --gpus all
```

You can watch the full final_launcher demo here:

https://github.com/user-attachments/assets/67cffdfc-f72b-49f5-a092-12ef77054b64

## Future Work

Incorporate object detection for the robot to navigate towards after receiving a gesture command

Adding obstacle avoidance
