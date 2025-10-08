# ROS Duckiebot

This project is a ROS-based control system for a Duckiebot, a small, autonomous vehicle. The system uses computer vision to detect lane lines and a differential drive controller to navigate the track. The entire project has been refactored to improve code quality, maintainability, and ease of use.

## Features

- **Refactored ROS Nodes:** The `camera`, `cv_lane_follower`, and `differential_drive` nodes have been rewritten using an object-oriented approach, making them more modular and easier to understand.
- **ROS Launch System:** The old `run_all_nodes.sh` script has been replaced with a ROS launch file, `launch/main.launch`, which provides a more robust and scalable way to manage the ROS nodes.
- **Configurable Parameters:** The system is now highly configurable through the `src/cv_lane_follower/config/config.yaml` file, allowing you to tune the behavior of the nodes without modifying the code.
- **Unit Tests:** The project now includes a suite of unit tests to ensure the correctness of the computer vision and motor control logic.
- **CI/CD Pipeline:** The project is equipped with a full CI/CD pipeline, powered by GitHub Actions, which automates the process of testing, building, and deploying the application.

## Getting Started

To get started with the project, you'll need to have Docker installed on your system.

### Building the Docker Image

First, clone the repository to your local machine:

```bash
git clone https://github.com/your-username/ros_duckie.git
cd ros_duckie
```

Next, build the Docker image using the provided `Dockerfile`:

```bash
docker build -t ros-duckie .
```

### Running the Project

Once the Docker image is built, you can run the project using the following command:

```bash
docker run -it --rm --net=host --privileged ros-duckie
```

This command will start the ROS nodes using the `main.launch` file. The `--net=host` flag is required for the ROS nodes to communicate with each other, and the `--privileged` flag is needed to access the camera and motor controller.

## Configuration

The behavior of the ROS nodes can be configured by modifying the `src/cv_lane_follower/config/config.yaml` file. This file contains parameters for the camera, lane follower, and drivetrain nodes.

### Camera Parameters

- `device`: The camera device index.
- `framerate`: The camera framerate.
- `width`: The width of the camera image.
- `height`: The height of the camera image.

### Lane Follower Parameters

- `save_images`: Whether to save debug images.
- `usb_mount`: The path to the USB mount for saving images.
- `lower_yellow`, `upper_yellow`: The lower and upper bounds for the yellow color mask.
- `lower_white`, `upper_white`: The lower and upper bounds for the white color mask.
- `canny_sigma`: The sigma value for the Canny edge detector.
- `hough_rho`, `hough_angle`, `hough_threshold`, `hough_min_line_length`, `hough_max_line_gap`: The parameters for the Hough line transform.

### Drivetrain Parameters

- `motor_addr`: The I2C address of the motor controller.
- `pulse_min`, `pulse_max`: The minimum and maximum pulse values for the motors.

## Testing

The project includes a suite of unit tests to ensure the correctness of the code. To run the tests, you'll need to have Python and the required dependencies installed.

First, install the dependencies:

```bash
pip install -r requirements.txt
pip install numpy opencv-python-headless
```

Then, run the tests from the root of the project:

```bash
PYTHONPATH=. python3 -m unittest discover tests
```

## CI/CD

This project uses GitHub Actions to automate the process of testing, building, and deploying the application.

- **CI Pipeline:** The CI pipeline is triggered on every push and pull request to the `main` and `develop` branches. It runs the unit tests and builds the Docker image to ensure that the code is always in a working state.
- **Deploy Pipeline:** The deployment pipeline is triggered on every push to the `main` branch. It builds the Docker image and pushes it to Docker Hub, making it easy to deploy the application to a device.