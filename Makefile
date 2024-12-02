# Variables
DOCKER_COMPOSE = docker compose
ROS_DISTRO = jazzy
SERVICE_NAME = ros2_${ROS_DISTRO}
WORKSPACE = ./ros2_ws

# Colors
COLOR_GREEN = \033[0;32m
COLOR_BLUE = \033[0;34m
COLOR_RESET = \033[0m

.PHONY: help launch_open_manipulator_x_controller launch_open_manipulator_x_rviz check_dependencies clean build build_marvin_control

# Show this help message
help:
	@cat $(MAKEFILE_LIST) | docker run --rm -i xanders/make-help
	@echo "$(COLOR_BLUE)Source workspace with \`. ros2_ws/install/setup.bash\`$(COLOR_RESET)"

##
## OpenManipulator targets
##

# Launch OpenManipulatorX controller
launch_open_manipulator_x_controller:
	ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

# Run OpenManipulatorX teleop keyboard
run_open_manipulator_x_teleop:
	ros2 run open_manipulator_x_teleop teleop_keyboard

# Launch OpenManipulatorX RViz
launch_open_manipulator_x_rviz:
	ros2 launch open_manipulator_x_description open_manipulator_x_rviz.launch.py 

##
## Local targets
##

# Check for missing dependencies
check_dependencies:
	@echo "$(COLOR_GREEN)Checking for missing dependencies...$(COLOR_RESET)"
	rosdep install -i --from-path src --rosdistro jazzy -y

# Clean ROS2 workspace
clean:
	@echo "$(COLOR_GREEN)Cleaning ROS2 workspace build files...$(COLOR_RESET)"
	rm -rf build install log

# Build ROS2 workspace
build:
	@echo "$(COLOR_GREEN)Building ROS2 workspace...$(COLOR_RESET)"
	colcon build

build_marvin_control:
	@echo "$(COLOR_GREEN)Building marvin control...$(COLOR_RESET)"
	colcon build --packages-select marvin_control --symlink-install