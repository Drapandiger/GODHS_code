# Language-Exclusive Mobile Manipulation for Efficient Object Search in Indoor Environments 

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Project Page](https://img.shields.io/badge/Project-Page-a)](https://drapandiger.github.io/GODHS/)

> Code for GODHS\
> [**Liding Zhang***](https://scholar.google.com/citations?user=AMFFKhkAAAAJ&hl=en), [**Zeqi Li***](mailto:zeqi.li@tum.de), [**Kuanqi Cai***](https://scholar.google.com/citations?user=3Y9wVfMOtP4C&hl=en), [**Zhenshan Bing**](https://www.ce.cit.tum.de/air/people/zhenshan-bing-drrernat/), [**Alois Knoll**](https://www.ce.cit.tum.de/en/air/people/prof-dr-ing-habil-alois-knoll/)\
> Technical University of Munich\
> Submitted to IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2025) 

## Approach

![GODHS Architecture](https://github.com/Drapandiger/GODHS/raw/main/static/images/fig2.png?raw=true)

<div align="center">
  <strong>Complete Architecture of the GODHS System</strong>
</div>

<br>

**For detailed methodology**, please explore comprehensive implementation details at our **[GitHub Page](https://drapandiger.github.io/GODHS/)**.

## Quick Start

This section assumes you have the following prerequisites installed on your system:

- **[Ubuntu 20.04 Focal](https://releases.ubuntu.com/focal)**
- **[ROS Noetic](http://wiki.ros.org/noetic)**
- **[MoveIt!](https://moveit.github.io/moveit_tutorials/)**
- **[Navigation Stack](http://wiki.ros.org/navigation)**
- **[Isaac Sim 4.0](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)**
- **[ollama](https://ollama.com/)**

### Environment Setup

#### 1. **Clone the Repository**  
   ```bash
   git clone https://github.com/Drapandiger/GODHS_code.git
   cd GODHS_code
   ```

#### 2. **Install Dependencies**  
   This project is based on ROS and MoveIt!. Please ensure that the appropriate versions are installed on your system. Example installation steps (Ubuntu 20.04, ROS Noetic):
   ```bash
   sudo apt update
   sudo apt install ros-noetic-moveit ros-noetic-navigation
   # Install other dependencies as required by specific modules
   ```

#### 3. **Build the Project**  
   Add this repository to your ROS workspace and compile:
   ```bash
   cd ~/catkin_ws/src
   ln -s /path/to/GODHS_code .
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
### Run the system

#### 1. Isaac Sim Setup

- **Download and Install Isaac Sim 4.0:**  
  Follow the official instructions from NVIDIA to install Isaac Sim 4.0.

- **Launch the Scene:**  
  You can either directly import the `robot_flat.usd` file or start with the `flat.usd` scene and then import the `rbkairos.usd` model.  
  For further guidance, refer to the [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html).

- **Run the simulation:**
  Click the "Start" buttom after importing the scene files.

#### 2. Start ollama

- **Basic Start:**  
  Run the following command to serve ollama:
  ```bash
  sudo ollama serve
  ```

- **Run Model:**  
  Run the following command to start a model:
  ```bash
  sudo ollama run <model name>
  ```

- **Custom Model Location (if local model size is large):**  
  If your local model is too large, store it in a custom location and launch ollama using:
  ```bash
  sudo OLLAMA_MODELS=/path/to/ollama/model ollama serve
  ```

- **Handling Old Instances:**  
  If you encounter a prompt indicating an already running instance, stop the old ollama service with:
  ```bash
  systemctl stop ollama
  ```

#### 3. Launch ROS Navigation

- **Launch rbkairos Navigation Stack:**  
  Make sure you have sourced your ROS workspace. Then, launch the navigation module using:
  ```bash
  roslaunch rbkairos_nav nav.launch
  ```

#### 4. Launch MoveIt! Execution

- **Run MoveIt! for Motion Planning:**  
  Execute the following command to start the MoveIt! execution for the Franka robot in Isaac Sim:
  ```bash
  roslaunch isaac_moveit franka_isaac_execution.launch
  ```

#### 5. Run the Main Application

- **Run the main file of Target Search System:**
  After launching the necessary ROS modules and MoveIt! execution, proceed to run the main application located in the `find_target/scripts` directory. Make sure you have sourced your ROS workspace (e.g., `source ~/catkin_ws/devel/setup.bash`) before executing the following commands:
  ```bash
  cd find_target/scripts
  python3 main.py
  ```

  This will start the target search process as implemented in the `main.py` script.

## TODO

- [ ] **Improve Motion Planning Module**
  - Further optimize the coordinated planning algorithm between the chassis and end-effector.
- [ ] **Enhance Semantic Parsing**
  - Leverage multi-modal foundation models and fine-tuned language models.
- [ ] **Expand Experimental Platforms**
  - Validate system performance on more real-world robot platforms and simulation environments.
- [ ] **Community Contributions**
  - Welcome suggestions for improvements in ROS integration, data generation, and experimental modules.

## Citation

If you find the code useful, please cite:

```bibtex
Coming soon...
```

## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).