# ROS2 basics

If you haven't used ROS2, or you need to refresh your knowledge, it's recommended to read trough the [ROS2 documentation](https://docs.ros.org/en/foxy/index.html), especially about nodes,topics,services,actions in the [tutorials section](https://docs.ros.org/en/foxy/Tutorials.html).

*Although a lot of coding has been simplified in the driver, the core concepts should be known.*

## Setting Up a ROS 2 Workspace

Before creating a package, ensure you have a ROS 2 workspace set up. A workspace is a directory containing multiple packages and a build system.

### 1. Create a ROS 2 Workspace
Open a terminal and create a directory for your workspace:
```bash
mkdir -p ~/colcon_ws/src
```
Here:
- `colcon_ws` is the name of the workspace (you can change it to something else if preferred).
- `src` is the source directory where all packages will be stored.

## Creating a ROS 2 Package
In ROS 2, a "package" is the fundamental unit of organization for a project. It contains nodes, libraries, scripts, and configuration files necessary for developing and running your ROS-based application. Every piece of code in a ROS 2 workspace must be properly structured within a package to be built and executed correctly. The workspace follows a structured approach to manage dependencies, facilitate modular development, and streamline deployment.
### 2. Generate a New Package
Navigate to the `src` directory and use `ros2 pkg create` to generate a new package:
```bash
cd ~/colcon_ws/src
ros2 pkg create my_package --build-type ament_python --dependencies rclpy std_msgs
```
This command creates a package named `my_package` with:
- `ament_python` as the build system.
- `rclpy` (Python client library) and `std_msgs` (standard message definitions) as dependencies. You can later add more dependencies the the package.xml file.

### 3. Understanding the Package Structure
After creating the package, its structure will look like this:
```
my_package/
├── package.xml       # Package metadata and dependencies
├── setup.py          # Package build script
├── setup.cfg         # Configuration file
├── resource/         # Package resource files
├── my_package/       # Python module containing the node
│   ├── __init__.py   # Marks the module as a package
│   ├── my_node.py    # Python script for the ROS node
└── tests/            # Test directory (optional)
```

## Building the Package

### 4. Compile the Package
Return to the root of your workspace and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```
- `colcon` is the build tool used in ROS 2.
- `--packages-select my_package` ensures only `my_package` is built.
>[!NOTE]
>When using colcon build without --packages_select, all packages found in the directory will be build. To ignore a directory when building (maybe its not a ros code), place an empty file named `COLCON_IGNORE` in the directory's root. 

### 5. Source the Workspace
After building, source the workspace to use the package:
```bash
source install/setup.bash
```
To make this change permanent, add it to your `~/.bashrc` file:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Creating and Running a Node

### 6. Create a Simple Node (Python)
Inside `my_package/`, create a file `my_node.py`:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info("Hello from my_node!")

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7. Update setup.py
Modify `setup.py` to include the executable:
```python
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    #.....
    #info about the package, the creator etc..
    #.....
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main'
        ],
    },
)
```

In the lines left out you can add details about the packages such as the creator or the version of the code. (Unnecessary IMO)

### 8. Rebuild and Run the Node
Rebuild the package:
```bash
colcon build --packages-select my_package
```
Run the node:
```bash
ros2 run my_package my_node
```
You should see output confirming that the node is running.