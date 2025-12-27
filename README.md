# ROS2_DEVELOPMENTS


````md
# ROS 2 Commands Guide

This document covers the **complete, correct order** for creating packages,
building, running, and verifying ROS 2 Python and C++ nodes.

---

## 0. Create Workspace and `src` Directory (FIRST STEP)

You must create the workspace and `src` directory **before creating any package**.

```bash
mkdir -p ~/Documents/ROS2_DEVELOPMENTS/ros2_workspace_1/src
cd ~/Documents/ROS2_DEVELOPMENTS/ros2_workspace_1
````

---

## 1. Create ROS 2 Packages

Run these commands **inside the `src` directory**.

```bash
cd src
```

### Create C++ Package

```bash
ros2 pkg create my_cpp_pkg \
  --build-type ament_cmake \
  --dependencies rclcpp
```

### Create Python Package

```bash
ros2 pkg create my_py_pkg \
  --build-type ament_python \
  --dependencies rclpy
```

---

## 2. Build the Workspace

Always run this from the **root of the workspace**.

```bash
cd ~/Documents/ROS2_DEVELOPMENTS/ros2_workspace_1

# Build all packages
colcon build

# Build only specific packages (faster)
colcon build --packages-select my_py_pkg my_cpp_pkg
```

---

## 3. Source the Environment

After building, you must source the setup file in **every new terminal**.

```bash
source install/setup.bash
```

---
```md
## 4. Run the Nodes

Before running, make sure the node source files are created in the correct
locations for both Python and C++.

---

### Node File Locations

#### Python Node File
Create the Python node file at:

```

src/my_py_pkg/my_py_pkg/my_first_node.py

```

This file must:
- Contain the node implementation
- Be executable (`chmod +x my_first_node.py`)
- Be registered in `setup.py` under `console_scripts`

---

#### C++ Node File
Create the C++ node file at:

```

src/my_cpp_pkg/src/my_first_node.cpp

````

This file must:
- Contain the C++ node implementation
- Be added as an executable in `CMakeLists.txt`
- Be installed using `install(TARGETS ...)`

---

### Python Node

Run the Python node from `my_py_pkg`:

```bash
ros2 run my_py_pkg my_first_node
````

**Expected Output**

```
Node my_first_node has started.
Hello from my_first_node | Count: X
```

---

### C++ Node

Run the C++ node from `my_cpp_pkg`:

```bash
ros2 run my_cpp_pkg my_first_node
```

**Expected Output**

```
[INFO] ...: Node my_first_node has started.
[INFO] ...: Hello from my_first_node | Count: X
```

---

```
```


## 5. Verification Tools

### Check Running Nodes

```bash
ros2 node list
```

---

### Visualize Nodes

```bash
rqt_graph
```

**Tip**
If you don’t see your node:

* Uncheck **Hide: Debug**
* Select **Nodes only** in the `rqt_graph` UI

---

```
```
