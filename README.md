# ROS 2 Camera Package

This ROS 2 package helps you simulate a camera by publishing images from a folder. It's great for testing things that need a camera feed!

## Quick Start

Follow these steps to get it running:

1.  **Get the Code:**
    ```bash
    git clone git@github.com:AlfrinP/ros_camera_package.git
    cd ros_camera_package
    ```

2.  **System Ready?**
    * Ubuntu 22.04
    * ROS 2 (already installed)
    * Python 3 (already installed)

3.  **Setup Environment:**
    * Create a virtual environment:
        ```bash
        python3 -m venv .venv
        ```
    * Activate it:
        ```bash
        source .venv/bin/activate
        ```
        (On Windows, use `.\.venv\Scripts\activate`)

4.  **Install Stuff:**
    * Install Python packages:
        ```bash
        pip install -r requirements.txt
        ```

5.  **Add Your Photos:**
    * Create a folder named `photos` inside `camera_package`.
    * Put all your simulation images (like `.jpg`, `.png`) into this `photos` folder.
    * Example path: `ros_camera_package/src/camera_package/photos/`

6.  **Build the Package:**
    ```bash
    colcon build --packages-select camera_package
    ```

7.  **Activate ROS 2:**
    ```bash
    source install/setup.bash
    ```
    (Do this after activating your .venv)

8.  **Run It!**
    ```bash
    ros2 run camera_package camera_node
    ```

Now, your node will start publishing images! All images will be published at the path `/camera/image_raw` at 30 FPS.