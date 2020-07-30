# motors_interface
ROS Node to interact with motors

## Initialize ROS2 package

1. Create ros2 C++ package

    ```bash
    ros2 pkg create --build-type ament_cmake --node-name motors_interface motors_interface
    ```

2. Build package

    ```bash
    colcon build --packages-select motors_interface
    ```

3. Source the setup file

    ```bash
    . install/setup.bash
    ```

4. Use the package

    ```bash
    ros2 run motors_interface motors_interface
    ```

5. Customize package.xml


## PCA9685

it needs wiringPi installed in /usr/local/lib

1. git clone https://github.com/WiringPi/WiringPi
2. cd WiringPi
3. sudo ./build

pca9685 from https://github.com/Reinbert/pca9685


# Copyright and License

This software contains code licensed as described in LICENSE.