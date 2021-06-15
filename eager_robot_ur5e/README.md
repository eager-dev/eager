## UR5 arm (Universal Robots)

This package supports the following engines:
- Webots
- Pybullet
- Gazebo
- Reality (UR5 cb2)

It requires the following external packages that cannot be installed via `rosdep`:

- `git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git`
- `git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git`