
This section provides information about configuration options.

# Sensor fusion
Currently, the state estimation is provided by [robot_localization](https://github.com/cra-ros-pkg/robot_localization) package.

Configuration file location: [./config/ekf.yaml](./config/ekf.yaml)


Acording the information from documentation: 'The better the omnidirectional motion model matches your system, the smaller these values can be'

The process noise covariance matrix 

which will cause the filter to trust the incoming measurement more during correction



<!-- LIDAR nois is:
- Mean
- Normal diviation: -->
