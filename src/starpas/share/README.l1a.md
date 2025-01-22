# STARPAS l1a data
This data stores the level 1a product from the STAbilized Radar Platform Alignment Sensor (STARPAS). This is the raw data from the sensor stored with added meta data in netcdf format.

## Coordinate system:
Sensor observation are on a karthesic coordinate system. Relative to the STARPAS-BOX the axis are as follows:

top view  | side view
╔═══════╗ |
║ x     ║ | ╔══════╗
║ ↑     ║ | ╠══════╣
║zX → y ║ | ║ O O  ║ → y
╚═╦═╦═══╝ | ╚══════╝
cables    |   ↓z

## Dimensions:
* *time*: 20Hz resolution

## Variables:
* *lat*: latitude (degree North), from GPS-Module
* *lon*: longitude (degree East), from GPS-Module
* *gps-speed*: platform speed over ground (ms-1), from GPS-Module
* *altitude*: GPS antenna altitude (m), from GPS-Module
* *gps-satellites*, *gps-fixquality*: number of available GPS satelltes and fix quality,  from GPS-Module
* *ax*, *ay*, *az* (mg): acceleration to sensor axis, Accelerometer
* *gx*, *gy*, *gz* (deg s-1): angular velocity around sensor axis, Gyroscope
* *mx*, *my*, *mz* (uT): magnetic field at sensor axis, Magnetometer,
* *q0*, *qx*, *qy*, *qz* (-): quarterion rotation from internal sensor algorithm
* *roll*, *pitch*, *yaw* (deg): euler angles from internal sensor algorithm
* *temperature* (K): sensor temperature
* *pressure* (Pa): air pressure
