# STARPAS l1b data
This data stores the level 1a product from the STAbilized Radar Platform Alignment Sensor (STARPAS). 
This is like raw data but euler angles are reprocessed with the Madgwick Algorithm (https://pypi.org/project/imufusion/). For settings and configuration see the netcdf global attributes.

## Coordinate system:
Sensor observation are on a karthesic coordinate system. Relative to the STARPAS-BOX the axis are as follows:
> Note: The x-Axis printed on the sensor board is the direction of the Accelerometer-Chip axis, but internally all sensor axis are transformed to obey the coordinate system given below. 

[Sentral README](https://github.com/gregtomasch/EM7180_SENtral_Calibration?tab=readme-ov-file#sentral-coordinate-system-definition)

top view  | side view
╔═══════╗ |   
║ x     ║ | ╔══════╗
║ ↑     ║ | ╠══════╣
║zX → y ║ | ║ O O  ║ → y
╚═╦═╦═══╝ | ╚══════╝
cables    |    ↓z

## Dimensions:
* *time*: 20Hz resolution

## Variables:
* *lat*: latitude (degree North), from GPS-Module
* *lon*: longitude (degree East), from GPS-Module
* *gps-speed*: platform speed over ground (ms-1), from GPS-Module
* *altitude*: GPS antenna altitude (m), from GPS-Module
* *gps-satellites*, *gps-fixquality*: number of available GPS satelltes and fix quality,  from GPS-Module
* *roll*, *pitch*, *yaw* (deg): euler angles from Madgwick alorithm
* *temperature* (K): sensor temperature
* *pressure* (Pa): air pressure
