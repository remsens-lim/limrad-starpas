# STARPAS l1b data
This data stores the level 1a product from the STAbilized Radar Platform Alignment Sensor (STARPAS). 
This is like raw data but euler angles are reprocessed with the Madgwick Algorithm (https://pypi.org/project/imufusion/). For settings and configuration see the imufusion group in the netcdf files.

## Coordinate system:
Sensor observation are on a karthesic coordinate system. Relative to the STARPAS-BOX the axis are as follows:

top view  | side view
╔═══════╗ |   ^z
║ y     ║ | ╔══════╗
║ │     ║ | ╠══════╣
║zo─> x ║ | ║ O O  ║ -> x
╚═╦═╦═══╝ | ╚══════╝
cables    |

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
