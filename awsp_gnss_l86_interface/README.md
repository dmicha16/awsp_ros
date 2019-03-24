Quectel GNSS L86 - ROS Interface
================================
ROS package to get GNSS information from Quectel GNSS L86 modules using UART communication.

Features
--------
**awsp_gnss_l86_interface** uses [**gnss_l86_lib**](https://github.com/EduPonz/gnss_l86_lib) to obtain messages from the module and publish real time GNSS information to the `gnss_data` topic. The message structure is:

```
float64 latitude
float64 longitude
int64 fix
int64 number_of_satelites
float64 horizontal_precision
float64 altitude
float64 timestamp
```

Getting Started
---------------
**awsp_gnss_l86_interface** has been developed and tested in November 2018 for ROS Kinetic using a Raspberry Pi 3 model B with Ubuntu 16.04 as Operating System. Other platforms, operating systems or distributions have not been tested and may requiere source code modification.

### Prerequisites
[**gnss_l86_lib**](https://github.com/EduPonz/gnss_l86_lib) relies on [wiringPi](http://wiringpi.com/) for establishing a serial connection with the GNSS module. The distributor offers different installation procedures. The one used for the development of this package was:

```
$ git clone git://git.drogon.net/wiringPi
$ cd wiringPi
$ git pull origin
$ ./build
```

This will complete the library instalation. For this case, the header files where included under `/usr/local/include/`. However, this has not been tested for other distributuions. In case the destination folder changes, the following line of `CMakeLists.txt` needs to be modify to reflect the installation directory.

```CMake
set(wiringPi_include /usr/local/include)
```

### Installation
For installing this package, just place it under the `src/` directory of your Catkin Workspace.

Usage
-----
The **awsp_gnss_l86_interface_node** can be run with:

```
$ rosrun awsp_gnss_l86_interface awsp_gnss_l86_interface_node <port>
```

Where `<port>` specifies the serial port where the module is attached. An example of this command would be:

```
$ rosrun awsp_gnss_l86_interface awsp_gnss_l86_interface_node /dev/serial0
```

Authors
-------
**awsp_gnss_l86_interface_node** has been developed by **Eduardo Ponz** for the use on the project [Project Name](project_repo).

License
-------
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
