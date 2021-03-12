# w0303-ipc2

**Overall Functions**

1. manual control mir via joystick (ros)

2. ~~communication with mssql~~ 

> SQL Server is not supported on ARM architecture.

3. *todo: communication with ipc1


**Environment Setup**

hardware: raspberry pi 4b

platform: ubuntu 18.04

prerequisite:

1. ros melodic:
   
   (1) https://www.hackster.io/shahizat005/getting-started-with-ros-melodic-on-raspberry-pi-4-model-b-cbdec8

   (2) http://wiki.ros.org/melodic/Installation/Ubuntu
   
   (3) https://blog.csdn.net/kinglarry1/article/details/107155753
   
2. glog
   
   (1) https://github.com/google/glog

   (2) https://blog.csdn.net/qq_22634949/article/details/101718879

3. nanodbc

   (1) https://nanodbc.github.io/nanodbc/install.html#

4. odbc driver:
   
   (1) https://docs.microsoft.com/en-us/sql/connect/odbc/linux-mac/installing-the-microsoft-odbc-driver-for-sql-server?view=sql-server-ver15

   (2) https://stackoverflow.com/questions/43606832/r-unixodbcdriver-managercant-open-lib-sql-server-file-not-found/51266453

5. ros-joy-package

   (1)http://wiki.ros.org/joy/Tutorials
   
6. xbox controller driver

   (1)https://github.com/medusalix/xow
   
**Test**

(1) unixODBC & FreeTDS: https://help.interfaceware.com/kb/904

(2) odbcinst.ini config

      sudo gedit /etc/odbcinst.ini

      [FreeTDS]
      Description=FreeTDS Driver
      Driver = /usr/lib/x86_64-linux-gnu/odbc/libtdsodbc.so
      UsageCount= 1
      

(2) for wifi setting via terminal

      sudo gedit /etc/netplan/50-cloud-init.yaml
      sudo netplan apply
      systemctl daemon-reload
      
 (3) vnc for ubuntu 20.04: https://www.raspberrypi.org/forums/viewtopic.php?t=288769


**How to use**
1. connect mir wifi
2. check your ip address: 
            
         ifconfig
   
3. ros ip configuration
   
         gedit ~/.bashrc
         export ROS_MASTER_URI=http://192.168.12.20:11311
         export ROS_HOSTNAME=[your ip address]  //e.g. export ROS_HOSTNAME=192.168.12.254
         source ~/.bashrc
4. run the program 

         roscore
         rosrun joy joy_node
         rosrun mir_test teleop_mir
