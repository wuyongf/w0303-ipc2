# w0303-ipc2

**Overall Functions**

1. done: manual control mir via joystick (ros)

2. done: communication with mssql

3. *todo: communication with ipc1


**Environment Setup**

hardware: raspberry pi 4b

platform: ubuntu 20.04

prerequisite:

1. system installation

   workflow:

   (1) install ubuntu 20.04 server 

   (2) update && upgrade software to latest version

   (3) install ubuntu 20.04 desktop
   
   link: 
   
   (1) https://qengineering.eu/install-ubuntu-20.04-on-raspberry-pi-4.html
    
2. remote control(vnc) installation

   link: 
   
   (1) https://www.raspberrypi.org/forums/viewtopic.php?t=288769

3. ros installation
   
   link: 
   
   (1) http://wiki.ros.org/melodic/Installation/Ubuntu
   
   (2) https://blog.csdn.net/kinglarry1/article/details/107155753
   
4. xbox controller driver

   link: 
   
   (1) https://github.com/medusalix/xow
   
5. ros-joy-package installation

   link: 
   
   (1) http://wiki.ros.org/joy/Tutorials
   
6. glog installation (LOG)
   
   link:
   
   (1) https://github.com/google/glog

   (2) https://blog.csdn.net/qq_22634949/article/details/101718879

3. nanodbc intallation (MSSQL API)

   link:

   (1) https://nanodbc.github.io/nanodbc/install.html#
   
   Error:
         
   (1) if pi4 can not find a suitable odbc driver manager
         
   ** Need to manually install the unixODBC from source: http://www.unixodbc.org/ **

4.odbc driver intallation and usage
      
   Error explaination:
   
   >Normally, we will choose MSSQL ODBC Driver. 
   >Link:  https://docs.microsoft.com/en-us/sql/connect/odbc/linux-mac/installing-the-microsoft-odbc-driver-for-sql-server?view=sql-server-ver15
   >
   > However, MSSQL ODBC Driver is not supported on ARM architecture. 
   > 
   > We can use third-party ODBC Drivers instead, which are unixODBC and FreeTDS. 
   > 
   > For more info, please refer to https://help.interfaceware.com/kb/904
  
   workflow:
   
   (1) unixODBC & FreeTDS Installation: 
      
         https://help.interfaceware.com/kb/904

   (2) odbcinst.ini config

      **note: Need to locate which odbcinst.ini is read by ODBC Manager: unixODBC

         sudo gedit /etc/odbcinst.ini

      **a. For AMD processor** 
      
         [FreeTDS]
      
         Description=FreeTDS Driver
      
         Driver = /usr/lib/x86_64-linux-gnu/odbc/libtdsodbc.so
      
         UsageCount= 1

      **b. For ARM processor: Raspberry Pi 4 --- Ubuntu 18.04** 
      
         [FreeTDS]
      
         Description=FreeTDS Driver
      
         Driver = /usr/lib/aarch64-linux-gnu/odbc/libtdsodbc.so
      
         Setup = /usr/lib/aarch64-linux-gnu/odbc/libtdsS.so
      
         UsageCount= 1    

   (3) Usage: Connection String Format

      a. For MS ODBC Driver
      
         "Driver={SQL Server};Server=<ip_address>;Database=<database_name>;Uid=<user_id>;Pwd=<user_password>"
      
      b. For unixODBC and FreeTDS
   
         "Driver={FreeTDS};Server=<ip_address>;Port=1433;Database=<database_name>;Uid=<user_id>;Pwd=<user_password>"

**Ubuntu Configuration**

1. For wifi setting via terminal: please refer to "50-cloud-init.yaml" 

         sudo gedit /etc/netplan/50-cloud-init.yaml
         sudo netplan apply
         systemctl daemon-reload
      
2. For Debug:

   (1) CLion Prerequisite: 
   
         sudo apt install openjdk-11-jdk openjdk-11-jre
      
   (2) Clion Installation
   
3. For Backup:

   (1) Reference:
   
   https://thepihut.com/blogs/raspberry-pi-tutorials/17789160-backing-up-and-restoring-your-raspberry-pis-sd-card#:~:text=Using%20Windows&text=Once%20you%20open%20Win32%20Disk,backed%20up%20to%20your%20PC.
   
4. Auto run the program when the ubuntu system startup
   
   (1) reference: 
      
      https://askubuntu.com/questions/1151080/how-do-i-run-a-script-as-sudo-at-boot-time-on-ubuntu-18-04-server
      
      https://blog.csdn.net/liurunjiang/article/details/78595073?utm_source=blogxgwz0
   
   (2) create & configure a service
            
         cd /etc/systemd/system
         sudo touch ipc2.service
            
         sudo gedit ipc2.service
              
            [Unit]
            Description=ipc2 script

            [Service]
            ExecStart=/home/ubuntu/catkin_ws/src/mir_test/scripts/ipc2_startup.sh

            [Install]
            WantedBy=multi-user.target
            
   (3) create a script
   
         cd ~/catkin_ws/src/mir_test/scripts
         touch ipc2_startup.txt (for now)
         
   (4) enable the service
   
         systemctl enable ipc2.service
         
   (5) keep checking the status
   
         a. system setting --- startup application --- config --- command
         
         gnome-terminal -x journalctl -f -u ipc2.service

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
         rosrun mir_test mir_xbox_test_01
         
**Todo**

1. use static ip address. make sure the program is stable.
         
2. test ros bridge and the virtual joystick

3. 
