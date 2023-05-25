# How to set up multi-machine ROS
## Process
### Set up the hosts file with IP aliases for convenience
Use **ifconfig** to find the ip address of both machines
**or**
The static ip from the hub as of 11/25/2020:
192.168.1.55    pi
192.168.1.52    companion

Add the ip addresses to their respective /etc/hosts file
It should look something like this:

127.0.0.1  localhost
127.0.1.1  {name of machine e.g. ubuntu}

192.168.1.AA companion {master}
192.168.1.BB pi {slave}

## Turn off firewall
$ sudo systemctl stop firewalld
$ sudo firewall-cmd --state
Need to fix this so we just open the needed ports

## Set up ROS values
On the master machine in one terminal:
$ export ROS\_HOSTNAME={master ip}
$ export ROS\_MASTER\_URI=http://{master ip}:11311
As far as I can tell ROS\_HOSTNAME and ROS\_IP perform the same function, but ROS\_HOSTNAME allows you to use the alias created in the hosts file above. ROS\_IP requires a written ip address. ROS\_MASTER\_URI will accept either.

In a second terminal:  
$ roscore  
It will not work if you run rpscore in the same terminal that you just set up and the program in a second  
Terminal 2 (No set up): roscore  
Terminal 1 (set up): rosrun {pkg} {file}  
roslaunch is also okay  


On the slave machine:
$ export ROS\_HOSTNAME=<slave ip>
$ export ROS\_MASTER\_URI=http://{master ip}:11311

### How to do permanently
These values can't just be put in a launch file.  Need to be part of bashrc?

## Installing firewalld
$ sudo apt install firewalld

(Should I uninstall this and use ufw?)
## Other useful commands
$ sudo systemctl enable firewalld

$ sudo firewall-cmd --state
$ echo $ROS\_MASTER\_URI
$ echo $ROS\_IP

Does this work?  It didn't work for me, but there may be more to it.
$ sudo firewall-cmd --add-port=11311/tcp --permanent

https://computingforgeeks.com/install-and-use-firewalld-on-ubuntu-18-04-ubuntu-16-04/

## ufw
Check status
$ sudo ufw status verbose

Allow in the right ports
$ sudo ufw allow 22
$ sudo ufw allow 11311




https://answers.ros.org/question/35603/ros_master_uri-change-it-and-save-it/
https://github.com/StanleyInnovation/segway_v3_network/issues/1
https://computingforgeeks.com/install-and-use-firewalld-on-ubuntu-18-04-ubuntu-16-04/