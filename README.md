# dalek_laser

## Cloning
Make sure to use

    git clone --recursive <this repos git url>

so that submodules are also cloned.

## Building/running

This repo contains a ROS 1 workspace and a dockerfile and compose file for launching the workspace in a portable way.

Most of the containers in `compose.yaml` are created from the same image which is the one built from `Dockerfile`. The only difference is what command is launched in the container. Each container launches a different roslaunch, corresponding to different services which can be started and stopped independently.

To build for the robot

    docker compose --profile matchplay build

And to run it use

    docker compose --profile matchplay up -d

The -d lets it run in the background, otherwise you need to `^C` to gain control of the shell again but only after closing all the containers.

If you make changes to a specific service, you can relaunch that specific container by rebuilding it with

    docker compose --profile matchplay build <service_name>
    docker compose --profile matchplay up -d

This will build `<service_name>` again and only take down and relaunch that service when you run `up` again.

To do the above on the coprocessor from a separate machine (dev laptop), you want to specify your docker host as the coprocessor's hostname or IP.

    export DOCKER_HOST=tcp://10.36.37.xxx:2375

or over ssh

    export DOCKER_HOST=ssh://orangepi@10.36.37.xxx

Take care in figuring out what the hostname of the coprocessor is or its IP address to use here.
NOTE: the orangepi's address should be 10.36.37.12. If you are unable to ping this address, make sure you plug it into the ethernet port closest to the USB-C power port.
May need to unplug and replug.

Also, the latest commit of this repo should be cloned on the HOST OS of the orangepi and it should be launchable from there.

### Running with `.env` files

Depending on the situation, you will want to run different services with different arguments. For example, when connected to a robot you want to make sure the network table server is pointed at the rio, but when running locally you may want to point it at localhost where you have simulation running.

To configure this, `.env` files are used to specify "profiles" to run and set certain variables for running.

Here are the available profiles current:
 - lidar: runs the scanner and localization algorithms depending on the lidar scanner
 - cameras: runs photonvision

Here are the variables used in the compose file
 - `NT_SERVER`: by default it will search localhost, but if this value is set, specifies the address of the network tables server. On a real robot, should be `10.36.37.2`
 - `ROS_IP`: if left out, ROS will use dalekpi hostname. Setting this to `10.36.37.12` allows for querying ROS from off the robot.

Here are the `.env` files currently available to use
 - `robot.env`: this is the configuration that should be used when deploying to the actual robot.

 To run with a specific env file, use the following command

    docker compose --env-file <filename>.env up -d

## Building without docker

You can use the Dockerfile as a guide for building the ROS workspace outside of docker. Note that WPILIB is needed for frc_basics network tables bridge, and needs to be installed with the cmake instructions. You can follow the example in the Dockerfile for this too.

To build the ROS workspace, just `cd ros_ws` and `catkin_make`. Of course, you will need to install dependencies, as you can see this is best done with

    rosdep update
    rosdep install --from-paths src/ -y --ignore-src

## Deploying to a fresh orangepi

These instructions are specific to setting up an orangepi with this stack as it currently exists.

### Flash ubuntu to an SD card
Use balena etcher to image a new SD card for the orangepi. Use one of the ubuntu server or desktop images found here: http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-Pi-5-plus.html

### Boot up orangepi and log-in
Connect your orangepi to a LAN with an internet connection (home router over ethernet is easiest). Use the ethernet port FURTHEST from the USB-C power port.

Power it over USB-C with a standard USB-C 5V power supply.

Hopefully, you should be able to ssh into it with its default hostname

```
ssh orangepi@orangepi5plus
```

Also try `orangepi5plus.local` or `orangepi5plus.lan` if that doesn't work.

If no hostname works, then try to log into your router and see the device list with IP addresses. If this isn't possible, you could try to use nmap or something to probe the local network for devices.

### First time configuration

Once you log in over ssh, run `sudo orangepi-config`.
Go through the settings and make changes as you desire. I usually set the timezone to US Eastern.
Be sure to enable ssh and avahi dns service.

More importantly, update the hostname to be `dalekpi` or whatever you want.

Most importantly, under the IP settings under network, set a *static* IP on the interface that is not currently plugged in (the empty ethernet port). Put a static IP `10.36.37.12` with netmask `255.255.255.0` and default gateway `10.36.37.1`. Make sure to always plug the pi into the robot using THIS port (the port closest to the USB-C port).

This allows you to access it via this static IP and not have to fuss with hostnames.

### Add ssh keys to ease future logins

Edit the `~.ssh/authorized_keys` file and paste in your public key so you don't need to use a password to login and can use ssh to remote deploy docker containers.

```
vim ~/.ssh/authorized_keys
<paste in your public key>
```

### Install docker packages

We need 2 packages, docker-buildx-plugin and docker-compose-plugin

```
sudo apt update
sudo apt install docker-buildx-plugin docker-compose-plugin
sudo apt upgrade # may as well do this too
```

### Clone this repo

In the home directory
```
git clone --recursive https://github.com/FRC-3637-Daleks/dalek_laser.git
```

### Build the containers

Simply cd into dalek_laser and build it as normal

```
cd dalek_laser
docker compose --profile matchplay --env-file robot.env build
```

### First time run

Run it for the first time with the default configuration. This should pull containers as well.
```
docker compose --profile matchplay --env-file robot.env up -d
```

### Enable docker so containers come up on startup
the docker daemon must be enabled in systemd so that whenever the pi powers up the containers start on their own

```
sudo systemctl enable docker
```

### Customize configuration

Files in the dalek_laser_configs directories will be mounted to a "custom" config directory in the workspace depending on the service. See the `robot_custom.env` as an example.

If you want to customize those configs, first copy the default/original into the relevant dalek_laser_configs directory from the dalek_laser source directory. Then feel free to edit them. Finally, modify your `.env` file to point the appropriate launch files at the custom configs via the relevant environment variable (see `robot_custom.env` to see which exist).

### Optional setup

The orangepi has no internal clock and relies on the network for an accurate time. It uses a package called fake-hwclock to save the time between reboots if it comes up without connecting to a network with access to the time.
However, it only updates this saved clock every hour. To fix this, we can add a cron job to save the time every minute.

Add the following to a file in `/etc/cron.d`

```
* * * * * root fake-hwclock save
```

## Services

### rosmaster

This is a small container which just runs the master and is guaranteed to be brought up before all other containers. This allows nodes across all containers to see one another. Note the environment variables being passed to every container which indicates `rosmaster` as the master URI.

If you are on the host machine you can use the same master URI to access the ROS nodes, but this won't work if you're on a different machine (I think). Instead, prefer to log in with portainer or ssh to do ROS things on the host machine.

The master may be changed to use host networking instead in the future to facilitate connecting ROS remotely (making it easier to use RVIZ in the pit for example).

### scanner

This is unaltered from the rplidar_ros git repo for now, but I'm keeping it open as a service in case we add features/filters/launch parameters to make it fancier.

This service just runs the scanner node, attaching the necessary hardware and publishing the scans to /scan.

There is also a static transform published from this node which MUST be calibrated to match where the scanner is mounted to the robot.

### camera

This container runs the camera which it looks for at `/dev/video0`. You may need to `sudo chmod 666 /dev/video0` to give it access but this should only happen when the USB port changes, not every restart.

This will publish both raw and rectified images on the `/arducam/*` topics (use `rostopic list` to see). You should always use the `compressed` versions when trying to view them. Note that it runs only at 5fps but with max resolution.
The theory here is that correction against the apriltags doesn't need to occur often or recently, just regularly.

The camera is calibrated with files in the container, as well as configured (exposure, brightness settings, etc).
These settings can be changed by modifying the config in the dalek_apriltag folder and rebuilding.
To experiment with different camera setting stuff, you can ssh into the pi and use
```
v4l2-ctl --list-controls # show all avaialble controls and current settings
sudo v4l2-ctl -c <parameter>=<value>
```

Some of these seem to have no effect but the ones I set in `arducam_controls.yaml` have some effect at least.

Alternatively, you can make a new file outside the container and create a *bind mount* to inside the container, and then pass in a file argument pointing at the bound file so that you don't need to rebuild.

Recalibration was done with http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration which you can run locally pointing your ROS_MASTER at the robot while tethered, or by connecting the camera over USB to a local machine and running the publisher there as well.

### apriltag

This container runs the apriltag algorithm and publishes a pose with covariance to `/apriltag/pose`. It also publishes frames for apriltags it sees as well as tag bundles

Tag bundles are specified in the `reefscape.yaml` config in the `dalek_apriltag/config` folder. They define the location of apriltags relative to some common landmark.
The `full_field` bundle is used for localization, all tags are defined relative to the field origin.
NOTE: this was a pain to make, because FIRST's conventions for angles are different from ROS'. If you need to make fixes here, just adjust the positions, NOT the quaternions.

Another very important piece is the static transform published in `apriltag_detector.launch`. The `base_to_camera` transform specifically. This defines the location of the camera relative to the robot center.
Currently it's defined as 30cm forward, centered left/right, and 15cm off the floor. This is SURELY wrong and MUST be corrected or it WONT work.
Consult the CAD, or make measurements to the tip of the lens to get this transform.

This also includes a quaternion which describes the camera as facing directly in front of the robot.

Adjusting this correctly is much harder as the camera frame has Z facing out of the lens as opposed to X, so any angle adjustment has to be made on top of this one.
Pray that the camera is actually mounted perfectly straight (it's not).

Ask a mentor to ask another mentor for help producing the corrected value if an angle offset is discovered. Preferrably, level the camera mechnanically so this value is right.

### pose_fusion

This container takes in the information from odometry and the apriltag poses and fuses it together with a Kalman filter. Sadly this hasn't been tuned at all. The covariance values for the apriltags and for the odometry are completely guessed.
If you read the code for for the apriltag publisher in dalek_apriltag/src you can adjust some parameters to try and get better values.

This really needs some in person tuning.
You can see all the different poses and their covariances being fused by opening the RVIZ file in this directory while `ROS_MASTER_URI` is pointed at the robot. You can see the tags it sees, the odometry, the way it adjusts over time.

In the robot code there is a command now to IGNORE vision measurements and just go off odom. By making it a command it can be bound to a driver button to use in a match to bail on the vision.

### nt_bridge

This service is the network tables to ROS bridge I found that suited our needs best. In the future it should be changed to a submodule and then their repo should be forked with our modifications.

For now I've just added a helper launch file which launches the other 2 launch files together.

The key feature is the taking of data from network tables and populating an odom and twist topic in ROS.
Specifically, you should see an /frc/drivetrain/nt2ros/odom topic being published to.

*NOTE*: Currently it was tested with sim, notice that the hostname (host.docker.internal) chosen for the network tables server is a special internal hostname which lets containers see the simulation running on the host machine.
The hostname launch parameter needs to be changed to `10.36.37.2`(roborio's static IP). Do not change the `extra_hosts` section.

### bagger

This service is a very simple launch file for rosbag right now which collects bag files in 5 minute increments up to 3 at a time on a special volume called `bags`. The `bags` volume is mounted to a USB drive (`/dev/sda1`) by default. If no usb-drive is plugged in, this service won't run. To access the data on the host, look in the `/var/lib/docker/volumes/dalek_laser_bags` directory (need to be root). Alternatively you can add a service which has this volume mounted as well and run that interactively or something. The pipeline for bag retrieval and playback will be fleshed out in the future.

Note, the camera topic is not recorded by default. To record it, the `record_camera` parameter must be set to true in `compose.yaml` and it will record the compressed camera image to a bag by a different name.

You will run out of space often if you enable this.

### web_dashboard

This service is a simple web-based ROS viewer node that allows for quickly seeing the state of the system (as well as the camera image) very rapidly.

The web dashboard can be accessed at the robot URL on port 5803 (`http://10.36.37.12:5803`)

### filebrowser

This service is a simple service to access the above bag files for quick download and deletion.
Its a simple web server which can be access at port 5802 on the pi's address. (`http://10.36.37.12:5802`).
Username: admin
Password: admin

### portainer

portainer is a web frontend for managing docker containers and its especially useful when working with docker running on remote systems like with the coprocessor.
Navigating to the coprocessor's hostname with port 5801 allows you to view a nice dashboard with all the active containers. Here you can view their logs, see any errors, and even open an interactive shell in the browser in any container you choose to interact with it.

Its a web server which can be accessed at port 5801 on the pi's address. (`https://10.36.37.12:5801`). Note that you need to tell the web browser to be okay with using unsafe https connection.
This is good for starting and stopping containers on the field as well as accessing a shell into one of the containers in a pinch.

Username: admin
Password: team3637team3637


## TODO

- [x] Add localization nodes to workspace and service.
- [x] Add docker compose profiles to enable running different services depending on context (simulation, map creation, bag playback, etc)
- [x] Implement some form of map creation using SLAM techniques
- [ ] Implement a map editor for cleaning up SLAM maps
- [x] Feed localization back to ros2nt topics to use on the robot
- [ ] Implement dynamic obstacle detection/identification
- [ ] Implement multi-scanner processing

## Usefull commands
```
# Connect local ROS to orange pi
export ROS_MASTER_URI=http://10.36.37.12:11311

#Run to build and run
sudo docker compose build
sudo docker compose --profile (name) up

# on the orangepi, frees up space
docker builder prune
docker image prune

#Run once ran
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
or
rosrun rviz rviz -d /path/to/config.rviz

rosbag play --clock -l /path/to/bag/name.bag # play before rviz

#Run if need to make
catkin_make

```
