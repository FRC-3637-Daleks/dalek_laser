# dalek_laser

## Cloning
Make sure to use

    git clone --recursive <this repos git url>

so that submodules are also cloned.

## Building/running

This repo contains a ROS 1 workspace and a dockerfile and compose file for launching the workspace in a portable way.

Most of the containers in `compose.yaml` are created from the same image which is the one built from `Dockerfile`. The only difference is what command is launched in the container. Each container launches a different roslaunch, corresponding to different services which can be started and stopped independently.

To build everything, run

    docker compose build

And to run it use

    docker compose up -d

The -d lets it run in the background, otherwise you need to `^C` to gain control of the shell again but only after closing all the containers.

If you make changes to a specific service, you can relaunch that specific container by rebuilding it with

    docker compose build <service_name>
    docker compose up -d

This will build `<service_name>` again and only take down and relaunch that service when you run `up` again.

To do the above on the coprocessor from a separate machine (dev laptop), you want to specify your docker host as the coprocessor's hostname or IP.

    export DOCKER_HOST=tcp://10.36.37.xxx:2375

Take care in figuring out what the hostname of the coprocessor is or its IP address to use here.

## Building without docker

You can use the Dockerfile as a guide for building the ROS workspace outside of docker. Note that WPILIB is needed for frc_basics network tables bridge, and needs to be installed with the cmake instructions. You can follow the example in the Dockerfile for this too.

To build the ROS workspace, just `cd ros_ws` and `catkin_make`. Of course, you will need to install dependencies, as you can see this is best done with

    rosdep update
    rosdep install --from-paths src/ -y --ignore-src

## Services

### rosmaster

This is a small container which just runs the master and is guaranteed to be brought up before all other containers. This allows nodes across all containers to see one another. Note the environment variables being passed to every container which indicates `rosmaster` as the master URI.

If you are on the host machine you can use the same master URI to access the ROS nodes, but this won't work if you're on a different machine (I think). Instead, prefer to log in with portainer or ssh to do ROS things on the host machine.

The master may be changed to use host networking instead in the future to facilitate connecting ROS remotely (making it easier to use RVIZ in the pit for example).

### scanner

This is unaltered from the rplidar_ros git repo for now, but I'm keeping it open as a service in case we add features/filters/launch parameters to make it fancier.

This service just runs the scanner node, attaching the necessary hardware and publishing the scans to /scan.

### nt_bridge

This service is the network tables to ROS bridge I found that suited our needs best. In the future it should be changed to a submodule and then their repo should be forked with our modifications.

For now I've just added a helper launch file which launches the other 2 launch files together.

The key feature is the taking of data from network tables and populating an odom and twist topic in ROS.
Specifically, you should see an /frc/drivetrain/nt2ros/odom topic being published to.

*NOTE*: Currently it was tested with sim, notice that the hostname (host.docker.internal) chosen for the network tables server is a special internal hostname which lets containers see the simulation running on the host machine.
The hostname launch parameter needs to be changed to `10.36.37.2`(roborio's static IP). Do not change the `extra_hosts` section.

### bagger

This service is a very simple launch file for rosbag right now which collects bag files in 5 minute increments up to 3 at a time on a special volume called `bags`. To access the data on the host, look in the `/var/lib/docker/volumes/dalek_laser_bags` directory (need to be root). Alternatively you can add a service which has this volume mounted as well and run that interactively or something. The pipeline for bag retrieval and playback will be fleshed out in the future.

### portainer

portainer is a web frontend for managing docker containers and its especially useful when working with docker running on remote systems like with the coprocessor.
Navigating to the coprocessor's hostname with port 9443 (subject to change to allow for connecting over FMS) allows you to view a nice dashboard with all the active containers. Here you can view their logs, see any errors, and even open an interactive shell in the browser in any container you choose to interact with it.

### dnsresolver

This is a nifty little docker image I found which magically makes the docker service names appear as hostnames on the host system so you can connect to them with ROS on the host machine.
Mostly useful for simulation.

## TODO

- [ ] Add localization nodes to workspace and service.
- [ ] Add docker compose profiles to enable running different services depending on context (simulation, map creation, bag playback, etc)
- [ ] Implement some form of map creation using SLAM techniques
- [ ] Implement a map editor for cleaning up SLAM maps
- [ ] Feed localization back to ros2nt topics to use on the robot
- [ ] Implement dynamic obstacle detection/identification
- [ ] Implement multi-scanner processing
