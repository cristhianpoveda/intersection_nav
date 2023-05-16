# Intersection driving for Duckiebots DB19.

**NOTE:** Based on duckietown/template-ros.

This repository contains an intersection negotiation functionality for Duckiebots model DB19.

**Considerations:** 

* Road users: vehicles (duckiebtos) and pedestrians (duckies).
* Static road users.
* Intersections of 4 perpendicular wyas, planar ground and given map.
* Vehicles just perform 90° turns.
* Pedestrians only cross the street between adjacent blocks.
* Priority rules accoring to Colombia's National Transit Code.
* Negotiation refers to avoid collisions inside conflict zones without muli-agent communication.
* Duckiebot DB19 equipped with a Raspberry pi 3b+ (ARM32v7, 1.4 GHz, 1G RAM).

## 

### Preliminar instalations
To execute this system it is required to have a flashed Duckiebot model DB19, and to follow the [Laptop Setup](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_laptop/index.html) tutorial from Duckietown.

### Scene setup
It is required to locate all road users in the scene before executing the system.

## How to use it

### 1. Fork repository

Fork this repository to your local machine. Then move to the main directory.

`cd /local_path/intersection_nav/`

### 2. Build a docker image on the Duckiebot

`dts devel build -f -H ROBOTNAME.local` build docker image on the robot.

### 3. Run a container with the system

`dts devel build -f -H ROBOTNAME.local` run a docker container created from the system's image.

### 4. Configure the system's initial state

In another terminal run `docker -H ROBOTNAME.local exec -it dts-run-intersection_nav bash` to attach to the container.

Once entered to the container run: `/launch/intersection_nav` to start autoconfiguration. This command will show the message: **Setting up intersection negotiation!**.

### 5. Send a navigation goal

`rostopic pub /ROBOTNAME/chief_node/goal destination: DESIRED_DESTINATION_NUMBER`

After setting a destination, the system will wait for the current navigation mode to move robot toward the intersection. At this point the intersection will be being detected, and the system will take control of the robot once it has arrived to the desired stop distance from the intersection.
