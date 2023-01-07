DISCONTINUATION OF PROJECT

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.  

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project.  

Contact: webadmin@linux.intel.com
# Mavlink Vehicle Abstraction Layer #

[![Build Status](https://travis-ci.org/01org/camera-streaming-daemon.svg?branch=master)](https://travis-ci.org/01org/camera-streaming-daemon) <a href="https://scan.coverity.com/projects/01org-mavlink-vehicles">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/11939/badge.svg"/>
</a>

A Mavlink wrapper dedicated to the most common messages that are exchanged
between an air vehicle and a ground station.

## Requirements ##
To generate MAVLink headers during the build:
  * Python 2.7+
  * python future
  
The following requirements are only needed for running the tests:

  * MavProxy (https://github.com/ArduPilot/MAVProxy)
  * ArduPilot (https://github.com/ArduPilot/ardupilot)

## Build and Install ##

1. Make sure you have initialized the submodules of this project:

    ```
    git submodule update --init --recursive
    ```

2. Create a build folder and compile using CMAKE as follows:

    ```
    mkdir build
    cd build
    cmake ..
    make
    ```

## Run Tests ##

1. Open a second terminal and run Ardupilot:

    ```
    cd ${ARDUPILOTDIR}/build/sitl/bin
    ./arducopter-quad --model x
    ```

2. In a third terminal, run Mavproxy:

    ```
    mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14557 --streamrate -1
    ```

3. Run tests:

    ```
    build/tests/tests
    ```

    A series of tests will be executed and their results (OK/FAIL) will be
    displayed on the screen.
