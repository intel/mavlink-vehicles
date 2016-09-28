# Mavlink Vehicle Abstraction Layer #

A Mavlink wrapper dedicated to the most common messages that are exchanged
between an air vehicle and a ground station.

## Requirements ##
    * Mavlink (https:https://github.com/mavlink/mavlink)
    * MavProxy (https://github.com/ArduPilot/MAVProxy)
    * Ardupilot (https://github.com/ArduPilot/ardupilot)

## Build and Install ##

1. Make sure you have all dependencies properly installed in your system.

2. Check if $PKG_CONFIG_PATH points to the location of the mavlink pkg-config
configuration file. If you have installed Mavlink using its cmake defaults,
$PKG_CONFIG_PATH should be set with the following command:

    ```
    export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig
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

3. Then simply open the following executable to run a simple test.

    ```
    build/tests/tests
    ```

    This test connects to an active vehicle and displays its current state
    variables every once in a while.
