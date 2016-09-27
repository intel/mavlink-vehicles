/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "mavlink_vehicles.hh"
#include "tests.hh"

namespace tests
{
struct testcase {
    std::string name;
    std::shared_ptr<tests::common_test> test_object;
    int time_s;
};

std::vector<testcase> testcases = {
    {"connection", std::make_shared<tests::connection_test>(), 10},
    {"conversion", std::make_shared<tests::conversion_test>(), 0},
    {"mission", std::make_shared<tests::mission_test>(), 30}};
}

void print_result(std::string test_name, bool result)
{
    if (result) {
        std::cout << "[tests] [" << test_name << "] PASSED" << std::endl;
    } else {
        std::cout << "[tests] [" << test_name << "] FAILED" << std::endl;
    }
}

int main()
{
    for (tests::testcase tc : tests::testcases) {
        std::cout << "[tests] [" << tc.name << "] BEGIN" << std::endl;
        print_result(tc.name, tc.test_object->run(tc.time_s));
    }

    return 0;
}

namespace tests
{

namespace defaults
{
const uint16_t mavproxy_port = 14557;
const double arrival_max_dist_m = 0.5;
const double comp_tol = 0.2;
}

common_test::common_test()
{
}

bool common_test::connect()
{
    // Open socket
    this->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        perror("[common test] Error opening socket");
        return false;
    }

    // Allow socket to bind to a port which remains in TIME_WAIT. A port
    // remains in TIME_WAIT for some seconds after close() is called in a
    // socket that is bound to it.
    int reuse_port = true;
    if (setsockopt(this->sock, SOL_SOCKET, SO_REUSEADDR, &reuse_port,
                   sizeof reuse_port) == -1) {
        perror("[common test] Error setting socket as reusable");
        goto cleanup_err;
    }

    // Bind socket to port
    this->local_addr.sin_family = AF_INET;
    this->local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    this->local_addr.sin_port = htons(defaults::mavproxy_port);
    if (bind(this->sock, (struct sockaddr *)&this->local_addr,
             sizeof(struct sockaddr)) == -1) {
        perror("[common test] Error binding socket to port");
        goto cleanup_err;
    }

    // Attempt to make it non blocking
    if (fcntl(this->sock, F_SETFL, O_NONBLOCK) == -1) {
        perror("[common test] Error setting socket as nonblocking");
        goto cleanup_err;
    }

    // Instantiate mav_vehicle
    this->mav = std::make_shared<mavlink_vehicles::mav_vehicle>(this->sock);

    // Initialize mavlink_vehicles update thread
    this->mav_update_thread_run = true;
    this->mav_update_thread = std::thread(&common_test::update, this);

    return true;

cleanup_err:
    close(this->sock);
    return false;
}

common_test::~common_test()
{
    if (this->mav_update_thread_run == true) {
        this->mav_update_thread_run = false;
        this->mav_update_thread.join();
        close(this->sock);
    }
}

void common_test::update()
{
    while (this->mav_update_thread_run) {
        this->mav->update();
        std::this_thread::yield();
    }
}

bool connection_test::run(int time_s)
{
    // Try to connect to vehicle
    if(!this->connect()) {
        return false;
    }

    // Store test start time
    std::chrono::time_point<std::chrono::system_clock> test_start_time =
        std::chrono::system_clock::now();

    // Wait for vehicle initialization returning false if timed out
    while (!this->mav->is_ready()) {
        int time_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - test_start_time).count();
        if (time_elapsed > time_s * 1e3) {
            return false;
        }
    }

    // Show state variables
    while (true) {

        // Show mavlink state
        if (!show_mav_state()) {
            return false;
        }

        // Wait time
        std::this_thread::sleep_for(
            std::chrono::duration<int, std::milli>(1000));

        // Check loop-end condition
        int time_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - test_start_time).count();
        if (time_elapsed > time_s * 1e3) {
            break;
        }
    }

    return true;
}

bool connection_test::show_mav_state()
{

    if (!this->mav->is_ready()) {
        return true;
    }

    using namespace mavlink_vehicles;

    mode mod = this->mav->get_mode();
    status stat = this->mav->get_status();
    attitude att = this->mav->get_attitude();
    gps_status gps = this->mav->get_gps_status();
    local_pos local = this->mav->get_local_position_ned();
    global_pos_int home = this->mav->get_home_position_int();
    global_pos_int global = this->mav->get_global_position_int();
    global_pos_int target_pos_global = this->mav->get_mission_waypoint();

    local_pos target_pos = math::global_to_local_ned(target_pos_global, home);

    std::cout << "[connection_test] Status: " << (int)stat << std::endl;
    std::cout << "[connection_test] Mode: " << (int)mod << std::endl;

    if (home.is_initialized()) {
        std::cout << "[connection_test] Home Position: " << home.lat << ", "
                  << home.lon << ", " << home.alt << std::endl;
    }
    if (att.is_initialized()) {
        std::cout << "[connection_test] Attitude: " << att.roll << ", "
                  << att.pitch << ", " << att.yaw << std::endl;
    }
    if (global.is_initialized()) {
        std::cout << "[connection_test] Global Position: " << global.lat << ", "
                  << global.lon << ", " << global.alt << std::endl;
    }
    if (local.is_initialized()) {
        std::cout << "[connection_test] Local Position: " << local.x << ", "
                  << local.y << ", " << local.z << std::endl;
    }
    if (target_pos_global.is_initialized()) {
        std::cout << "[connection_test] Target Position Global: "
                  << target_pos_global.lat << ", " << target_pos_global.lon
                  << ", " << target_pos_global.alt << std::endl;

        std::cout << "[connection_test] Target Position: " << target_pos.x
                  << ", " << target_pos.y << ", " << target_pos.z << std::endl;
    }
    std::cout << "[connection_test] Gps: " << (int)gps << std::endl;

    return true;
}

bool conversion_test::run(int time_s)
{
    using namespace mavlink_vehicles;

    // Set home position
    global_pos_int home(-353631722, 1491651272, 574090);

    // Set a new target position
    local_pos target_local(10.0, 10.0, 10.0);
    std::cout << "[conversion_test] Local Position (before conversion): "
              << target_local.x << ", " << target_local.y << ", "
              << target_local.z << std::endl;

    // Convert from local to global
    global_pos_int target_global =
        math::local_ned_to_global(target_local, home);
    std::cout << "[conversion_test] Global Position: " << target_global.lat
              << ", " << target_global.lon << ", " << target_global.alt
              << std::endl;

    // Convert from global to local
    local_pos target_local_conv =
        math::global_to_local_ned(target_global, home);
    std::cout << "[conversion_test] Local Position (after conversion): "
              << target_local_conv.x << ", " << target_local_conv.y << ", "
              << target_local_conv.z << std::endl;

    // Compare values
    if (std::fabs(target_local_conv.x - target_local.x) > defaults::comp_tol) {
        return false;
    }

    if (std::fabs(target_local_conv.y - target_local.y) > defaults::comp_tol) {
        return false;
    }

    if (std::fabs(target_local_conv.z - target_local.z) > defaults::comp_tol) {
        return false;
    }

    return true;
}

bool mission_test::run(int time_s)
{
    // Try to connect to vehicle
    if(!this->connect()) {
        return false;
    }

    // Store test start time
    std::chrono::time_point<std::chrono::system_clock> test_start_time =
        std::chrono::system_clock::now();

    // Wait for vehicle initialization returning false if timed out
    while (!this->mav->is_ready()) {
        int time_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - test_start_time).count();
        if (time_elapsed > time_s * 1e3) {
            return false;
        }
    }

    using namespace mavlink_vehicles;

    // Get home position
    global_pos_int home = this->mav->get_home_position_int();

    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(1000));

    // Set mission target position
    local_pos target_local(30.0, 30.0, -10.0);
    global_pos_int target_global =
        math::local_ned_to_global(target_local, home);

    // Set detour target position
    local_pos detour_local(0.0, 0.0, -10.0);
    global_pos_int detour_global =
        math::local_ned_to_global(detour_local, home);

    // Send detour target position
    this->mav->send_detour_waypoint(detour_global);
    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(10000));

    // Send mission target position
    this->mav->send_mission_waypoint(target_global);

    // Wait for 5 seconds
    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(5000));

    // Send Rotation command
    this->mav->brake(true);

    // Wait until the vehicle gets to the mission target
    std::chrono::time_point<std::chrono::system_clock> mission_start_time =
        std::chrono::system_clock::now();
    while (true) {
        std::this_thread::sleep_for(
            std::chrono::duration<int, std::milli>(100));

        // The vehicle has arrived at the mission waypoint
        if (math::dist(target_global, this->mav->get_global_position_int()) <=
            defaults::arrival_max_dist_m) {
            return true;
        }

        // The vehicle has not arrived at the mission waypoint in time
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - mission_start_time)
                .count() > time_s * 1e3) {
            return false;
        }
    }
}
}

