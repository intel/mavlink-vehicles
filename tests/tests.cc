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
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "mavlink_vehicles.hh"
#include "tests.hh"

int main()
{
    // tests::connection_test st;
    // st.run();

    // tests::conversion_test ct;
    // ct.run();

    tests::mission_test mt;
    mt.run();
}

namespace tests
{

namespace defaults
{
const uint16_t mavproxy_port = 14557;
}

connection_test::connection_test()
{
    // Socket Initialization
    this->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        perror("error opening socket");
        exit(EXIT_FAILURE);
    }

    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local_addr.sin_port = htons(defaults::mavproxy_port);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) ==
        -1) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK) == -1) {
        perror("error setting socket as nonblocking");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Instantiate mav_vehicle
    this->mav = std::make_shared<mavlink_vehicles::mav_vehicle>(sock);
}

connection_test::~connection_test()
{
}

void connection_test::run()
{

    // Initialize mavlink_vehicles update thread
    this->send_recv_thread_run = true;
    this->send_recv_thread = std::thread(&connection_test::update, this);
    this->send_recv_thread.detach();

    // Check if mav_vehicle has been initialized
    std::cout << "[connection test] "
              << "Waiting for mav-vehicle initialization..." << std::endl;
    while (!this->mav->is_ready()) {
        // Continue looping while allowing other threads to run concurrently.
        std::this_thread::yield();
    }
    std::cout << "[connection test] "
              << "mav-vehicle initialized." << std::endl;

    // Execute test
    while (true) {
        // Execute every once in a while only

        // Show mavlink state
        show_mav_state();
        std::this_thread::sleep_for(
            std::chrono::duration<int, std::milli>(1000));
    }
}

void connection_test::update()
{
    while (send_recv_thread_run) {
        this->mav->update();
    }
}

void connection_test::show_mav_state()
{

    if (!this->mav->is_ready()) {
        return;
    }

    mavlink_vehicles::mode mod = this->mav->get_mode();
    mavlink_vehicles::status stat = this->mav->get_status();
    mavlink_vehicles::attitude att = this->mav->get_attitude();
    mavlink_vehicles::gps_status gps = this->mav->get_gps_status();
    mavlink_vehicles::local_pos local = this->mav->get_local_position_ned();
    mavlink_vehicles::global_pos_int home = this->mav->get_home_position_int();
    mavlink_vehicles::global_pos_int global =
        this->mav->get_global_position_int();
    mavlink_vehicles::global_pos_int target_pos_global =
        this->mav->get_mission_waypoint();

    mavlink_vehicles::local_pos target_pos =
        mavlink_vehicles::math::global_to_local_ned(target_pos_global, home);

    std::cout << "[connection test] Status: " << (int)stat << std::endl;
    std::cout << "[connection test] Mode: " << (int)mod << std::endl;

    if (home.is_initialized()) {
        std::cout << "[connection test] Home Position: " << home.lat << ", "
                  << home.lon << ", " << home.alt << std::endl;
    }
    if (att.is_initialized()) {
        std::cout << "[connection test] Attitude: " << att.roll << ", "
                  << att.pitch << ", " << att.yaw << std::endl;
    }
    if (global.is_initialized()) {
        std::cout << "[connection test] Global Position: " << global.lat << ", "
                  << global.lon << ", " << global.alt << std::endl;
    }
    if (local.is_initialized()) {
        std::cout << "[connection test] Local Position: " << local.x << ", "
                  << local.y << ", " << local.z << std::endl;
    }
    if (target_pos_global.is_initialized()) {
        std::cout << "[connection test] Target Position Global: "
                  << target_pos_global.lat << ", " << target_pos_global.lon
                  << ", " << target_pos_global.alt << std::endl;

        std::cout << "[connection test] Target Position: " << target_pos.x
                  << ", " << target_pos.y << ", " << target_pos.z << std::endl;
    }
    std::cout << "[connection test] Gps: " << (int)gps << std::endl;
}

conversion_test::conversion_test()
{
}

conversion_test::~conversion_test()
{
}

void conversion_test::run()
{

    // Execute test
    while (true) {

        // Set home position
        mavlink_vehicles::global_pos_int home(-353631722, 1491651272, 574090);

        // Set a new target position
        mavlink_vehicles::local_pos target_local(10.0, 10.0, 10.0);
        std::cout << "Local Position (before conversion): " << target_local.x
                  << ", " << target_local.y << ", " << target_local.z
                  << std::endl;

        // Convert from local to global
        mavlink_vehicles::global_pos_int target_global =
            mavlink_vehicles::math::local_ned_to_global(target_local, home);
        std::cout << "Global Position: " << target_global.lat << ", "
                  << target_global.lon << ", " << target_global.alt
                  << std::endl;

        // Convert from global to local
        target_local =
            mavlink_vehicles::math::global_to_local_ned(target_global, home);
        std::cout << "Local Position (after conversion): " << target_local.x
                  << ", " << target_local.y << ", " << target_local.z
                  << std::endl;

        std::this_thread::sleep_for(
            std::chrono::duration<int, std::milli>(2000));
    }
}


mission_test::mission_test()
{
    // Socket Initialization
    this->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        perror("error opening socket");
        exit(EXIT_FAILURE);
    }

    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local_addr.sin_port = htons(defaults::mavproxy_port);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) ==
        -1) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK) == -1) {
        perror("error setting socket as nonblocking");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Instantiate mav_vehicle
    this->mav = std::make_shared<mavlink_vehicles::mav_vehicle>(sock);
}

mission_test::~mission_test()
{
}

void mission_test::run()
{

    // Initialize mavlink_vehicles update thread
    this->send_recv_thread_run = true;
    this->send_recv_thread = std::thread(&mission_test::update, this);
    this->send_recv_thread.detach();

    // Check if mav_vehicle has been initialized
    std::cout << "[connection test] "
              << "Waiting for mav-vehicle initialization..." << std::endl;
    while (!this->mav->is_ready()) {
        // Continue looping while allowing other threads to run concurrently.
        std::this_thread::yield();
    }
    std::cout << "[connection test] "
              << "mav-vehicle initialized." << std::endl;

    // Execute test

    // Get home position
    mavlink_vehicles::global_pos_int home = this->mav->get_home_position_int();

    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(1000));

    // Set mission target position
    mavlink_vehicles::local_pos target_local(30.0, 30.0, -10.0);
    mavlink_vehicles::global_pos_int target_global =
        mavlink_vehicles::math::local_ned_to_global(target_local, home);

    // Set detour target position
    mavlink_vehicles::local_pos detour_local(0.0, 0.0, -10.0);
    mavlink_vehicles::global_pos_int detour_global =
        mavlink_vehicles::math::local_ned_to_global(detour_local, home);

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
    while (true) {
        std::this_thread::sleep_for(
            std::chrono::duration<int, std::milli>(100));
    }
}

void mission_test::update()
{
    while (send_recv_thread_run) {
        this->mav->update();
    }
}
}

