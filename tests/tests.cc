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

#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "mavserver.hh"
#include "tests.hh"

int main()
{

    tests::connection_test st;
    st.run();
}

namespace tests
{

namespace defaults
{
const uint16_t mavproxy_port = 14556;
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

    // Instantiate mavserver
    this->mav = std::make_shared<mavconn::mavserver>(sock);
}

connection_test::~connection_test()
{
}

void connection_test::run()
{

    // Initialize mavconn update thread
    this->send_recv_thread_run = true;
    this->send_recv_thread = std::thread(&connection_test::update, this);
    this->send_recv_thread.detach();

    // Check if mavserver has been initialized
    std::cout << "[connection test] " << "Waiting for mav-vehicle initialization..." << std::endl;
    while (!this->mav->started()) {
        continue;
    }
    std::cout << "[connection test] " << "mav-vehicle initialized." << std::endl;

    // Execute test
    while (true) {
        // Execute every once in a while only
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

    if(!this->mav->started()) {
        return;
    }

    mavconn::status stat = this->mav->get_status();
    mavconn::mode mod = this->mav->get_mode();
    mavconn::attitude att = this->mav->get_attitude();
    mavconn::global_pos home = this->mav->get_home_position();
    mavconn::global_pos global = this->mav->get_global_position();
    mavconn::local_pos local = this->mav->get_local_position_ned();
    mavconn::gps_status gps = this->mav->get_gps_status();

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
        std::cout << "[connection test] Local Position: " << local.x << ", " << local.y << ", " << local.z << std::endl;
    }

    std::cout << "[connection test] Gps: " << (int)gps << std::endl;
}
}

