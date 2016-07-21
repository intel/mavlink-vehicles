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
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "mavserver.hh"
#include "tests.hh"

int main()
{

    tests::simple_test st;
    st.run();
}

namespace tests
{

namespace defaults
{
const uint16_t mavproxy_port = 14556;
}

simple_test::simple_test()
{
    // Socket Initialization
    this->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        perror("error opening socket");
        exit(EXIT_FAILURE);
    }

    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(defaults::mavproxy_port);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) ==
        -1) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        perror("error setting socket as nonblocking");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Instantiate mavserver
    this->mav = std::make_shared<mavconn::mavserver>(sock);
}

simple_test::~simple_test()
{
}

void simple_test::run()
{

    // Initialize mavconn update thread
    this->send_recv_thread_run = true;
    this->send_recv_thread = std::thread(&simple_test::update, this);
    this->send_recv_thread.detach();

    // Execute test
    while (true) {
        // Execute every once in a while only
        show_mav_state();
        std::this_thread::sleep_for(
            std::chrono::duration<int, std::milli>(3000));
    }
}

void simple_test::update()
{
    while (send_recv_thread_run) {
        this->mav->update();
    }
}

void simple_test::show_mav_state()
{

    // Check if mavserver has been initialized
    if (!this->mav->started()) {
        return;
    }

    // mavconn::status stat = this->mav->get_status();
    // mavconn::mode mod = this->mav->get_mode();
    mavconn::attitude att = this->mav->get_attitude();
    mavconn::global_pos home = this->mav->get_home_position();
    // mavconn::global_pos global = this->mav->get_global_position();
    // mavconn::local_pos local = this->mav->get_local_position();
    // mavconn::gps_status gps = this->mav->get_gps_status();

    if (att.is_initialized()) {
        std::cout << "Attitude: " << att.roll << std::endl;
    }

    if (home.is_initialized()) {
        std::cout << "Home: " << home.lon << std::endl;
    }
}
}

