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

#pragma once

#include <memory>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>

#include "mavlink_vehicles.hh"

namespace tests
{

class connection_test
{

  public:
    connection_test();
    ~connection_test();

    void run();

  private:
    int sock = 0;
    socklen_t fromlen = {0};
    struct sockaddr_in local_addr = {0};
    std::shared_ptr<mavlink_vehicles::mav_vehicle> mav;
    std::thread send_recv_thread;
    bool send_recv_thread_run = false;

    void show_mav_state();
    void update();
};

class mission_test
{

  public:
    mission_test();
    ~mission_test();

    void run();

  private:
    int sock = 0;
    socklen_t fromlen = {0};
    struct sockaddr_in local_addr = {0};
    std::shared_ptr<mavlink_vehicles::mav_vehicle> mav;
    std::thread send_recv_thread;
    bool send_recv_thread_run = false;

    void show_mav_state();
    void update();
};

class conversion_test
{
  public:
    conversion_test();
    ~conversion_test();

    void run();
};
}

