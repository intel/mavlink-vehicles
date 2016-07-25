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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <sys/socket.h>
#include <unordered_map>

namespace mavconn
{

struct state_variable {
    std::chrono::time_point<std::chrono::system_clock> timestamp =
        std::chrono::system_clock::from_time_t(0);
    bool is_new = false;
    bool is_initialized()
    {
        return timestamp != std::chrono::system_clock::from_time_t(0);
    }
};

struct attitude : state_variable {
    float roll;
    float pitch;
    float yaw;
};

struct global_pos : state_variable {
    int32_t lat;
    int32_t lon;
    int32_t alt;
};

struct local_pos : state_variable {
    float x;
    float y;
    float z;
};

enum class status { STANDBY, ACTIVE };

enum class mode { GUIDED, OTHER };

enum class arm_status { ARMED, NOT_ARMED };

enum class gps_status { NO_FIX, FIX_2D_PLUS };

enum class cmd_custom { HEARTBEAT, SET_MODE };

class msghandler;

class mavserver
{
  public:
    mavserver(int socket_fd);
    ~mavserver();

    void update();

    status get_status() const;
    arm_status get_arm_status() const;
    mode get_mode() const;
    attitude get_attitude();
    global_pos get_home_position();
    local_pos get_local_position_ned();
    global_pos get_global_position();
    gps_status get_gps_status() const;

    bool started();
    void send_heartbeat();
    void set_mode(mode m);
    void arm_throttle();
    void takeoff();
    void rotate(double angleDeg);
    void goto_waypoint(global_pos pos);

  private:
    mode base_mode;
    status stat;
    arm_status arm_stat;
    attitude att;
    global_pos home;
    global_pos global;
    local_pos local;
    gps_status gps;

    friend class msghandler;

    int sock = 0;
    struct sockaddr_storage remote_addr = {0};

    socklen_t remote_addr_len = sizeof(remote_addr);

    std::chrono::time_point<std::chrono::system_clock>
        remote_last_respond_time = std::chrono::system_clock::from_time_t(0);

    std::unordered_map<int, std::chrono::time_point<std::chrono::system_clock>>
        cmd_long_timestamps;

    std::unordered_map<cmd_custom,
                       std::chrono::time_point<std::chrono::system_clock>>
        cmd_custom_timestamps;

    bool is_remote_responding() const;
    ssize_t send_data(uint8_t *data, size_t len);
    void send_cmd_long(int cmd, float p1, float p2, float p3, float p4,
                       float p5, float p6, float p7, int timeout);
};
}
