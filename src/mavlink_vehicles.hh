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
#include <sys/socket.h>
#include <sys/types.h>
#include <unordered_map>
#include <vector>

/**
 * @file
 * @brief Mavlink Vehicles library
 */

namespace mavlink_vehicles
{

/**
 * @brief Basic vehicle state storage
 */
struct state_variable {
    std::chrono::time_point<std::chrono::system_clock> timestamp =
        std::chrono::system_clock::from_time_t(0);
    bool is_new = false;
    bool is_initialized()
    {
        return timestamp != std::chrono::system_clock::from_time_t(0);
    }
};

/**
 * @brief Store vehicle attitude in radians
 */
struct attitude : state_variable {
    float roll;
    float pitch;
    float yaw;
};

/**
 * @brief Store vehicle global position in ENU
 *    int32_t lat; //! @brief Latitude in degrees * 1e7
 *    int32_t lon; //! @brief Longitude in degrees * 1e7
 *    int32_t alt; //! @brief Millimeters (AMSL)
 */
struct global_pos_int : state_variable {
    int32_t lat = 0; // Degrees * 1e7
    int32_t lon = 0; // Degrees * 1e7
    int32_t alt = 0; // Millimeters (AMSL)
    global_pos_int() {}
    global_pos_int(int32_t _lat, int32_t _lon, int32_t _alt)
        : lat(_lat), lon(_lon), alt(_alt) {}
};

/**
 * @brief Store vehicle local position in ENU or NED
 * float x; //! @brief X position in meters
 * float y; //! @brief Y position in meters
 * float z; //! @brief Z position in meters
 */
struct local_pos : state_variable {
    float x = 0.0; // Meters
    float y = 0.0; // Meters
    float z = 0.0; // Meters
    local_pos() {}
    local_pos(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

/**
 * @brief Vehicle status: Active means flying
 */
enum class status { STANDBY, ACTIVE };

/**
 * @brief Vehicle modes
 */
enum class mode { GUIDED, AUTO, BRAKE, OTHER, TAKEOFF };

/**
 * @brief Vehicle arm status
 */
enum class arm_status { ARMED, NOT_ARMED };

/**
 * @brief Vehicle GPS status
 */
enum class gps_status { NO_FIX, FIX_2D_PLUS };

/**
 * @brief Vehicle Installed autopilot
 */
enum class autopilot_type { APM, PX4, UNKNOWN };

/**
 * @brief Custom vehicle commands
 */
enum class cmd_custom {
    HEARTBEAT,
    SET_MODE_GUIDED,
    SET_MODE_AUTO,
    SET_MODE_BRAKE,
    SET_MODE_TAKEOFF,
    REQUEST_MISSION_ITEM,
    REQUEST_MISSION_LIST,
    ROTATE,
    DETOUR
};

/**
 * @brief Vehicle mission status
 */
enum class mission_status { BRAKING, DETOURING, ROTATING, NORMAL };

/**
 * @brief A friend class of mav_vehicle specialized in parsing mavlink messages
 */
class msghandler;

namespace math
{

/**
 * @brief Convert radians to degrees
 * @param x Angle in radians
 * @return Angle in degrees
 */
inline double rad2deg(double x);

/**
 * @brief Convert degrees to radians
 * @param x Angle in degrees
 * @return Angle in radians
 */
inline double deg2rad(double x);

/**
 * @brief Calculate the distance in meters between two global positions
 * @param p1 Global position 1
 * @param p2 Global position 2
 * @return Distance in meters
 */
double dist(global_pos_int p1, global_pos_int p2);

/**
 * @brief Project two global positions onto the ground plane and return
 *        the distance in meters between the projected points.
 * @param p1 Global position 1
 * @param p2 Global position 2
 * @return Projected Distance in meters
 */
double ground_dist(global_pos_int p1, global_pos_int p2);

/**
 * @brief Project two local positions onto the ground plane and return
 *        the distance in meters between the projected points.
 * @param p1 Global position 1
 * @param p2 Global position 2
 * @return Projected Distance in meters
 */
double ground_dist(local_pos p1, local_pos p2);

/**
 * @brief Convert global position to local position NED relative to a
 *        reference point.
 * @param point Global position to be converted
 * @param reference Global position to be used as a reference
 * @return Local position NED
 */
local_pos global_to_local_ned(global_pos_int point, global_pos_int reference);

/**
 * @brief Convert local position NED to global position relative to a
 *        reference point.
 * @param point Local position to be converted
 * @param reference Global position to be used as a reference
 * @return Global position
 */
global_pos_int local_ned_to_global(local_pos point, global_pos_int reference);

/**
 * @brief Calculate the relative horizontal angle between a waypoint and the
 *        vehicle
 * @param wp_pos Global position of the waypoint
 * @param ref_pos Global position of the vehicle
 * @param ref_att Attitude of the vehicle
 * @return Relative horizontal angle (yaw)
 */
double get_waypoint_rel_angle(global_pos_int wp_pos, global_pos_int ref_pos,
                              attitude ref_att);
}

/**
 * @brief Mavlink Vehicle abstraction
 */
class mav_vehicle
{
     /**
     * @brief Default Constructor. Generates a random system id for our side.
     * @param socket_fd Socket connected to the vehicle.
     */
    public: mav_vehicle(int socket_fd);

    /**
     * @brief Alternative Constructor, receives system id as an argument.
     * @param socket_fd Socket connected to the vehicle.
     * @param sysid System id provided by the caller.
     */
    public: mav_vehicle(int socket_fd, uint8_t sysid);

    /**
     * @brief Destructor.
     */
    public: ~mav_vehicle();

    /**
     * @brief Update loop. Receive and parse all mavlink messages, update
     *        state variables and execute specific commands according to current
     *        vehicle state.
     */
    public: void update();

    /**
     * @brief Check if the vehicle is ready.
     *        - Detect if there is a vehicle sending and receiving through the
     *          socket given to the constructor.
     *        - Detect if the vehicle has been initialized and is ready to receive commands.
     *        - Detect if the vehicle is still responding.
     * @return True if all checks above are true. False otherwise.
     */
    public: bool is_ready();

    /**
     * @brief Get vehicle mode.
     * @return Current vehicle mode.
     */
    public: mode get_mode() const;

    /**
     * @brief Get vehicle status.
     * @return Current vehicle status.
     */
    public: status get_status() const;

    /**
     * @brief Get the vehicle gps status.
     * @return Current gps status.
     */
    public: gps_status get_gps_status() const;

    /**
     * @brief Get the vehicle arm status.
     * @return Current arm status.
     */
    public: arm_status get_arm_status() const;

    /**
     * @brief Get the vehicle attitude.
     * @return Current attitude.
     */
    public: attitude get_attitude();

    /**
     * @brief Get the vehicle local position NED.
     * @return Current local position NED.
     */
    public: local_pos get_local_position_ned();

    /**
     * @brief Get the vehicle home position.
     * @return Current home position in global coordinates.
     */
    public: global_pos_int get_home_position_int();

    /**
     * @brief Get the vehicle global position.
     * @return Current global position.
     */
    public: global_pos_int get_global_position_int();

    /**
     * @brief Get the mission waypoint the vehicle is currently heading to.
     *        The vehicle moves automatically to a mission waypoint when in
     *        AUTO mode. A mission waypoint can be sent externaly, through a
     *        ground station - or any other system capable of controlling the
     *        vehicle - or through a call to send_mission_waypoint().
     * @return Current mission waypoint in global coordinates.
     */
    public: global_pos_int get_mission_waypoint();

    /**
     * @brief Get the detour waypoint the vehicle is currently heading to.
     *        The vehicle moves automatically to a detour waypoint when in
     *        GUIDED mode. A detour waypoint can be sent externaly, through a
     *        ground station - or any other system capable of controlling the
     *        vehicle - or through a call to send_detour_waypoint().
     * @return Current detour waypoint in global coordinates.
     */
    public: global_pos_int get_detour_waypoint();

    /**
     * @brief Request takeoff. The result of this call can be checked with
     *        get_status(). If the vehicle has successfully taken off, status
     *        will be ACTIVE.
     */
    public: void takeoff();

    /**
     * @brief Request to arm or disarm throttle. the result of this call can be
     *        checked with get_arm_status().
     */
    public: void arm_throttle(bool arm_disarm = true);

    /**
     * @brief Send a heartbeat through vehicle socket.
     *        Some autopilots need to receive periodic heartbeats from external
     *        systems in order to leave their communication channels with them
     *        open.
     */
    public: void send_heartbeat();

    /**
     * @brief Set vehicle mode.
     * @param m Mode.
     */
    public: void set_mode(mode m);

    /**
     * @brief Request mission list
     */
    public: void request_mission_list();

    /**
     * @brief Command the vehicle to immediately stop and rotate before moving
     *        to the next detour or mission waypoint.
     * @param angle_rad Rotation angle in radians (Right handed, Z-up)
     * @param autocontinue Go back to the previous mode (GUIDED/AUTO) after
     *                     rotation is complete.
     */
    public: void rotate(double angle_rad, bool autocontinue = true);

    /**
     * @brief Enable or disable autorotation during mission.
     *        When this feature is enabled, the vehicle makes its best effort
     *        to keep looking straight to the next mission waypoint at all
     *        times.
     * @param autorotate Enable if true, disable if false
     */
    public: void set_autorotate_during_mission(bool autorotate);

    /**
     * @brief Enable or disable autorotation during detour.
     *        When this feature is enabled, the vehicle makes its best effort
     *        to keep looking straight to the next detour waypoint at all
     *        times.
     * @param autorotate Enable if true, disable if false
     */
    public: void set_autorotate_during_detour(bool autorotate);

    /**
     * @brief Check if the vehicle is still rotating after a rotate() command.
     * @return True if so and false otherwise.
     */
    public: bool is_rotation_active() const;

    /**
     * @brief Command the vehicle to immediately take a detour throught the
     *        given waypoint. If autocontinue is true, the vehicle gets back to
     *        the previous mode (GUIDED/AUTO) right after finishing the detour,
     *        continuing the previous mission in the first case.
     * @param lat Latitude of the waypoint in degrees
     * @param lon Longitude of the waypoint in degrees
     * @param lon Altitude of the waypoint in meters
     * @param autocontinue Go back to the previous mode (GUIDED/AUTO) after
     *                     rotation is complete.
     * @param autorotate Enable or disable autorotation during the detour.
     */
    public: void send_detour_waypoint(double lat, double lon, double alt,
                              bool autocontinue = true,
                              bool autorotate = false);
    /**
     * @brief Command the vehicle to immediately take a detour throught the
     *        given waypoint. If autocontinue is true, the vehicle gets back to
     *        the previous mode (GUIDED/AUTO) right after finishing the detour,
     *        continuing the previous mission in the first case.
     * @param global Global position of the waypoint
     * @param autocontinue Go back to the previous mode (GUIDED/AUTO) after
     *                     detour is complete.
     * @param autorotate Enable or disable autorotation during the detour.
     */
    public: void send_detour_waypoint(global_pos_int global, bool autocontinue = true,
                              bool autorotate = false);

    /**
     * @brief Check if the vehicle is still detouring after a
     *        send_detour_waypoint() command.
     * @return True if so and false otherwise.
     */
    public: bool is_detour_active() const;

    /**
     * @brief Command the vehicle to go immediately to the given mission
     *        waypoint. This command overwrites any mission list that may be
     *        stored into the vehicle, replacing it with a list with only one
     *        waypoint.
     * @param lat Latitude of the waypoint in degrees
     * @param lon Longitude of the waypoint in degrees
     * @param lon Altitude of the waypoint in meters
     * @param autorotate Enable or disable autorotation during the detour.
     */
    public: void send_mission_waypoint(double lat, double lon, double alt,
                               bool autorotate = false);
    /**
     * @brief Command the vehicle to go immediately to the given mission
     *        waypoint. This command overwrites any mission list that may be
     *        stored into the vehicle, replacing it with a list with only one
     *        waypoint.
     * @param global Global position of the waypoint
     * @param autorotate Enable or disable autorotation during the detour.
     */
    public: void send_mission_waypoint(global_pos_int global, bool autorotate = false);

    /**
     * @brief Check if our system is still sending a mission to the vehicle.
     * @return True if so and false otherwise.
     */
    public: bool is_sending_mission() const;

    /**
     * @brief Check if our system is currently receiving a mission list from
     *        the vehicle.
     * @return True if so and false otherwise.
     */
    public: bool is_receiving_mission() const;

    /**
     * @brief Command the vehicle to brake immediately. The vehicle
     *        automatically continues the mission or the detour it might be
     *        currently in after complete stop if autocontinue is true.
     * @param autocontinue Go back to the previous mode (GUIDED/AUTO) after
     *                     complete stop.
     */
    public: void brake(bool autocontinue);

    /**
     * @brief Check if the vehicle is still braking after a
     *        brake() command.
     * @return True if so and false otherwise.
     */
    public: bool is_brake_active() const;

    /**
     * @brief In case more than one mavlink_vehicle instance is connected to a
     *        vehicle at the same time, only one of them can be responsible for
     *        the autorotation of the vehicle. Take_control sets this instance
     *        as the one responsible for that feature, making disabling the
     *        autorotation responsibility of all the other existing mav_vehicle
     *        instances.
     */
    public: void take_control(bool take_control);

  private:
    status stat = status::STANDBY;
    arm_status arm_stat = arm_status::NOT_ARMED;
    gps_status gps = gps_status::NO_FIX;
    mode base_mode = mode::OTHER;

    attitude att;
    local_pos local;
    local_pos speed;
    global_pos_int home;
    global_pos_int global;

    global_pos_int mission_waypoint;
    uint16_t mission_waypoint_id = 0;
    bool mission_waypoint_outdated = true;

    bool sending_mission = false;
    std::vector<global_pos_int> mission_to_send;

    bool receiving_mission = false;
    unsigned int mission_size = 0;
    std::vector<global_pos_int> received_mission;

    mission_status mstatus = mission_status::NORMAL;

    global_pos_int detour_waypoint;
    bool detour_waypoint_autocontinue = true;
    bool mission_waypoint_autorotate = false;
    bool detour_waypoint_autorotate = false;

    float rotation_goal = 0;
    bool is_our_control = false;
    bool autocontinue_after_brake = true;
    bool autocontinue_after_rotation = false;
    mission_status autocontinue_action = mission_status::NORMAL;

    std::chrono::time_point<std::chrono::system_clock> stop_time =
        std::chrono::system_clock::from_time_t(0);
    bool is_stopped();

    autopilot_type autopilot = autopilot_type::UNKNOWN;
    uint8_t system_id = 0;
    int sock = 0;
    struct sockaddr_storage remote_addr = {0};
    socklen_t remote_addr_len = sizeof(remote_addr);
    std::chrono::time_point<std::chrono::system_clock>
        remote_last_response_time = std::chrono::system_clock::from_time_t(0);
    bool remote_responding = false;
    bool is_remote_responding() const;

    struct EnumHash
    {
        template <typename T> std::size_t operator()(T t) const
        {
            return static_cast<std::size_t>(t);
        }
    };

    std::unordered_map<int, std::chrono::time_point<std::chrono::system_clock>>
        cmd_long_timestamps;
    std::unordered_map<cmd_custom,
                       std::chrono::time_point<std::chrono::system_clock>,
                       EnumHash>
        cmd_custom_timestamps;

    ssize_t send_data(uint8_t *data, size_t len);
    void send_cmd_long(int cmd, float p1, float p2, float p3, float p4,
                       float p5, float p6, float p7, int timeout);
    void send_mission_waypoint(global_pos_int wp, uint16_t seq);
    void send_mission_ack(uint8_t type);
    void set_mode(mode m, int timeout);
    void send_mission_count(int n);
    void request_mission_item(uint16_t item_id);
    void set_position_target(global_pos_int wp, int timeout);
    void set_position_target(global_pos_int wp);
    void set_yaw_target(float yaw, int timeout);
    void set_yaw_target(float yaw);

    friend class msghandler;
};
}
