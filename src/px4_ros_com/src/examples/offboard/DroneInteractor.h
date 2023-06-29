#ifndef DRONEINTERACTOR_H
#define DRONEINTERACTOR_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>

#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>

// #include "WindSubscriber.h"
// #include "telemetryPublisher.h"
// #include "gc_cc_typesPubSubTypes.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

class DroneInteractor
{

    std::vector<mavsdk::Mission::MissionItem> missionItems;
    // path pathInfo;

    // std::array<std::array<std::array<float, 50>, 50>, 2> localWindData;

    // Mission mission;
    mavsdk::Telemetry::GpsGlobalOrigin origin;

    std::shared_ptr<Telemetry> telemetry;
    // std::shared_ptr<Mission> mission;
    void setTelemetry(Telemetry telemetryParam);
    float computePoverPHover(float groundVelocity, float vWind);
    float computeDeltaE(float P, float vGround, float distToLastPoint);
    float vectorMagnitude(float v1, float v2, float v3);
    float deltaE(mavsdk::Telemetry::PositionVelocityNed telemetryPositionUpdate, mavsdk::Telemetry::PositionVelocityNed telemetryPosition);

public:
    // System* system;
    DroneInteractor();

    // void flyOffboard(path pathInfo, WindSubscriber *ptrToWindSubObj, telemetryPublisher *ptrToTelemetryPubObj);
    void flyMission(std::vector<px4_msgs::msg::TrajectorySetpoint> trajectorySetpointsFromFile, bool convertToGeodetic);

    mavsdk::Telemetry::GpsGlobalOrigin getHomeLocation();
    void createMission(std::vector<px4_msgs::msg::TrajectorySetpoint> trajectorySetpointsFromFile);
    void uploadMission();
    void startMission();
    std::shared_ptr<System> get_system(Mavsdk &mavsdk);
    void set_system(std::shared_ptr<System> system);
};

#endif
