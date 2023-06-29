#include "DroneInteractor.h"
#include "nedDistToGeodetic.h"

// hover Performance is set to 150 W
float P_HOVER = 150;

struct BatteryInfo
{
    std::vector<float> voltage;
    std::vector<float> batteryPercentage;
};

std::shared_ptr<System> DroneInteractor::get_system(Mavsdk &mavsdk)
{
    // Mavsdk mavsdk;
    // ConnectionResult connection_result = mavsdk.add_udp_connection();

    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                   {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        } });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout)
    {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

void DroneInteractor::flyMission(std::vector<px4_msgs::msg::TrajectorySetpoint> trajectorySetpointsFromFile, bool convertToGeodetic)
{
    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_udp_connection();

    BatteryInfo batteryInfo = BatteryInfo();

    if (connection_result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
    }

    auto system = this->get_system(mavsdk);
    auto telemetry = Telemetry{system};

    while (!telemetry.health_all_ok())
    {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";

    Telemetry::Position homePosition = telemetry.position();
    std::cout << "Reading home position in Global coordinates\n";

    const auto res_and_gps_origin = telemetry.get_gps_global_origin();
    if (res_and_gps_origin.first != Telemetry::Result::Success)
    {
        std::cerr << "Telemetry failed: " << res_and_gps_origin.first << '\n';
    }
    origin = res_and_gps_origin.second;
    std::cerr << "Origin (lat, lon, alt amsl):\n " << origin << '\n';

    missionItems.clear();

    enuCoords currentENUCoord;
    geodeticCoords homeGeo = {origin.latitude_deg, origin.longitude_deg, origin.altitude_m};

    geodeticCoords calculatedGeoTarget;

    for (auto i : trajectorySetpointsFromFile)
    {
        Mission::MissionItem new_item = Mission::MissionItem();

        currentENUCoord.e1 = i.x;
        currentENUCoord.n1 = i.y;
        currentENUCoord.u1 = i.z;

        if (convertToGeodetic)
        {
            calculatedGeoTarget = enu2geodetic(&currentENUCoord, &homeGeo);
        }
        else
        {
            calculatedGeoTarget = {i.x, i.y, i.z};
        }
        new_item.latitude_deg = calculatedGeoTarget.lat;
        new_item.longitude_deg = calculatedGeoTarget.lon;
        new_item.relative_altitude_m = -i.z;
        new_item.speed_m_s = 5.0f;
        std::cout << "Latitude: " << calculatedGeoTarget.lat << ", "
                  << "Longitude: " << calculatedGeoTarget.lon << "Alti: " << -i.z << std::endl;
        //        new_item.is_fly_through = true;
        //         new_item.gimbal_pitch_deg = 20.0f;
        //         new_item.gimbal_yaw_deg = 60.0f;
        new_item.camera_action = Mission::MissionItem::CameraAction::None;
        missionItems.push_back(new_item);
    }

    std::cout << "Length of Mission-Vector is: " << missionItems.size() << std::endl;
    std::cout << "Uploading mission...\n";
    Mission::MissionPlan missionPlan{};
    missionPlan.mission_items = missionItems;
    auto mission = Mission{system};

    const Mission::Result upload_result = mission.upload_mission(missionPlan);

    if (upload_result != Mission::Result::Success)
    {
        std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
        // return false;
    }

    auto action = Action{system};
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        std::cerr << "Arming failed: " << arm_result << '\n';
        // return 1;
    }
    std::cout << "Armed.\n";

    // subscribe to mission updates:
    std::atomic<bool> want_to_pause{false};
    // Before starting the mission, we want to be sure to subscribe to the mission progress.

    Telemetry::Battery currentBatteryInfo = Telemetry::Battery();
    /*    while (std::isnan(currentBatteryInfo.voltage_v)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            currentBatteryInfo = Telemetry::Battery();
        }       */
    batteryInfo.voltage = {currentBatteryInfo.voltage_v};

    batteryInfo.batteryPercentage = {currentBatteryInfo.remaining_percent};

    std::cout << "Start the Mission..." << std::endl;
    Mission::Result start_mission_result = mission.start_mission();

    Telemetry::PositionVelocityNed telemetryPosition;
    Telemetry::PositionVelocityNed telemetryPositionUpdate;
    telemetryPosition = telemetry.position_velocity_ned();

    // wait until one valid position is available:
    while (std::isnan(telemetryPosition.position.north_m))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        telemetryPosition = telemetry.position_velocity_ned();
    }

    //     batteryInfo.voltage = {currentBatteryInfo.voltage_v};
    //
    //     batteryInfo.batteryPercentage = {currentBatteryInfo.remaining_percent};

    std::cout << "Battery Info... \n Voltage:" << batteryInfo.voltage[0] << " V" << std::endl;
    std::cout << "remaining Percent: " << batteryInfo.batteryPercentage[0] * 100.0 << " %" << std::endl;

    Mission::MissionProgress currentMissionProgress;

    //     float ;
    //     float ;
    //     float ;
    float deltaEn;
    float energyConsumption = 0.0;
    while (!mission.is_mission_finished().second)
    {

        if (start_mission_result != Mission::Result::Success)
        {
            std::cerr << "Starting mission failed: " << start_mission_result << '\n';
            std::cout << "Retry after 1 second..." << std::endl;
            sleep_for(seconds(1));
        }
        else
        {
            telemetryPositionUpdate = telemetry.position_velocity_ned();
            if (std::isnan(telemetryPositionUpdate.position.north_m))
            {
                telemetryPositionUpdate = telemetryPosition;
            }

            // send the position to the telemetry Publisher:
            // ptrToTelemetryPubObj->run(telemetryPositionUpdate, 1, 1);
            // std::cout << "Sending Telemetry..." << std::endl;

            deltaEn = this->deltaE(telemetryPositionUpdate, telemetryPosition);
            energyConsumption += deltaEn;

            // check windSubscriber for local windfield update:
            // localWindData = ptrToWindSubObj->on_data_available2();

            // get mission update:
            currentMissionProgress = mission.mission_progress();
            std::cout << "Mission status update: " << currentMissionProgress.current << " / "
                      << currentMissionProgress.total << '\n';

            // telemetryPosition is the Telemetry-Info from the last iteration.
            telemetryPosition = telemetryPositionUpdate;

            currentBatteryInfo = Telemetry::Battery();
            //             while (std::isnan(currentBatteryInfo.voltage_v)) {
            //                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //                 currentBatteryInfo = Telemetry::Battery();
            //             }
            //
            //             batteryInfo.voltage.push_back(currentBatteryInfo.voltage_v);
            //             batteryInfo.batteryPercentage.push_back(currentBatteryInfo.remaining_percent);

            std::cout << "Battery-Status after " << currentMissionProgress.current << "th-Waypoint:" << std::endl;
            std::cout << "Battery Info... \n Voltage:" << currentBatteryInfo.voltage_v << " V" << std::endl;
            std::cout << "remaining Percent: " << currentBatteryInfo.remaining_percent * 100.0 << " %" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Calculated the Energy Consumption for the mission: " << energyConsumption << " J" << std::endl;

    mission.clear_mission();

    mission.subscribe_mission_progress([&want_to_pause](Mission::MissionProgress mission_progress)
                                       {
        std::cout << "Mission status update: " << mission_progress.current << " / "
                  << mission_progress.total << '\n';

        if (mission_progress.current >= 2) {
            // We can only set a flag here. If we do more request inside the callback,
            // we risk blocking the system.
            want_to_pause = true;
        } });

    // We are done, and can do RTL to go home.
    std::cout << "Commanding RTL...\n";
    const Action::Result rtl_result = action.return_to_launch();
    if (rtl_result != Action::Result::Success)
    {
        std::cout << "Failed to command RTL: " << rtl_result << '\n';
        // return 1;
    }
    std::cout << "Commanded RTL.\n";

    // We need to wait a bit, otherwise the armed state might not be correct yet.
    sleep_for(seconds(2));

    while (telemetry.armed())
    {
        // Wait until we're done.
        sleep_for(seconds(1));
    }
    std::cout << "Disarmed, exiting.\n";
}

// void DroneInteractor::plotLocalWindField() {
//
// }

float DroneInteractor::deltaE(Telemetry::PositionVelocityNed telemetryPositionUpdate, Telemetry::PositionVelocityNed telemetryPosition)
{

    float distFromLastPoint, deltaE, groundVelocity, P, pOverPHover;

    // this method is a wrapper for the methods calculating Delta E:
    groundVelocity = this->vectorMagnitude(telemetryPositionUpdate.velocity.north_m_s, telemetryPositionUpdate.velocity.east_m_s, telemetryPositionUpdate.velocity.down_m_s);
    distFromLastPoint = vectorMagnitude(abs(telemetryPositionUpdate.position.north_m - telemetryPosition.position.north_m), abs(telemetryPositionUpdate.position.east_m - telemetryPosition.position.east_m), abs(telemetryPositionUpdate.position.down_m - telemetryPosition.position.down_m));
    // calculate normalized Performance:
    pOverPHover = this->computePoverPHover(groundVelocity, 0.0);
    P = pOverPHover * P_HOVER;

    return this->computeDeltaE(P, groundVelocity, distFromLastPoint);
}

float DroneInteractor::computeDeltaE(float P, float vGround, float distanceFromLastPoint)
{

    return P * distanceFromLastPoint / vGround;
}

float DroneInteractor::vectorMagnitude(float v1, float v2, float v3)
{
    return sqrt(pow(v1, 2) + pow(v2, 2) + pow(v3, 2));
}
float DroneInteractor::computePoverPHover(float groundVelocity, float vWind)
{

    // ATTENTION: Here the down component is also used to compute the ground-velocity. The polynomial formula
    // just holds for flying at constant altitude.

    float airVelocity = groundVelocity - vWind;

    float a0 = 1.0041;
    float a1 = -0.0023;
    float a2 = -0.0079;
    float a3 = 0.0011;
    return a0 + a1 * airVelocity + a2 * pow(airVelocity, 2) + a3 * pow(airVelocity, 3);
}

DroneInteractor::DroneInteractor()
{
    // Mavsdk mavsdk;
    // system = DroneInteractor::get_system();
    //     this->telemetry =  std::shared_ptr<Telemetry> (new Telemetry{system});
    // auto telemetry = Telemetry{system};
    // this->mission = Mission{system};
}

void DroneInteractor::startMission()
{
    Mavsdk mavsdk;
    auto system = DroneInteractor::get_system(mavsdk);
    auto action = Action{system};
    auto mission = Mission{system};
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        std::cerr << "Arming failed: " << arm_result << '\n';
        // return 1;
    }
    std::cout << "Armed.\n";
    std::cout << "Start the Mission..." << std::endl;
    mission.start_mission();
}

void DroneInteractor::uploadMission()
{
    // this->mission = Mission{system};
    Mavsdk mavsdk;
    auto system = DroneInteractor::get_system(mavsdk);
    auto mission = Mission{system};

    std::cout << "Length of Mission in uploadMission Function: " << missionItems.size() << std::endl;

    std::cout << "Uploading mission...\n";
    Mission::MissionPlan missionPlan{};
    missionPlan.mission_items = missionItems;

    auto action = Action{system};
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        std::cerr << "Arming failed: " << arm_result << '\n';
        // return 1;
    }
    std::cout << "Armed.\n";

    const Mission::Result upload_result = mission.upload_mission(missionPlan);

    if (upload_result != Mission::Result::Success)
    {
        std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
        // return false;
    }

    std::cout << "Start the Mission..." << std::endl;
    mission.start_mission();
}
