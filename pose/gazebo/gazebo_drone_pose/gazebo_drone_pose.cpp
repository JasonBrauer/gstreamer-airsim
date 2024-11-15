#include <string>

#include "gazebo_drone_pose.hpp"
#include "pose.hpp"
#include "pose_sender.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>

constexpr int NWIDTH = 7;
static constexpr int MESSAGE_THROTTLE = 100;


GenerateCbLocalPose::GenerateCbLocalPose(
    PoseSender* poseSender
) {
    this->poseSender = poseSender;
    this->poseSender->create_socket();
}


GenerateCbLocalPose::~GenerateCbLocalPose() {};


void GenerateCbLocalPose::trackDroneIds(std::string droneName) {
    if (this->droneIds.contains(droneName)) {
      return;
    }

    const std::string gidPrefix = "_gid_"; //Example droneName: drone_0_gid_123
    size_t gidPrefixPos = droneName.rfind(gidPrefix);

    if (gidPrefixPos != std::string::npos && gidPrefixPos != 0) {
      std::string gidStr = droneName.substr(gidPrefixPos + gidPrefix.length());

      try {
        int globalId = std::stoi(gidStr);
        this->droneIds[droneName] = globalId;

      } catch (const std::invalid_argument &e) {
        this->droneIds[droneName] = uniqueDroneCount;
      }
    } else {
      this->droneIds[droneName] = uniqueDroneCount;
    }

    this->uniqueDroneCount++;
}


std::string GenerateCbLocalPose::getCurrentTimeInFormat(){
    auto now = std::chrono::system_clock::now();

    // Convert to time_t to extract date and time components
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTime = *std::localtime(&currentTime);

    // Extract fractional seconds
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()) % 1'000'000;
    
     // Use a stringstream to format the output as a string
    std::ostringstream oss;
    oss << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S")
        << std::setw(6) << std::setfill('0') << microseconds.count();

    return oss.str();
}


void GenerateCbLocalPose::cbLocalPose(ConstPosesStampedPtr& msg) {
    // "~/pose/local/info" is published at 250 Hz
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    static int count = 0;

    PoseTransfer::Pose drone_pose = (PoseTransfer::Pose) {
        .x = -1.0,
        .y = -1.0,
        .z = -1.0,
        .w = -1.0,
        .xi = -1.0,
        .yj = -1.0,
        .zk = -1.0
    };
    PoseTransfer::Pose camera_pose = (PoseTransfer::Pose) {
        .x = -1.0,
        .y = -1.0,
        .z = -1.0,
        .w = -1.0,
        .xi = -1.0,
        .yj = -1.0,
        .zk = -1.0
    };
    std::string current_drone_name;

    for (int i = 0; i < msg->pose_size(); i++) {
        auto x = msg->pose(i).position().x();
        auto y = msg->pose(i).position().y();
        auto z = msg->pose(i).position().z();
        auto ow = msg->pose(i).orientation().w();
        auto ox = msg->pose(i).orientation().x();
        auto oy = msg->pose(i).orientation().y();
        auto oz = msg->pose(i).orientation().z();
    
        std::string msg_name = msg->pose(i).name();
        // https://en.cppreference.com/w/cpp/string/basic_string/npos
        // done body pose has no '::' delimiter - drone name only
        if (msg_name.find("::") == std::string::npos) {
            current_drone_name = msg->pose(i).name(); 
            this->trackDroneIds(current_drone_name);
            drone_pose.x = x;
            drone_pose.y = y;
            drone_pose.z = z;
            drone_pose.w = ow;
            drone_pose.xi = ox;
            drone_pose.yj = oy;
            drone_pose.zk = oz;
            // if (count % MESSAGE_THROTTLE == 0) {

            std::cout << "Packet Number: " << count << ", Drone Id: "
            << std::to_string(this->droneIds[current_drone_name]) << ", Timestamp: "
            << this->getCurrentTimeInFormat() << " , Drone Position: "
            << std::to_string(drone_pose.x) << ", "
            << std::to_string(drone_pose.y) << ", "
            << std::to_string(drone_pose.z) << ", Drone Orientation: "
            << std::to_string(drone_pose.w) << ", "
            << std::to_string(drone_pose.xi) << ", "
            << std::to_string(drone_pose.yj) << ", "
            << std::to_string(drone_pose.zk) 
            << std::endl;



                // (std::cout << "Drone name: " + current_drone_name << 
                // ", Drone id: " + std::to_string(this->droneIds[current_drone_name]));
                // (std::cout << "\nDrone Position: " + 
                // std::to_string(drone_pose.x) + ", " +
                // std::to_string(drone_pose.y) + ", " +
                // std::to_string(drone_pose.z));
                // (std::cout << "\nDrone Orientation: " + 
                // std::to_string(drone_pose.w) + ", " +
                // std::to_string(drone_pose.xi) + ", " +
                // std::to_string(drone_pose.yj) + ", " +
                // std::to_string(drone_pose.zk));

            // }
        }
        else if (
            msg_name.substr(msg_name.find("::") + 2, std::string::npos) == "cgo3_camera_link"
        ) {
            camera_pose.x = x;
            camera_pose.y = y;
            camera_pose.z = z;
            camera_pose.w = ow;
            camera_pose.xi = ox;
            camera_pose.yj = oy;
            camera_pose.zk = oz;

            // if (count % MESSAGE_THROTTLE == 0) {
            //     // messages should be grouped together in sequence for each drone, so the following
            //     // camera orientation is for the drone id associated with the drone position above
            //     (std::cout << "\nCamera Orientation: " + 
            //     std::to_string(camera_pose.w) + ", " +
            //     std::to_string(camera_pose.xi) + ", " +
            //     std::to_string(camera_pose.yj) + ", " +
            //     std::to_string(camera_pose.zk)
            //     << std::endl);
            // }
        }

        // 0 doesn't work because initial state is zero for each drone, so using -1.0
        // there may be a better value than -1.0, but even in the off chance xi for either the
        // camera or drone is exactly -1.0, it will likely only be so momentarily
        if (drone_pose.xi != -1.0 && camera_pose.xi != -1.0) {
            PoseTransfer::PoseMessage pose_message {
                .message_counter = (uint64_t) count,
                .drone = drone_pose,
                .camera = camera_pose,
                .drone_id = this->droneIds[current_drone_name]
            };


            // since all poses are grouped together for each drone within a message,
            // reset camera_pose and drone_pose to default values after sending a message
            // all drone a poses, then all drone b poses, then all drone c poses, . . .
            this->poseSender->send_pose_message(pose_message);
            // if (count % MESSAGE_THROTTLE == 0) {
            //     std::cout << "Sent pose for drone id: " << pose_message.drone_id << std::endl;
            // }
            drone_pose.x = -1.0;
            drone_pose.y = -1.0;
            drone_pose.z = -1.0;
            drone_pose.w = -1.0;
            drone_pose.xi = -1.0;
            drone_pose.yj = -1.0;
            drone_pose.zk = -1.0;

            camera_pose.x = -1.0;
            camera_pose.y = -1.0;
            camera_pose.z = -1.0;
            camera_pose.w = -1.0;
            camera_pose.xi = -1.0;
            camera_pose.yj = -1.0;
            camera_pose.zk = -1.0;
        }
    }
    // if (count % MESSAGE_THROTTLE == 0) {
    //     std::cout << std::endl;
    // }

    ++count;
}


gazebo::transport::SubscriberPtr GenerateCbLocalPose::subscribeGazeboNode(
    gazebo::transport::NodePtr gazeboNodePtr
) {
    // Listen to Gazebo topics
    // update freq ~250 hz
    return gazeboNodePtr->Subscribe(
        "~/pose/local/info",
        &GenerateCbLocalPose::cbLocalPose,
        this
    );
}