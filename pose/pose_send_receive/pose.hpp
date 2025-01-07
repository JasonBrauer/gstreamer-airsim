#ifndef POSE_H
#define POSE_H

#include <stdint.h>
#include <string>

namespace PoseTransfer {
    // we need a struct that has fields for
    // drone position and attitude
    // camera position and attitude
    typedef struct Pose {
        double x;
        double y;
        double z;
        double w;
        double xi;
        double yj;
        double zk;
    } Pose;

    typedef struct PoseMessage {
        uint64_t message_counter; // this is a counter that increments every time we send a message
        Pose drone;
        Pose camera;
        uint16_t drone_id;
    } PoseMessage;

    #pragma pack(push,1)
    typedef struct UdpPose {
        int64_t x;
        int64_t y;
        int64_t z;
        int64_t w;
        int64_t xi;
        int64_t yj;
        int64_t zk;
    } UdpPose;

    typedef struct UdpPoseMessage {
        uint64_t message_counter; // this is a counter that increments every time we send a message
        UdpPose drone;
        UdpPose camera;
        uint16_t drone_id;
    } UdpPoseMessage;
    #pragma pack(pop)

    static constexpr unsigned long udp_decimal_offset = 1e6; // used to convert doubles to uint64_t
}


#endif