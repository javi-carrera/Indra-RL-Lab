using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;

public class PoseConverter {

    public static Vector3 Ros2UnityPosition(PointMsg unityPosition) {
        return new Vector3((float)unityPosition.x, (float)unityPosition.z, (float)unityPosition.y);
    }

    public static PointMsg Unity2RosPosition(Vector3 rosPosition) {
        return new PointMsg(rosPosition.x, rosPosition.z, rosPosition.y);
    }

    public static Quaternion Ros2UnityRotation(QuaternionMsg rosRotation) {

        // 1. Negate x, y, and z axes -> (-x, -y, -z, w)
        // 2. Swap y and z axes -> (-x, -z, -y, w)

        return new Quaternion((float)-rosRotation.x, (float)-rosRotation.z, (float)-rosRotation.y, (float)rosRotation.w);
    }

    public static QuaternionMsg Unity2RosRotation(Quaternion unityRotation) {

        // 1. Swap y and z axes -> (x, z, y, w)
        // 2. Negate x, y, and z axes -> (-x, -z, -y, w)
        // 4. Return the new quaternion (use the 'zyx' order when recieving the quaternion in ROS)

        return new QuaternionMsg(-unityRotation.x, -unityRotation.z, -unityRotation.y, unityRotation.w);
    }

    public static Vector3 Ros2UnityLinearVelocity(Vector3Msg rosLinearVelocity) {
        return new Vector3((float)rosLinearVelocity.x, (float)rosLinearVelocity.z, (float)rosLinearVelocity.y);
    }

    public static Vector3Msg Unity2RosLinearVelocity(Vector3 unityLinearVelocity) {
        return new Vector3Msg(unityLinearVelocity.x, unityLinearVelocity.z, unityLinearVelocity.y);
    }

    public static Vector3 Ros2UnityAngularVelocity(Vector3Msg rosAngularVelocity) {
        return new Vector3(-(float)rosAngularVelocity.x, -(float)rosAngularVelocity.z, -(float)rosAngularVelocity.y);
    }

    public static Vector3Msg Unity2RosAngularVelocity(Vector3 unityAngularVelocity) {
        return new Vector3Msg(-unityAngularVelocity.x, -unityAngularVelocity.z, -unityAngularVelocity.y);
    }

}
