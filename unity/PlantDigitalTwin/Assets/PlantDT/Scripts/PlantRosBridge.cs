// ROS-TCP-Connector 로 Gazebo 로봇 포즈를 받아 Unity Spot 모델에 반영한다.
// 구독: /model/go2/odometry (nav_msgs/Odometry, Gazebo ground-truth) → RobotPoseFollower.SetPoseGz
// 연결: 씬의 ROSConnection 오브젝트 (127.0.0.1:10000) ↔ 컨테이너의 ros_tcp_endpoint
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace PlantDT
{
    public class PlantRosBridge : MonoBehaviour
    {
        public string odomTopic = "/model/go2/odometry";
        public RobotPoseFollower robot;
        [Header("상태 (읽기 전용)")] public int received;
        public Vector3 lastGzPos;

        void Start()
        {
            if (robot == null) robot = FindFirstObjectByType<RobotPoseFollower>();
            ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>(odomTopic, OnOdom);
        }

        void OnOdom(OdometryMsg m)
        {
            var p = m.pose.pose.position; var q = m.pose.pose.orientation;
            // yaw (z축 회전) — ROS 쿼터니언 → yaw
            float yaw = Mathf.Atan2(2f * (float)(q.w * q.z + q.x * q.y), 1f - 2f * (float)(q.y * q.y + q.z * q.z));
            lastGzPos = new Vector3((float)p.x, (float)p.y, (float)p.z); received++;
            if (robot != null) robot.SetPoseGz((float)p.x, (float)p.y, (float)p.z, yaw);
        }
    }
}
