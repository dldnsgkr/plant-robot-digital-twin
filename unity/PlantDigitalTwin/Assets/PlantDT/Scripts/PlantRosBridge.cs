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
        [Tooltip("Game 뷰 오른쪽 아래에 로봇 시점 PIP 표시")] public bool showRobotPov = true;
        [Header("상태 (읽기 전용)")] public int received;
        public Vector3 lastGzPos;

        [Tooltip("에디터 Play 의 CPU 점유를 줄여 같은 Mac 의 Gazebo RTF 저하를 막는다 (0=제한 없음)")] public int targetFrameRate = 30;

        void Start()
        {
            if (targetFrameRate > 0) { QualitySettings.vSyncCount = 0; Application.targetFrameRate = targetFrameRate; }
            if (robot == null) robot = FindFirstObjectByType<RobotPoseFollower>();
            ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>(odomTopic, OnOdom);
            if (showRobotPov && robot != null && FindFirstObjectByType<RobotPovCamera>() == null) RobotPovCamera.Create(robot);
        }

        // Game 뷰 좌상단 HUD: 연결·수신·로봇 포즈·속도·카메라 (스크린샷 대조용)
        void OnGUI()
        {
            var st = new GUIStyle(GUI.skin.label) { fontSize = 14, normal = { textColor = Color.white } };
            float spd = robot != null ? robot.speed : 0f;
            var cam = Camera.main != null ? Camera.main.transform.position : Vector3.zero;
            string txt = $"odom recv={received}  gz pos=({lastGzPos.x:F2}, {lastGzPos.y:F2}, {lastGzPos.z:F2})  yaw={lastYawDeg:F0}°  v={spd:F2} m/s\n" +
                         $"unity robot={(robot != null ? robot.transform.position : Vector3.zero)}  cam={cam}  moving={(spd > 0.04f)}";
            GUI.Box(new Rect(8, 30, 620, 46), "");
            GUI.Label(new Rect(14, 32, 610, 44), txt, st);
        }
        public float lastYawDeg;

        void OnOdom(OdometryMsg m)
        {
            var p = m.pose.pose.position; var q = m.pose.pose.orientation;
            // yaw (z축 회전) — ROS 쿼터니언 → yaw
            float yaw = Mathf.Atan2(2f * (float)(q.w * q.z + q.x * q.y), 1f - 2f * (float)(q.y * q.y + q.z * q.z));
            lastGzPos = new Vector3((float)p.x, (float)p.y, (float)p.z); received++; lastYawDeg = yaw * Mathf.Rad2Deg;
            if (robot != null) robot.SetPoseGz((float)p.x, (float)p.y, (float)p.z, yaw);
        }
    }
}
