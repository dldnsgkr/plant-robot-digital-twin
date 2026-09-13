// ROS-TCP-Connector 로 Gazebo/ROS 상태를 받아 Unity 씬에 반영한다.
//  - /model/go2/odometry (Odometry)  → RobotPoseFollower (로봇 위치·방향)
//  - /mission/state, /mission/event  → HUD 미션 단계, 이벤트 로그
//  - /gas/alarm, /gas/concentration, /gas/found, /gas/source_truth → 가스 분출 이펙트·농도 표시
//  - /inspection/gauge_value         → 게이지 판독값 표시 (게이지 패널 위 라벨)
//  - /robot/battery, /robot/docked   → 배터리·도킹
// 연결: 씬의 ROSConnection 오브젝트 (127.0.0.1:10000) ↔ 컨테이너의 ros_tcp_endpoint
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace PlantDT
{
    public class PlantRosBridge : MonoBehaviour
    {
        public string odomTopic = "/model/go2/odometry";
        public RobotPoseFollower robot;
        [Tooltip("Game 뷰 오른쪽 아래에 로봇 시점 PIP 표시")] public bool showRobotPov = true;
        [Tooltip("에디터 Play 의 CPU 점유를 줄여 같은 Mac 의 Gazebo RTF 저하를 막는다 (0=제한 없음)")] public int targetFrameRate = 30;
        [Tooltip("가스 분출 이펙트 오브젝트 (씬의 GasSpurt). 알람 시 누출원 위치로 옮겨 켠다")] public GameObject gasEffect;
        [Tooltip("게이지 패널 오브젝트 (판독값 라벨 위치)")] public Transform gaugePanel;

        [Header("상태 (읽기 전용)")] public int received;
        public Vector3 lastGzPos; public float lastYawDeg;
        public string missionState = ""; public float battery = -1f; public bool docked;
        public bool gasAlarm; public float gasPpm; public bool gasFound; public Vector3 gasSourceGz; public bool hasGasSource;
        public float gaugeBar = -1f; public float gaugeReadTime = -1f;
        readonly List<string> events = new();
        float t0 = -1f;

        static Vector3 Gz(float x, float y, float z) => new Vector3(-y, z, x);
        static readonly Dictionary<string, string> StateKo = new()
        {
            { "GOTO_GAUGE", "복도 순찰 → 게이지 접근" }, { "ALIGN", "게이지 방향 정렬" }, { "INSPECT", "압력계 판독" },
            { "GOTO_FACTORY", "공장 구역 이동" }, { "SEEK", "가스 누출원 탐색" }, { "WAIT_RTH", "자율 복귀·도킹 중" },
            { "DONE", "미션 종료 — 충전소 도킹 (다시 보려면 bash docker/run_mission.sh)" },
        };

        void Start()
        {
            if (targetFrameRate > 0) { QualitySettings.vSyncCount = 0; Application.targetFrameRate = targetFrameRate; }
            if (robot == null) robot = FindFirstObjectByType<RobotPoseFollower>();
            if (gasEffect == null) { var g = GameObject.Find("GasSpurt"); if (g == null) { foreach (var t in Resources.FindObjectsOfTypeAll<Transform>()) if (t.name == "GasSpurt" && t.gameObject.scene.IsValid()) { g = t.gameObject; break; } } gasEffect = g; }
            if (gaugePanel == null) { var p = GameObject.Find("GaugePanel"); if (p != null) gaugePanel = p.transform; }
            var ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<OdometryMsg>(odomTopic, OnOdom);
            ros.Subscribe<StringMsg>("/mission/state", m => OnState(m.data));
            ros.Subscribe<StringMsg>("/mission/event", m => AddEvent(m.data));
            ros.Subscribe<Float32Msg>("/robot/battery", m => battery = m.data);
            ros.Subscribe<BoolMsg>("/robot/docked", m => docked = m.data);
            ros.Subscribe<BoolMsg>("/gas/alarm", m => SetAlarm(m.data));
            ros.Subscribe<Float32Msg>("/gas/concentration", m => gasPpm = m.data);
            ros.Subscribe<BoolMsg>("/gas/found", m => { if (m.data && !gasFound) AddEvent("누출원 발견 보고"); gasFound = m.data; });
            ros.Subscribe<PointMsg>("/gas/source_truth", m => { gasSourceGz = new Vector3((float)m.x, (float)m.y, (float)m.z); hasGasSource = true; });
            ros.Subscribe<Float32Msg>("/inspection/gauge_value", m => { gaugeBar = m.data; gaugeReadTime = Time.time; });
            if (showRobotPov && robot != null && FindFirstObjectByType<RobotPovCamera>() == null) RobotPovCamera.Create(robot);
        }

        void OnState(string s)
        {
            if (s == missionState) return;
            missionState = s;
            // 알람 메시지를 놓쳐도(접속 전 발행) 상태로 보완: SEEK 이후는 알람 중, DONE 이면 해제
            if (s == "SEEK") SetAlarm(true);
            if (s == "DONE") SetAlarm(false);
        }

        void SetAlarm(bool on)
        {
            if (on == gasAlarm) return;
            gasAlarm = on;
            if (gasEffect != null)
            {
                if (on && hasGasSource) gasEffect.transform.position = Gz(gasSourceGz.x, gasSourceGz.y, Mathf.Max(gasSourceGz.z, 0.3f));
                gasEffect.SetActive(on);
                var ps = gasEffect.GetComponentInChildren<ParticleSystem>(true); if (ps != null) { if (on) ps.Play(true); else ps.Stop(true); }
            }
            AddEvent(on ? "가스 알람 발령 — 누출원 탐색 시작" : "가스 알람 해제");
        }

        void AddEvent(string text)
        {
            if (t0 < 0) t0 = Time.time;
            events.Add($"[{Time.time - t0,4:F0}s] {text}");
            if (events.Count > 9) events.RemoveAt(0);
        }

        void OnOdom(OdometryMsg m)
        {
            var p = m.pose.pose.position; var q = m.pose.pose.orientation;
            float yaw = Mathf.Atan2(2f * (float)(q.w * q.z + q.x * q.y), 1f - 2f * (float)(q.y * q.y + q.z * q.z));
            lastGzPos = new Vector3((float)p.x, (float)p.y, (float)p.z); received++; lastYawDeg = yaw * Mathf.Rad2Deg;
            if (robot != null) robot.SetPoseGz((float)p.x, (float)p.y, (float)p.z, yaw);
        }

        void OnGUI()
        {
            var st = new GUIStyle(GUI.skin.label) { fontSize = 14, normal = { textColor = Color.white } };
            var big = new GUIStyle(GUI.skin.label) { fontSize = 20, fontStyle = FontStyle.Bold, normal = { textColor = Color.white }, alignment = TextAnchor.MiddleCenter };
            float spd = robot != null ? robot.speed : 0f;
            string stKo = StateKo.TryGetValue(missionState, out var k) ? k : (string.IsNullOrEmpty(missionState) ? "(미션 상태 수신 대기)" : missionState);
            string txt = $"odom recv={received}  gz pos=({lastGzPos.x:F2}, {lastGzPos.y:F2}, {lastGzPos.z:F2})  yaw={lastYawDeg:F0}°  v={spd:F2} m/s\n" +
                         $"미션: {stKo}   배터리: {(battery < 0 ? "-" : battery.ToString("F0") + "%")}{(docked ? "  ⚡도킹" : "")}\n" +
                         $"압력계: {(gaugeBar < 0 ? "-" : gaugeBar.ToString("F2") + " bar")}   가스: {gasPpm:F1} ppm{(gasAlarm ? "  ⚠ 알람" : "")}{(gasFound ? "  ✔ 누출원 발견" : "")}";
            GUI.Box(new Rect(8, 30, 620, 84), ""); GUI.Label(new Rect(14, 32, 610, 82), txt, st);

            // 가스 알람 배너
            if (gasAlarm)
            {
                var c = GUI.color; GUI.color = new Color(1f, 0.25f, 0.2f, 0.85f);
                GUI.Box(new Rect(Screen.width / 2 - 220, 8, 440, 34), "");
                GUI.color = c; GUI.Label(new Rect(Screen.width / 2 - 220, 8, 440, 34), $"⚠ GAS ALARM  {gasPpm:F1} ppm{(gasFound ? "  — 누출원 발견" : "  — 누출원 탐색 중")}", big);
            }

            // 이벤트 로그 (왼쪽 아래)
            float h = 22 + events.Count * 18;
            GUI.Box(new Rect(8, Screen.height - h - 8, 520, h), ""); GUI.Label(new Rect(14, Screen.height - h - 6, 510, 18), "EVENT LOG", st);
            for (int i = 0; i < events.Count; i++) GUI.Label(new Rect(14, Screen.height - h + 12 + i * 18, 510, 18), events[i], st);

            // 게이지 판독값: 패널 위 월드 라벨 (판독 후 20초 강조)
            if (gaugePanel != null && gaugeBar >= 0 && Camera.main != null)
            {
                var sp = Camera.main.WorldToScreenPoint(gaugePanel.position + Vector3.up * 0.45f);
                if (sp.z > 0)
                {
                    bool fresh = Time.time - gaugeReadTime < 20f;
                    var c = GUI.color; GUI.color = fresh ? new Color(1f, 0.9f, 0.2f, 0.95f) : new Color(1f, 1f, 1f, 0.7f);
                    GUI.Box(new Rect(sp.x - 70, Screen.height - sp.y - 16, 140, 32), "");
                    GUI.color = c; GUI.Label(new Rect(sp.x - 70, Screen.height - sp.y - 16, 140, 32), $"{gaugeBar:F2} bar", big);
                }
            }
        }
    }
}
