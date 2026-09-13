// 로봇 시점(POV) 카메라 — Game 뷰 오른쪽 아래 PIP(picture-in-picture)로 표시.
// 로봇 머리 위치에서 진행 방향(RobotPoseFollower.Forward)을 바라본다. Play 시 PlantRosBridge 가 자동 생성.
using UnityEngine;

namespace PlantDT
{
    public class RobotPovCamera : MonoBehaviour
    {
        public RobotPoseFollower robot;
        [Tooltip("로봇 기준 위치: 앞(m), 위(m)")] public float forward = 0.35f, up = 0.30f;
        [Tooltip("화면 내 위치·크기 (뷰포트 비율)")] public Rect viewport = new Rect(0.70f, 0.04f, 0.28f, 0.28f);
        public float fov = 90f;
        Camera cam;

        public static RobotPovCamera Create(RobotPoseFollower robot)
        {
            var go = new GameObject("RobotPOVCamera");
            var pov = go.AddComponent<RobotPovCamera>(); pov.robot = robot;
            pov.cam = go.AddComponent<Camera>();
            pov.cam.rect = pov.viewport; pov.cam.depth = 10; pov.cam.fieldOfView = pov.fov;
            pov.cam.nearClipPlane = 0.05f; pov.cam.farClipPlane = 200f;
            return pov;
        }

        void LateUpdate()
        {
            if (robot == null) return;
            var fwd = robot.Forward; if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
            transform.position = robot.transform.position + fwd * forward + Vector3.up * up;
            transform.rotation = Quaternion.LookRotation(fwd, Vector3.up);
        }

        void OnGUI()
        {
            // PIP 테두리 + 라벨
            var r = new Rect(viewport.x * Screen.width - 2, (1 - viewport.y - viewport.height) * Screen.height - 2,
                             viewport.width * Screen.width + 4, viewport.height * Screen.height + 4);
            GUI.Box(r, ""); GUI.Label(new Rect(r.x + 6, r.y + 2, 200, 20), "ROBOT POV");
        }
    }
}
