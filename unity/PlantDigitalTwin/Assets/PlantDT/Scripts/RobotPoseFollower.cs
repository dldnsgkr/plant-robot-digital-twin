// Gazebo 로봇 포즈를 받아 Unity 모델을 움직인다.
// 1단계(현재): 외부에서 SetPoseGz() 호출 → 좌표 변환 후 적용
// 2단계: ROS-TCP-Connector 구독(/odom 또는 /tf) 으로 SetPoseGz 를 호출하도록 확장
using UnityEngine;

namespace PlantDT
{
    public class RobotPoseFollower : MonoBehaviour
    {
        [Tooltip("보간 시간(초). 0이면 즉시 적용")] public float smooth = 0.15f;
        Vector3 targetPos; Quaternion targetRot; bool has;

        static Vector3 Gz(float x, float y, float z) => new Vector3(-y, z, x);

        /// Gazebo 좌표(x,y,z m) 와 yaw(rad, z축 반시계) 로 목표 포즈 설정
        public void SetPoseGz(float x, float y, float z, float yaw)
        {
            targetPos = Gz(x, y, z);
            targetRot = Quaternion.Euler(0, -yaw * Mathf.Rad2Deg, 0);
            if (!has) { transform.position = targetPos; transform.rotation = targetRot; has = true; }
        }

        void Update()
        {
            if (!has) return;
            if (smooth <= 0) { transform.position = targetPos; transform.rotation = targetRot; return; }
            float k = 1f - Mathf.Exp(-Time.deltaTime / smooth);
            transform.position = Vector3.Lerp(transform.position, targetPos, k);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, k);
        }
    }
}
