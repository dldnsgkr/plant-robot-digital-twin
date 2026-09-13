// Gazebo 로봇 포즈를 받아 Unity 모델을 움직인다.
// 1단계(현재): 외부에서 SetPoseGz() 호출 → 좌표 변환 후 적용
// 2단계: ROS-TCP-Connector 구독(/odom 또는 /tf) 으로 SetPoseGz 를 호출하도록 확장
using UnityEngine;

namespace PlantDT
{
    public class RobotPoseFollower : MonoBehaviour
    {
        [Tooltip("보간 시간(초). 0이면 즉시 적용")] public float smooth = 0.3f;
        [Tooltip("걷기 애니메이션 제어 (moving/fall 파라미터)")] public Animator animator;
        [Tooltip("이 속도(m/s) 이상이면 걷기 애니메이션")] public float movingThreshold = 0.04f;
        [Tooltip("애니메이션 재생 속도 = 실제속도 / 이 값")] public float animNominalSpeed = 0.5f;
        [Tooltip("높이(z) 흔들림 필터 시간(초)")] public float heightSmooth = 0.6f;
        [Tooltip("이동 중 모델 방향을 몸체 yaw 대신 이동 방향(속도 벡터)으로 잡아 보행 헌팅을 숨김")] public bool headingFromVelocity = false;   // 기본 끔: Gazebo 몸체 yaw 를 그대로 반영 (0.22 에서 요동 ±3.5° 로 충분히 작음)
        [Tooltip("방향 필터 시간(초)")] public float headingSmooth = 1.0f;
        Vector3 velFiltered; float headingYaw; bool headingInit;
        [Header("상태 (읽기 전용)")] public float speed;
        Vector3 lastTarget; float lastTargetTime; float speedFiltered;
        [Tooltip("프리팹 루트의 원래 회전 (모델을 세우는 값). yaw 는 여기에 합성된다")] public Quaternion baseRotation = Quaternion.identity;
        Vector3 targetPos; Quaternion targetRot; bool has;
        public Vector3 TargetPosition => targetPos;
        public Quaternion TargetRotation => targetRot;
        /// 수평 진행 방향 (프리팹 루트 회전과 무관한 yaw 기준)
        public Vector3 Forward { get; private set; } = Vector3.forward;

        static Vector3 Gz(float x, float y, float z) => new Vector3(-y, z, x);

        /// Gazebo 좌표(x,y,z m) 와 yaw(rad, z축 반시계) 로 목표 포즈 설정
        public void SetPoseGz(float x, float y, float z, float yaw)
        {
            var p = Gz(x, y, z);
            float now = Time.time;
            if (has && now > lastTargetTime + 1e-3f)
            {
                var d = p - lastTarget; d.y = 0f;
                float dt = now - lastTargetTime;
                float v = d.magnitude / dt;
                speedFiltered = Mathf.Lerp(speedFiltered, v, 0.2f);
                float kv = 1f - Mathf.Exp(-dt / Mathf.Max(headingSmooth, 1e-3f));
                velFiltered = Vector3.Lerp(velFiltered, d / dt, kv);
            }
            lastTarget = p; lastTargetTime = now;
            targetPos = p;
            // 모델 방향: 이동 중이면 필터된 속도 방향(직진 시 몸체 yaw 헌팅 ±10° 를 숨김), 정지 시 몸체 yaw
            float yawDeg = -yaw * Mathf.Rad2Deg;
            if (headingFromVelocity && speedFiltered > movingThreshold && velFiltered.sqrMagnitude > 1e-4f)
                yawDeg = Mathf.Atan2(velFiltered.x, velFiltered.z) * Mathf.Rad2Deg;
            if (!headingInit) { headingYaw = yawDeg; headingInit = true; }
            headingYaw = Mathf.LerpAngle(headingYaw, yawDeg, 0.25f);   // 37Hz 기준 약 0.1s 필터
            targetRot = Quaternion.Euler(0, headingYaw, 0) * baseRotation;
            var f = Quaternion.Euler(0, headingYaw, 0) * Vector3.forward; Forward = f;
            if (!has) { transform.position = targetPos; transform.rotation = targetRot; has = true; }
        }

        /// 넘어짐 애니메이션 트리거 (외부 이벤트에서 호출)
        public void TriggerFall() { if (animator != null) animator.SetTrigger("fall"); }

        void Update()
        {
            if (!has) return;
            speed = speedFiltered;
            if (animator != null)
            {
                bool moving = speed > movingThreshold;
                animator.SetBool("moving", moving);
                animator.speed = moving ? Mathf.Clamp(speed / Mathf.Max(animNominalSpeed, 0.05f), 0.1f, 3f) : 1f;   // 보폭 기준 정확 매칭 (미끄러짐 방지)
            }
            if (smooth <= 0) { transform.position = targetPos; transform.rotation = targetRot; return; }
            float k = 1f - Mathf.Exp(-Time.deltaTime / smooth);
            float kh = 1f - Mathf.Exp(-Time.deltaTime / Mathf.Max(heightSmooth, 1e-3f));
            var pos = Vector3.Lerp(transform.position, targetPos, k);
            pos.y = Mathf.Lerp(transform.position.y, targetPos.y, kh);   // 보행 중 몸체 상하 흔들림은 더 강하게 필터
            transform.position = pos;
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, k);
        }
    }
}
