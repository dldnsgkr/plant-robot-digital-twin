// 로봇 3인칭 추적 카메라 — 로봇 뒤·위에서 진행 방향을 바라본다 (Game 뷰용)
using UnityEngine;

namespace PlantDT
{
    public class FollowCamera : MonoBehaviour
    {
        public Transform target;
        public Vector3 offset = new Vector3(0f, 1.6f, -2.4f);   // 위 1.6m, 뒤 2.4m (스폰 55 → 57.4, 복도 끝벽 57.5 안쪽)
        public float smooth = 0.25f;
        [Tooltip("카메라 기준 방향 필터(초). 로봇 몸체의 보행 요동(±8°)이 카메라에 전달되지 않도록 길게")] public float headingSmooth = 3.0f;

        RobotPoseFollower follower; Vector3 fwdSmooth = Vector3.forward;

        void Start() { if (target != null) follower = target.GetComponent<RobotPoseFollower>(); }

        void LateUpdate()
        {
            if (target == null) return;
            // 프리팹 루트 회전(모델 세우기용)에 영향받지 않도록 수평 진행 방향만 사용
            var fwd = follower != null ? follower.Forward : Vector3.ProjectOnPlane(target.forward, Vector3.up);
            if (fwd.sqrMagnitude < 1e-4f) fwd = fwdSmooth;
            fwdSmooth = Vector3.Slerp(fwdSmooth, fwd.normalized, 1f - Mathf.Exp(-Time.deltaTime / Mathf.Max(headingSmooth, 1e-3f)));
            if (follower != null && follower.Teleported)   // 로봇 순간이동 시 카메라도 즉시 따라감
            {
                follower.Teleported = false; fwdSmooth = fwd.normalized;
                transform.position = target.position + Quaternion.LookRotation(fwdSmooth, Vector3.up) * offset;
                transform.LookAt(target.position + Vector3.up * 0.4f);
            }
            var frame = Quaternion.LookRotation(fwdSmooth, Vector3.up);
            var want = target.position + frame * offset;
            float k = smooth <= 0 ? 1f : 1f - Mathf.Exp(-Time.deltaTime / smooth);
            transform.position = Vector3.Lerp(transform.position, want, k);
            var look = Quaternion.LookRotation((target.position + Vector3.up * 0.4f) - transform.position, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, look, k);
        }
    }
}
