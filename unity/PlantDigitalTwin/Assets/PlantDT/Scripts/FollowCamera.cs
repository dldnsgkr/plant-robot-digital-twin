// 로봇 3인칭 추적 카메라 — 로봇 뒤·위에서 진행 방향을 바라본다 (Game 뷰용)
using UnityEngine;

namespace PlantDT
{
    public class FollowCamera : MonoBehaviour
    {
        public Transform target;
        public Vector3 offset = new Vector3(0f, 1.4f, -2.2f);   // 로컬: 위 1.4m, 뒤 2.2m (스폰 55 → 57.2, 복도 끝벽 57.5 안쪽)
        public float smooth = 0.25f;

        void LateUpdate()
        {
            if (target == null) return;
            var want = target.TransformPoint(offset);
            float k = smooth <= 0 ? 1f : 1f - Mathf.Exp(-Time.deltaTime / smooth);
            transform.position = Vector3.Lerp(transform.position, want, k);
            var look = Quaternion.LookRotation((target.position + Vector3.up * 0.4f) - transform.position, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, look, k);
        }
    }
}
