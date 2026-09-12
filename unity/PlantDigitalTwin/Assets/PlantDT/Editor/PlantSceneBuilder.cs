// 제공 에셋 프리팹을 Gazebo 월드(simulation/worlds/plant_world.sdf)와 같은 배치로 씬에 놓는다.
// 실행: 메뉴 PlantDT > Build Plant Scene  또는
//   Unity -batchmode -quit -projectPath <proj> -executeMethod PlantDT.PlantSceneBuilder.Build
//
// 좌표 규약 (ROS-TCP-Connector 의 FLU→RUF 와 동일):
//   Unity.x = -Gazebo.y,  Unity.y = Gazebo.z,  Unity.z = Gazebo.x
// 배치 근거는 Gazebo 쪽과 동일: 프리팹 렌더 바운드를 박스 월드 치수에 정렬
//   공장 25×18 (중심 0,0), 복도 45×3.5 (x 12.5~57.5), 가스탱크 (-9,6), 팔레트 (-6,4)/(3,-5)
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace PlantDT
{
    public static class PlantSceneBuilder
    {
        const string ScenePath = "Assets/PlantDT/Scenes/PlantDigitalTwin.unity";

        static Vector3 Gz(float x, float y, float z) => new Vector3(-y, z, x);   // Gazebo → Unity

        [MenuItem("PlantDT/Build Plant Scene")]
        public static void Build()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            Directory.CreateDirectory("Assets/PlantDT/Scenes");

            // ---- 건물 ----
            var factory = Place("Map/factory_inner_1", "Factory");
            AlignBounds(factory, centerGz: new Vector3(0, 0, 0), sizeGz: new Vector3(25f, 18f, 0), scaleToFit: false);
            var corridor = Place("Map/factory_hall_1", "Corridor");
            AlignBounds(corridor, centerGz: new Vector3(35f, 0, 0), sizeGz: new Vector3(45f, 3.5f, 0), scaleToFit: true);

            // ---- 설비·장애물 ----
            var tank = Place("object/fac_gastank_1", "GasTank");
            AlignBounds(tank, centerGz: new Vector3(-9f, 6f, 0), sizeGz: Vector3.zero, scaleToFit: false, uniformScale: 0.45f);
            var p1 = Place("object/obs_palette_1", "Pallet_1");
            AlignBounds(p1, centerGz: new Vector3(-6f, 4f, 0), sizeGz: Vector3.zero, scaleToFit: false); p1.transform.Rotate(0, -Mathf.Rad2Deg * 0.3f, 0);
            var p2 = Place("object/obs_palette_1", "Pallet_2");
            AlignBounds(p2, centerGz: new Vector3(3f, -5f, 0), sizeGz: Vector3.zero, scaleToFit: false); p2.transform.Rotate(0, Mathf.Rad2Deg * 0.6f, 0);

            // ---- 로봇 (Gazebo 스폰 55,0 / 공장 방향(-x) 바라봄) ----
            var robot = Place("object/robot_spot_1", "Robot");
            if (robot != null)
            {
                AlignBounds(robot, centerGz: new Vector3(55f, 0, 0), sizeGz: Vector3.zero, scaleToFit: false);
                robot.transform.rotation = Quaternion.LookRotation(Gz(-1, 0, 0) - Vector3.zero, Vector3.up);
                if (robot.GetComponent<RobotPoseFollower>() == null) robot.AddComponent<RobotPoseFollower>();
            }

            // ---- 조명·환경 세팅 (에셋 제공 프리팹) + 가스 분출 이펙트 (가스탱크 옆, 초기 비활성) ----
            Place("effect/Doosan_MapSetting", "MapSetting");
            var gas = Place("effect/gas_spurt_1", "GasSpurt");
            if (gas != null) { gas.transform.position = Gz(-7.5f, 6f, 1.0f); gas.SetActive(false); }

            // ---- ROS 연결 (ros_tcp_endpoint, 컨테이너 포트 10000) + 포즈 브리지 ----
            var rosGo = new GameObject("ROSConnection");
            var ros = rosGo.AddComponent<Unity.Robotics.ROSTCPConnector.ROSConnection>();
            ros.RosIPAddress = "127.0.0.1"; ros.RosPort = 10000; ros.ConnectOnStart = true;
            var bridge = new GameObject("PlantRosBridge").AddComponent<PlantRosBridge>();
            if (robot != null) bridge.robot = robot.GetComponent<RobotPoseFollower>();

            // ---- 카메라: 복도 입구 위에서 공장 쪽 조망 ----
            var cam = Camera.main;
            if (cam != null) { cam.transform.position = Gz(20f, -6f, 6f); cam.transform.LookAt(Gz(0f, 0f, 1f)); cam.farClipPlane = 300f; }

            EditorSceneManager.SaveScene(scene, ScenePath);
            var scenes = EditorBuildSettings.scenes.Where(s => s.path != ScenePath).ToList();
            scenes.Insert(0, new EditorBuildSettingsScene(ScenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
            Debug.Log($"[PlantDT] scene built: {ScenePath}");
        }

        static GameObject Place(string prefabRelPath, string name)
        {
            var guids = AssetDatabase.FindAssets($"t:Prefab {Path.GetFileName(prefabRelPath)}");
            var path = guids.Select(AssetDatabase.GUIDToAssetPath)
                            .FirstOrDefault(p => p.EndsWith(prefabRelPath + ".prefab"));
            if (path == null) { Debug.LogWarning($"[PlantDT] prefab not found: {prefabRelPath}"); return null; }
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            go.name = name;
            return go;
        }

        // 렌더 바운드의 바닥 중심을 Gazebo 좌표 centerGz(z=0 바닥) 에 맞추고, 필요시 xy 크기를 sizeGz 에 맞춰 스케일
        static void AlignBounds(GameObject go, Vector3 centerGz, Vector3 sizeGz, bool scaleToFit, float uniformScale = 1f)
        {
            if (go == null) return;
            go.transform.position = Vector3.zero; go.transform.rotation = Quaternion.identity;
            go.transform.localScale = Vector3.one * uniformScale;
            var rs = go.GetComponentsInChildren<Renderer>(true);
            if (rs.Length == 0) { go.transform.position = Gz(centerGz.x, centerGz.y, centerGz.z); return; }
            var b = rs[0].bounds; foreach (var r in rs) b.Encapsulate(r.bounds);
            if (scaleToFit && sizeGz.x > 0 && sizeGz.y > 0)
            {
                // Gazebo x → Unity z, Gazebo y → Unity x
                var s = go.transform.localScale;
                s.z *= sizeGz.x / b.size.z; s.x *= sizeGz.y / b.size.x;
                go.transform.localScale = s;
                b = rs[0].bounds; foreach (var r in rs) b.Encapsulate(r.bounds);
            }
            var bottomCenter = new Vector3(b.center.x, b.min.y, b.center.z);
            go.transform.position += Gz(centerGz.x, centerGz.y, centerGz.z) - bottomCenter;
            Debug.Log($"[PlantDT] {go.name}: bounds size={b.size} → placed at gz({centerGz.x},{centerGz.y})");
        }
    }
}
