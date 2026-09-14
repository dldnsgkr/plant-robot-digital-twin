// 제공 에셋 프리팹을 Gazebo 월드(simulation/worlds/plant_world.sdf)와 같은 배치로 씬에 놓는다.
// 실행: 메뉴 PlantDT > Build Plant Scene  또는
//   Unity -batchmode -quit -projectPath <proj> -executeMethod PlantDT.PlantSceneBuilder.Build
//
// 좌표 규약 (ROS-TCP-Connector 의 FLU→RUF 와 동일):
//   Unity.x = -Gazebo.y,  Unity.y = Gazebo.z,  Unity.z = Gazebo.x
// 배치 근거는 Gazebo 쪽과 동일: 프리팹 렌더 바운드를 박스 월드 치수에 정렬
//   공장 25×18 (중심 0,0), 복도 45×3.5 (x 12.5~57.5), 가스탱크 (-9,6), 팔레트 (-6,4)/(3,-5)
using System.IO;
using System.Globalization;
using System.Xml;
using System.Linq;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Animations;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace PlantDT
{
    public static class PlantSceneBuilder
    {
        const string ScenePath = "Assets/PlantDT/Scenes/PlantDigitalTwin.unity";

        static Vector3 Gz(float x, float y, float z) => new Vector3(-y, z, x);   // Gazebo → Unity

        [MenuItem("PlantDT/Build Plant Scene")]
        public static void Build()
        {
            // ROS-TCP-Connector 를 ROS2 프로토콜로 (Robotics > ROS Settings 와 동일한 효과)
            var target = UnityEditor.Build.NamedBuildTarget.Standalone;
            var defines = PlayerSettings.GetScriptingDefineSymbols(target);
            if (!defines.Split(';').Contains("ROS2")) PlayerSettings.SetScriptingDefineSymbols(target, string.IsNullOrEmpty(defines) ? "ROS2" : defines + ";ROS2");

            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            Directory.CreateDirectory("Assets/PlantDT/Scenes");

            // ---- 건물 ----
            // 두 건물 프리팹은 FBX 로컬 +x 가 긴 변이고 그쪽(공장 동벽·복도 근단)에 문이 있다.
            // Gazebo x(동) = Unity +z 이므로 로컬 +x → 월드 +z 가 되도록 Y축 -90° 회전 후 정렬한다.
            var factory = Place("Map/factory_inner_1", "Factory");
            OrientLongAxisToZ(factory);
            AlignBounds(factory, centerGz: new Vector3(0, 0, 0), sizeGz: new Vector3(25f, 18f, 0), scaleToFit: false);
            // Gazebo 월드(plant_assets/convert.py)와 동일한 절단: 로봇 경로와 겹치는 동측 스트립 구조물 제거
            //  - FC_Fence_2(동측 구조물), FC_Fence_01_3(펜스): 오브젝트 비활성
            //  - FC_Fence_01 속 내부 칸막이벽(gz x≈7.0~7.7), FC_Fence_01_6 남벽 앞 설비(gz x -7.7~-5.8, y -8.65~-6.8): 삼각형 절단
            //  (프리팹 자식 이름이 FBX 노드명과 달라 이름 대신 월드 좌표 범위로 선택)
            DisableRenderersInside(factory, GzBox(7.0f, 12.55f, -8.6f, 8.6f, -0.1f, 6.5f));   // (별도 오브젝트일 때) 동측 구조물
            CutTriangles(factory, GzBox(7.0f, 7.73f, -8.8f, 8.8f, 0f, 7.5f));                  // 내부 칸막이벽 (전체 높이)
            CutTriangles(factory, GzBox(7.3f, 12.4f, -8.5f, 8.5f, 0.02f, 6.0f));               // 동측 스트립 구조물·펜스 (바닥·동벽·지붕 제외)
            CutTriangles(factory, GzBox(-7.68f, -5.78f, -8.65f, -6.8f, 0f, 3.0f));             // 남벽 앞 설비
            var corridor = Place("Map/factory_hall_1", "Corridor");
            OrientLongAxisToZ(corridor);
            AlignBounds(corridor, centerGz: new Vector3(35f, 0, 0), sizeGz: new Vector3(45f, 3.5f, 0), scaleToFit: true);
            // 바닥면 보정: AlignBounds 는 바운드 최하단을 0 에 두는데, 메시에 바닥 두께가 있어 바닥 윗면이 떠오른다
            //   복도: 바닥 슬래브 -0.90~0.04 → 윗면이 0.94 에 놓이므로 0.935 내림 / 공장: -0.07 → 0.07 내림
            if (corridor != null) corridor.transform.position += Vector3.down * 0.935f;
            if (factory != null) factory.transform.position += Vector3.down * 0.07f;

            // ---- 복도 평바닥: Gazebo 물리 세계와 일치 ----
            // 복도 에셋 끝부분(x 42~54m)은 경사로로 0.8m 내려가는 저지대 + 계단이지만, Gazebo 는 z=0 평면 지면이
            // 그 위를 덮고 있어 로봇이 평지를 걷는다. Unity 도 같은 높이의 바닥을 깔아 로봇이 허공을 걷지 않게 한다.
            var floor = GameObject.CreatePrimitive(PrimitiveType.Plane); floor.name = "CorridorFloor";
            Object.DestroyImmediate(floor.GetComponent<Collider>());
            floor.transform.position = Gz(35f, 0f, 0.005f);
            floor.transform.localScale = new Vector3(3.5f / 10f, 1f, 45f / 10f);           // Plane 기본 10×10 m
            var floorMat = AssetDatabase.FindAssets("Floor_01 t:Material").Select(AssetDatabase.GUIDToAssetPath)
                                        .Select(AssetDatabase.LoadAssetAtPath<Material>).FirstOrDefault(m => m != null);
            if (floorMat != null)
            {
                var inst = new Material(floorMat); inst.name = "CorridorFloor_Mat";
                if (inst.HasProperty("_BaseColorMap")) inst.SetTextureScale("_BaseColorMap", new Vector2(3.5f / 2f, 45f / 2f));
                AssetDatabase.CreateAsset(inst, "Assets/PlantDT/CorridorFloor_Mat.mat");
                floor.GetComponent<Renderer>().sharedMaterial = inst;
            }

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
                var follower = robot.GetComponent<RobotPoseFollower>() ?? robot.AddComponent<RobotPoseFollower>();
                follower.baseRotation = robot.transform.rotation;                       // 프리팹 루트 회전 보존
                follower.SetPoseGz(55f, 0f, 0f, Mathf.PI);                                 // Gazebo 스폰: 공장(-x) 방향
                robot.transform.rotation = follower.TargetRotation; robot.transform.position = follower.TargetPosition;
                follower.animator = SetupSpotAnimator(robot);
            }

            // ---- Gazebo 월드의 박스·실린더 장애물(계단·파이프·크레이트·드럼통·기계·충전소·스팀트랩)을 SDF 에서 읽어 배치 ----
            BuildGazeboObstacles();

            // ---- 게이지 패널: Gazebo gauge_panel (38, 1.62, 0.6) 과 같은 자리, 복도 북벽 안쪽면에 눈금판 ----
            BuildGaugePanel();

            // ---- 조명·환경 세팅 (에셋 제공 프리팹) + 가스 분출 이펙트 (가스탱크 옆, 초기 비활성) ----
            Place("effect/Doosan_MapSetting", "MapSetting");
            var gas = Place("effect/gas_spurt_1", "GasSpurt");
            if (gas != null) { gas.transform.position = Gz(-7.5f, 6f, 0f); FixGasEffect(gas); gas.SetActive(false); }

            // ---- ROS 연결 (ros_tcp_endpoint, 컨테이너 포트 10000) + 포즈 브리지 ----
            var rosGo = new GameObject("ROSConnection");
            var ros = rosGo.AddComponent<Unity.Robotics.ROSTCPConnector.ROSConnection>();
            ros.RosIPAddress = "127.0.0.1"; ros.RosPort = 10000; ros.ConnectOnStart = true;
            // 엔드포인트(컨테이너) 재시작으로 소켓이 죽었을 때 Play 정지/재접속에서 메인 스레드가 오래 막히지 않도록 짧은 타임아웃
            ros.NetworkTimeoutSeconds = 2f; ros.KeepaliveTime = 1f; ros.SleepTimeSeconds = 0.05f;
            var bridge = new GameObject("PlantRosBridge").AddComponent<PlantRosBridge>();
            if (robot != null) bridge.robot = robot.GetComponent<RobotPoseFollower>();

            // ---- 야간 실내 씬: 기본 태양광 제거, 노출 범위 제한 볼륨 추가 ----
            // MapSetting 의 HDRI 는 야간 배경이지만 하늘 조명이 20,000 lux 라 건물 밖에서 보면 지붕이 과다노출된다.
            // 실내(로봇 시점)에서 보는 것이 기준이므로 자동 노출의 상한을 제한해 날림을 막는다.
            var sun = GameObject.Find("Directional Light");
            if (sun != null) Object.DestroyImmediate(sun);
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(profile, "Assets/PlantDT/PlantDTVolume.asset");
            var exposure = profile.Add<Exposure>(true);
            exposure.mode.Override(ExposureMode.Automatic);
            exposure.limitMin.Override(3f); exposure.limitMax.Override(11f);
            exposure.adaptationSpeedDarkToLight.Override(4f); exposure.adaptationSpeedLightToDark.Override(4f);
            var volGo = new GameObject("PlantDT Volume");
            var vol = volGo.AddComponent<Volume>(); vol.isGlobal = true; vol.priority = 10; vol.sharedProfile = profile;

            // ---- 실내 조명: Gazebo 월드와 같은 위치의 포인트 라이트 (공장 2, 복도 2) ----
            AddPointLight("FactoryLight_1", Gz(-5f, 0f, 6f), 20000f, 25f);
            AddPointLight("FactoryLight_2", Gz(5f, 0f, 6f), 20000f, 25f);
            AddPointLight("CorridorLight_1", Gz(25f, 0f, 3.6f), 6000f, 18f);
            AddPointLight("CorridorLight_2", Gz(45f, 0f, 3.6f), 6000f, 18f);

            // ---- 카메라: 로봇 3인칭 추적 (Play 시) / 초기 위치는 복도 스폰 뒤 ----
            var cam = Camera.main;
            if (cam != null)
            {
                cam.farClipPlane = 300f;
                var fc = cam.gameObject.AddComponent<FollowCamera>();
                if (robot != null) { fc.target = robot.transform; cam.transform.position = robot.transform.TransformPoint(fc.offset); cam.transform.LookAt(robot.transform.position + Vector3.up * 0.4f); }
            }

            EditorSceneManager.SaveScene(scene, ScenePath);
            var scenes = EditorBuildSettings.scenes.Where(s => s.path != ScenePath).ToList();
            scenes.Insert(0, new EditorBuildSettingsScene(ScenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
            Debug.Log($"[PlantDT] scene built: {ScenePath}");
        }

        static void AddPointLight(string name, Vector3 pos, float lumen, float range)
        {
            var go = new GameObject(name); go.transform.position = pos;
            var l = go.AddComponent<Light>(); l.type = LightType.Point; l.range = range; l.color = new Color(1f, 0.95f, 0.85f);
            var hd = go.AddComponent<HDAdditionalLightData>();
            hd.SetIntensity(lumen, LightUnit.Lumen);
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

        // 제공 애니메이션(spot_move 걷기, spot_fall 넘어짐)으로 Animator Controller 생성·연결
        //   파라미터: moving(bool) → Idle↔Move,  fall(trigger) → Fall → Idle
        //   루트 모션은 끈다: 위치·방향은 ROS 포즈가 결정한다
        static Animator SetupSpotAnimator(GameObject robot)
        {
            var move = LoadClip("spot_move"); var fall = LoadClip("spot_fall_1");
            if (move != null) { var st = AnimationUtility.GetAnimationClipSettings(move); st.loopTime = true; AnimationUtility.SetAnimationClipSettings(move, st); }
            const string path = "Assets/PlantDT/SpotAnimator.controller";
            var ctrl = AnimatorController.CreateAnimatorControllerAtPath(path);
            ctrl.AddParameter("moving", AnimatorControllerParameterType.Bool);
            ctrl.AddParameter("fall", AnimatorControllerParameterType.Trigger);
            var sm = ctrl.layers[0].stateMachine;
            var idle = sm.AddState("Idle"); idle.motion = move; idle.speed = 0f;      // 걷기 첫 프레임에서 정지
            var walk = sm.AddState("Move"); walk.motion = move;
            var fallSt = sm.AddState("Fall"); fallSt.motion = fall;
            sm.defaultState = idle;
            var t1 = idle.AddTransition(walk); t1.AddCondition(AnimatorConditionMode.If, 0, "moving"); t1.hasExitTime = false; t1.duration = 0.15f;
            var t2 = walk.AddTransition(idle); t2.AddCondition(AnimatorConditionMode.IfNot, 0, "moving"); t2.hasExitTime = false; t2.duration = 0.15f;
            var t3 = sm.AddAnyStateTransition(fallSt); t3.AddCondition(AnimatorConditionMode.If, 0, "fall"); t3.hasExitTime = false; t3.duration = 0.1f;
            var t4 = fallSt.AddTransition(idle); t4.hasExitTime = true; t4.exitTime = 1f; t4.duration = 0.2f;
            var anim = robot.GetComponent<Animator>() ?? robot.AddComponent<Animator>();
            anim.runtimeAnimatorController = ctrl; anim.applyRootMotion = false;
            // 제자리 걷기 클립의 자연 전진 속도 = 발의 한 사이클 수평 왕복 거리(보폭) / 주기 → 발 미끄러짐 방지 기준값
            var follower = robot.GetComponent<RobotPoseFollower>();
            if (move != null && follower != null) follower.animNominalSpeed = EstimateWalkSpeed(robot, move);
            Debug.Log($"[PlantDT] Spot animator: move={(move != null)} fall={(fall != null)} avatar={(anim.avatar != null ? anim.avatar.name : "none")}");
            return anim;
        }

        static float EstimateWalkSpeed(GameObject robot, AnimationClip clip)
        {
            var body = robot.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == "Spot_Body_Bone_01");
            var feet = robot.GetComponentsInChildren<Transform>(true).Where(t => t.name.EndsWith("Hand_Bone_00")).ToList();
            if (body == null || feet.Count == 0) { Debug.LogWarning("[PlantDT] walk speed: bones not found"); return 0.5f; }
            const int N = 40; var minA = new float[feet.Count]; var maxA = new float[feet.Count];
            for (int i = 0; i < feet.Count; i++) { minA[i] = float.MaxValue; maxA[i] = float.MinValue; }
            var savedPos = robot.transform.position; var savedRot = robot.transform.rotation;
            robot.transform.position = Vector3.zero; robot.transform.rotation = Quaternion.identity;
            for (int k = 0; k < N; k++)
            {
                clip.SampleAnimation(robot, clip.length * k / N);
                for (int i = 0; i < feet.Count; i++)
                {
                    var rel = body.InverseTransformPoint(feet[i].position);          // 몸체 기준 발 위치
                    float along = new Vector2(rel.x, rel.z).magnitude * Mathf.Sign(rel.z == 0 ? rel.x : rel.z); // 수평 투영
                    minA[i] = Mathf.Min(minA[i], along); maxA[i] = Mathf.Max(maxA[i], along);
                }
            }
            clip.SampleAnimation(robot, 0f); robot.transform.position = savedPos; robot.transform.rotation = savedRot;
            float stride = 0f; for (int i = 0; i < feet.Count; i++) stride += (maxA[i] - minA[i]); stride /= feet.Count;
            float v = stride / clip.length;
            Debug.Log($"[PlantDT] walk clip: feet={feet.Count} stride≈{stride:F3} m, period={clip.length:F3} s → nominal speed≈{v:F3} m/s");
            return Mathf.Max(v, 0.05f);
        }

        static AnimationClip LoadClip(string name)
        {
            var g = AssetDatabase.FindAssets($"t:AnimationClip {name}").Select(AssetDatabase.GUIDToAssetPath).FirstOrDefault(p => p.EndsWith($"/{name}.anim"));
            return g == null ? null : AssetDatabase.LoadAssetAtPath<AnimationClip>(g);
        }

        // Gazebo 축 박스(x0~x1, y0~y1, z0~z1) → Unity 월드 Bounds  (Unity.x = -gz.y, Unity.y = gz.z, Unity.z = gz.x)
        static Bounds GzBox(float x0, float x1, float y0, float y1, float z0, float z1)
        {
            var min = new Vector3(-y1, z0, x0); var max = new Vector3(-y0, z1, x1);
            var b = new Bounds(); b.SetMinMax(min, max); return b;
        }

        // 렌더 바운드가 월드 박스 안에 완전히 들어가는 자식 오브젝트를 비활성화 (건물 전체 쉘·바닥·지붕은 박스보다 커서 제외됨)
        static void DisableRenderersInside(GameObject root, Bounds worldBox)
        {
            foreach (var r in root.GetComponentsInChildren<Renderer>(true))
            {
                var b = r.bounds;
                if (worldBox.Contains(b.min) && worldBox.Contains(b.max))
                { r.gameObject.SetActive(false); Debug.Log($"[PlantDT] disabled {r.name} (bounds {b.size})"); }
            }
        }

        // 모든 자식 메시에서, 월드 좌표 박스 안에 중심이 있는 삼각형을 제거한 메시 사본을 만들어 교체
        static void CutTriangles(GameObject root, Bounds worldBox)
        {
            Directory.CreateDirectory("Assets/PlantDT/Meshes");
            foreach (var mf in root.GetComponentsInChildren<MeshFilter>(true))
            {
                if (mf.sharedMesh == null || !mf.gameObject.activeInHierarchy) continue;
                var rb = mf.GetComponent<Renderer>(); if (rb == null || !rb.bounds.Intersects(worldBox)) continue;
                string childName = mf.name;
                var src = mf.sharedMesh; var verts = src.vertices; var l2w = mf.transform.localToWorldMatrix;
                var mesh = Object.Instantiate(src); mesh.name = $"{src.name}_cut"; int removed = 0;
                for (int sm = 0; sm < src.subMeshCount; sm++)
                {
                    var tris = src.GetTriangles(sm); var keep = new System.Collections.Generic.List<int>(tris.Length);
                    for (int i = 0; i < tris.Length; i += 3)
                    {
                        var c = (l2w.MultiplyPoint3x4(verts[tris[i]]) + l2w.MultiplyPoint3x4(verts[tris[i + 1]]) + l2w.MultiplyPoint3x4(verts[tris[i + 2]])) / 3f;
                        if (worldBox.Contains(c)) { removed++; continue; }
                        keep.Add(tris[i]); keep.Add(tris[i + 1]); keep.Add(tris[i + 2]);
                    }
                    mesh.SetTriangles(keep, sm);
                }
                if (removed == 0) { Object.DestroyImmediate(mesh); continue; }
                var path = AssetDatabase.GenerateUniqueAssetPath($"Assets/PlantDT/Meshes/{src.name}_cut.asset");
                AssetDatabase.CreateAsset(mesh, path);
                mf.sharedMesh = mesh;
                Debug.Log($"[PlantDT] cut {childName}/{src.name}: {removed} triangles removed → {path}");
            }
        }

        // Gazebo 회전(roll,pitch,yaw) → Unity 회전. 좌표 변환 (x,y,z)→(-y,z,x) 는 반사(det −1)를 포함하므로
        // 쿼터니언은 축을 변환하고 각도 부호를 뒤집는다: (qx,qy,qz,qw) → (qy, −qz, −qx, qw)
        static Quaternion GzRot(float roll, float pitch, float yaw)
        {
            float cr = Mathf.Cos(roll / 2), sr = Mathf.Sin(roll / 2), cp = Mathf.Cos(pitch / 2), sp = Mathf.Sin(pitch / 2), cy = Mathf.Cos(yaw / 2), sy = Mathf.Sin(yaw / 2);
            float qw = cr * cp * cy + sr * sp * sy, qx = sr * cp * cy - cr * sp * sy, qy = cr * sp * cy + sr * cp * sy, qz = cr * cp * sy - sr * sp * cy;
            return new Quaternion(qy, -qz, -qx, qw);
        }

        static float[] Nums(string text) => text.Trim().Split(new[] { ' ', '\t', '\n' }, System.StringSplitOptions.RemoveEmptyEntries).Select(v => float.Parse(v, CultureInfo.InvariantCulture)).ToArray();

        // plant_world.sdf 의 <model> 중 원시 도형(box/cylinder) visual 을 그대로 Unity 프리미티브로 만든다.
        // 건물·에셋으로 대체된 모델(plant_building, gas_tank, 팔레트, 게이지)은 제외.
        static void BuildGazeboObstacles()
        {
            var sdfPath = Path.GetFullPath(Path.Combine(Application.dataPath, "../../../simulation/worlds/plant_world.sdf"));
            if (!File.Exists(sdfPath)) { Debug.LogWarning($"[PlantDT] SDF not found: {sdfPath}"); return; }
            var skip = new HashSet<string> { "ground", "plant_building", "gas_tank", "factory_pallet_1", "factory_pallet_2", "gauge_panel", "gauge_needle" };
            var doc = new XmlDocument(); doc.Load(sdfPath);
            var root = new GameObject("GazeboObstacles");
            var matCache = new Dictionary<string, Material>();
            int count = 0;
            foreach (XmlNode model in doc.SelectNodes("//world/model"))
            {
                string mname = model.Attributes["name"].Value; if (skip.Contains(mname)) continue;
                var mp = Nums(model.SelectSingleNode("pose")?.InnerText ?? "0 0 0 0 0 0");
                var mPos = new Vector3(mp[0], mp[1], mp[2]); var mRot = GzRot(mp[3], mp[4], mp[5]);
                var mgo = new GameObject(mname); mgo.transform.SetParent(root.transform, false);
                foreach (XmlNode vis in model.SelectNodes(".//visual"))
                {
                    var geom = vis.SelectSingleNode("geometry"); if (geom == null) continue;
                    var vp = Nums(vis.SelectSingleNode("pose")?.InnerText ?? "0 0 0 0 0 0");
                    var vPosGz = new Vector3(vp[0], vp[1], vp[2]); var vRot = GzRot(vp[3], vp[4], vp[5]);
                    // 월드 포즈 = 모델 포즈 ∘ visual 포즈 (Gazebo 좌표에서 합성 후 변환)
                    // 위치: R_model(gz) 을 Unity 로 옮겨 적용해도 동일하므로 Unity 공간에서 합성
                    var worldPos = Gz(mPos.x, mPos.y, mPos.z) + mRot * Gz(vPosGz.x, vPosGz.y, vPosGz.z);
                    var worldRot = mRot * vRot;
                    GameObject go; var box = geom.SelectSingleNode("box/size"); var cyl = geom.SelectSingleNode("cylinder");
                    if (box != null)
                    {
                        var sz = Nums(box.InnerText); go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        go.transform.localScale = new Vector3(sz[1], sz[2], sz[0]);            // gz (x,y,z) 크기 → Unity (y,z,x)
                    }
                    else if (cyl != null)
                    {
                        float r = float.Parse(cyl.SelectSingleNode("radius").InnerText, CultureInfo.InvariantCulture);
                        float l = float.Parse(cyl.SelectSingleNode("length").InnerText, CultureInfo.InvariantCulture);
                        go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);                 // Unity 실린더: 높이 2(y축), 반지름 0.5
                        go.transform.localScale = new Vector3(2 * r, l / 2f, 2 * r);           // gz 실린더 축 z → Unity y
                    }
                    else continue;
                    Object.DestroyImmediate(go.GetComponent<Collider>());
                    go.name = vis.Attributes["name"]?.Value ?? "visual"; go.transform.SetParent(mgo.transform, false);
                    go.transform.position = worldPos; go.transform.rotation = worldRot;
                    var diff = vis.SelectSingleNode("material/diffuse")?.InnerText ?? "0.6 0.6 0.6 1"; var c = Nums(diff);
                    string key = $"{c[0]:F2}_{c[1]:F2}_{c[2]:F2}";
                    if (!matCache.TryGetValue(key, out var mat))
                    {
                        mat = new Material(Shader.Find("HDRP/Lit")) { name = $"Gz_{key}" }; mat.SetColor("_BaseColor", new Color(c[0], c[1], c[2], 1f)); mat.SetFloat("_Smoothness", 0.3f);
                        Directory.CreateDirectory("Assets/PlantDT/Materials"); AssetDatabase.CreateAsset(mat, $"Assets/PlantDT/Materials/Gz_{key}.mat"); matCache[key] = mat;
                    }
                    go.GetComponent<Renderer>().sharedMaterial = mat; count++;
                }
            }
            Debug.Log($"[PlantDT] Gazebo obstacles: {count} visuals from SDF");
        }

        static void BuildGaugePanel()
        {
            // 눈금판 텍스처: 리포의 simulation/models/gauge/dial.png (make_dial.py 생성) 을 프로젝트로 복사
            Directory.CreateDirectory("Assets/PlantDT/Textures");
            var src = Path.GetFullPath(Path.Combine(Application.dataPath, "../../../simulation/models/gauge/dial.png"));
            const string dst = "Assets/PlantDT/Textures/dial.png";
            if (File.Exists(src)) { File.Copy(src, dst, true); AssetDatabase.ImportAsset(dst); }
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(dst);
            var panel = GameObject.CreatePrimitive(PrimitiveType.Quad); panel.name = "GaugePanel";
            Object.DestroyImmediate(panel.GetComponent<Collider>());
            panel.transform.position = Gz(38f, 1.58f, 0.6f);                 // 북벽(y=+1.75) 안쪽에 살짝 띄움
            panel.transform.rotation = Quaternion.LookRotation(Gz(0, 1, 0), Vector3.up);   // 면이 복도 중앙(-y)을 향하도록
            panel.transform.localScale = new Vector3(0.5f, 0.5f, 1f);
            var mat = new Material(Shader.Find("HDRP/Lit")); mat.name = "GaugeDial_Mat";
            if (tex != null) mat.SetTexture("_BaseColorMap", tex);
            mat.SetColor("_BaseColor", Color.white); mat.SetFloat("_Smoothness", 0.2f);
            AssetDatabase.CreateAsset(mat, "Assets/PlantDT/GaugeDial_Mat.mat");
            panel.GetComponent<Renderer>().sharedMaterial = mat;
            // 뒤판(벽걸이 케이스)
            var back = GameObject.CreatePrimitive(PrimitiveType.Cube); back.name = "GaugeCase"; back.transform.SetParent(panel.transform, false);
            Object.DestroyImmediate(back.GetComponent<Collider>());
            back.transform.localPosition = new Vector3(0, 0, 0.03f); back.transform.localScale = new Vector3(1.1f, 1.1f, 0.06f);
            var bm = new Material(Shader.Find("HDRP/Lit")); bm.SetColor("_BaseColor", new Color(0.15f, 0.15f, 0.17f)); bm.name = "GaugeCase_Mat";
            AssetDatabase.CreateAsset(bm, "Assets/PlantDT/GaugeCase_Mat.mat"); back.GetComponent<Renderer>().sharedMaterial = bm;
            Debug.Log($"[PlantDT] gauge panel at gz(38,1.58,0.6), dial texture={(tex != null)}");
        }

        // 에셋 gas_spurt 프리팹은 내장 기본 파티클 재질(HDRP 미지원 → 렌더 안 됨)을 쓰고 이미터가 4.35m 오프셋에 있다.
        // HDRP 투명 Unlit 재질 + 부드러운 원 텍스처로 초록 가스 제트(영상과 같은 연출)로 바꾸고 이미터를 원점(높이 0.9m)에 둔다.
        static void FixGasEffect(GameObject gas)
        {
            var ps = gas.GetComponentInChildren<ParticleSystem>(true); if (ps == null) { Debug.LogWarning("[PlantDT] gas effect: no ParticleSystem"); return; }
            ps.transform.localPosition = new Vector3(0, 0.9f, 0); ps.transform.localRotation = Quaternion.Euler(-15f, 0, 0);   // 살짝 위로
            gas.transform.rotation = Quaternion.LookRotation(Gz(1, 0, 0), Vector3.up);                                    // 탱크(-x) → 공장 중앙(+x) 방향 분사
            var main = ps.main; main.playOnAwake = true; main.loop = true; main.simulationSpace = ParticleSystemSimulationSpace.World;
            main.startSize = new ParticleSystem.MinMaxCurve(0.25f, 0.5f); main.startLifetime = new ParticleSystem.MinMaxCurve(1.0f, 1.6f);
            main.startSpeed = new ParticleSystem.MinMaxCurve(3f, 5f); main.startColor = new Color(0.4f, 1f, 0.5f, 0.5f); main.maxParticles = 800;
            var em = ps.emission; em.enabled = true; em.rateOverTime = 150f;
            var shape = ps.shape; shape.enabled = true; shape.shapeType = ParticleSystemShapeType.Cone; shape.angle = 14f; shape.radius = 0.04f;
            var sol = ps.sizeOverLifetime; sol.enabled = true; sol.size = new ParticleSystem.MinMaxCurve(1f, AnimationCurve.Linear(0, 0.4f, 1, 2.0f));
            var col = ps.colorOverLifetime; col.enabled = true; var g = new Gradient();
            g.SetKeys(new[] { new GradientColorKey(new Color(0.4f, 1f, 0.5f), 0), new GradientColorKey(new Color(0.7f, 1f, 0.8f), 1) },
                      new[] { new GradientAlphaKey(0.8f, 0), new GradientAlphaKey(0f, 1) });
            col.color = g;
            // 부드러운 원 텍스처 (절차 생성)
            const int N = 64; var tex = new Texture2D(N, N, TextureFormat.RGBA32, false) { name = "soft_particle" };
            for (int y = 0; y < N; y++) for (int x = 0; x < N; x++)
            { float d = Vector2.Distance(new Vector2(x, y), new Vector2(N / 2f, N / 2f)) / (N / 2f); float a = Mathf.Clamp01(1f - d); a = a * a; tex.SetPixel(x, y, new Color(1, 1, 1, a)); }
            tex.Apply(); Directory.CreateDirectory("Assets/PlantDT/Textures"); AssetDatabase.CreateAsset(tex, "Assets/PlantDT/Textures/soft_particle.asset");
            var mat = new Material(Shader.Find("HDRP/Unlit")) { name = "GasParticle_Mat" };
            mat.SetTexture("_UnlitColorMap", tex); mat.SetColor("_UnlitColor", new Color(0.45f, 1f, 0.55f, 0.35f));
            HDMaterial.SetSurfaceType(mat, true); HDMaterial.SetAlphaClipping(mat, false); HDMaterial.ValidateMaterial(mat);
            AssetDatabase.CreateAsset(mat, "Assets/PlantDT/GasParticle_Mat.mat");
            var r = ps.GetComponent<ParticleSystemRenderer>(); r.sharedMaterial = mat; r.renderMode = ParticleSystemRenderMode.Billboard;
            Debug.Log("[PlantDT] gas effect: HDRP particle material applied");
        }

        static int Dominant(Vector3 v) { var a = new[] { Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z) }; return a[0] >= a[1] && a[0] >= a[2] ? 0 : (a[1] >= a[2] ? 1 : 2); }

        static Bounds WorldBounds(GameObject go)
        {
            var rs = go.GetComponentsInChildren<Renderer>(true);
            var b = rs[0].bounds; foreach (var r in rs) b.Encapsulate(r.bounds); return b;
        }

        // 프리팹의 긴 수평축을 월드 +z(Gazebo +x) 로 돌리고, 바운드가 +z 쪽으로 치우치도록(문·근단 방향) 180° 보정
        static void OrientLongAxisToZ(GameObject go)
        {
            if (go == null) return;
            var baseRot = go.transform.rotation;                 // 프리팹 루트의 원래 회전(모델을 세우는 값) 보존
            go.transform.position = Vector3.zero;
            var raw = WorldBounds(go);
            Debug.Log($"[PlantDT] {go.name}: raw bounds min={raw.min} max={raw.max} baseRot={baseRot.eulerAngles}");
            if (raw.size.x > raw.size.z) go.transform.rotation = Quaternion.Euler(0, -90f, 0) * baseRot;   // 긴 변 → 월드 z
            var b = WorldBounds(go);
            if (b.max.z < -b.min.z) go.transform.rotation = Quaternion.Euler(0, 180f, 0) * go.transform.rotation;   // 치우침이 -z 면 뒤집기
            b = WorldBounds(go);
            Debug.Log($"[PlantDT] {go.name}: oriented rot={go.transform.eulerAngles} bounds z[{b.min.z:F2},{b.max.z:F2}] x[{b.min.x:F2},{b.max.x:F2}]");
        }

        // 렌더 바운드의 바닥 중심을 Gazebo 좌표 centerGz(z=0 바닥) 에 맞추고, 필요시 xy 크기를 sizeGz 에 맞춰 스케일
        static void AlignBounds(GameObject go, Vector3 centerGz, Vector3 sizeGz, bool scaleToFit, float uniformScale = 1f)
        {
            if (go == null) return;
            go.transform.position = Vector3.zero;   // 회전은 프리팹/OrientLongAxisToZ 값을 유지
            go.transform.localScale = go.transform.localScale * uniformScale;
            var rs = go.GetComponentsInChildren<Renderer>(true);
            if (rs.Length == 0) { go.transform.position = Gz(centerGz.x, centerGz.y, centerGz.z); return; }
            var b = WorldBounds(go);
            if (scaleToFit && sizeGz.x > 0 && sizeGz.y > 0)
            {
                // 월드 z 크기 → sizeGz.x(Gazebo x), 월드 x 크기 → sizeGz.y(Gazebo y). 회전돼 있으므로 로컬축으로 환산
                float fz = sizeGz.x / b.size.z, fx = sizeGz.y / b.size.x;
                var s = go.transform.localScale;
                s[Dominant(go.transform.InverseTransformDirection(Vector3.forward))] *= fz;   // 월드 z ↔ 로컬축
                s[Dominant(go.transform.InverseTransformDirection(Vector3.right))] *= fx;     // 월드 x ↔ 로컬축
                go.transform.localScale = s;
                b = WorldBounds(go);
            }
            var bottomCenter = new Vector3(b.center.x, b.min.y, b.center.z);
            go.transform.position += Gz(centerGz.x, centerGz.y, centerGz.z) - bottomCenter;
            Debug.Log($"[PlantDT] {go.name}: bounds size={b.size} → placed at gz({centerGz.x},{centerGz.y})");
        }
    }
}
