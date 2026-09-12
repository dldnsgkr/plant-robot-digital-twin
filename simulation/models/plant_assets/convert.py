#!/usr/bin/env python3
"""과제 제공 Unity 에셋(.unitypackage) → Gazebo용 OBJ 변환 (sim 컨테이너 안에서 실행)

  사용: python3 convert.py <pkg_dir> <out_dir>
    pkg_dir : unitypackage 들을 각각 tar xzf 로 푼 디렉토리들의 상위 (guid 폴더/asset, pathname, asset.meta)
    out_dir : meshes/  (모델별 하위 폴더에 OBJ + MTL + 텍스처)

  1) assimp export : FBX → glb (임베디드 텍스처 추출)
  2) trimesh       : glb(Y-up, cm) → OBJ(Z-up, m). Gazebo OBJ 로더는 좌표를 그대로 쓰므로
                     여기서 축·단위를 확정한다 (glb 는 로더의 Y-up 회전이 파일마다 달라 불채택)
  3) MTL 보정      : Unity .mat 의 텍스처 GUID 를 따라가 재질명↔텍스처를 매핑하고,
                     Ka/Kd 를 1 로 올려 텍스처가 원색으로 보이게 함 (기본값은 0.4 라 어둡게 렌더됨)
"""
import re, shutil, subprocess, sys, pathlib
import numpy as np, trimesh

pkg_dir, out_dir = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
out_dir.mkdir(parents=True, exist_ok=True)
SKIP = {"SpotHPRIG_Rigging_02"}   # 97만 삼각형 리깅 로봇 — Gazebo 에는 과함 (Go2 URDF 사용)

# 기존 박스 월드(plant_world.sdf)의 검증된 배치·경로와 충돌하는 부분만 제거한다.
#  - Factory_03 : 동측 스트립(x 9.8~15 로컬)의 칸막이벽·펜스·구조물 → 계단/드럼통/복도 진입로와 겹침
#  - Corridor   : 바닥 위 6cm 플레이트(_12) → 라이다/지형맵에 가짜 단차로 잡힘
EXCLUDE = {"Factory_03": {"FC_Fence_2", "FC_Fence_01_3"},
           "Corridor_Wall_02": {"Corridor_Wall_01_12"}}
# (모델명, 서브메시명): [면 절단 규칙, ...] — 면 중심이 박스 안(normal_axis 지정 시 그 축 법선 면만) 이면 삭제
FACE_CUT = {
    # 외벽쉘 속 내부 칸막이벽(로컬 x≈9.7, 월드 x≈7.3) — 동측 스트립을 별도 방으로 나누는 벽
    ("Factory_03", "FC_Fence_01"): [dict(xmin=9.5, xmax=10.2, ymin=-8.8, ymax=8.8, zmax=7.5)],
    # 남벽 앞 바닥 설비(월드 x -7.5~-6, y -8.5~-7.5) — 공장기계(-9,-6) 점검 접근로와 겹침
    ("Factory_03", "FC_Fence_01_6"): [dict(xmin=-5.2, xmax=-3.3, ymin=-8.85, ymax=-7.0, zmax=3.0)],
}

# ---- 1. unitypackage 인덱스: guid → (pathname, asset 파일) ----
by_guid, by_name = {}, {}
for pn in pkg_dir.rglob("pathname"):
    d = pn.parent
    path = pn.read_text().splitlines()[0].strip()
    meta = d / "asset.meta"
    guid = None
    if meta.exists():
        m = re.search(r"^guid:\s*([0-9a-f]+)", meta.read_text(), re.M)
        guid = m.group(1) if m else None
    rec = {"path": path, "asset": d / "asset", "guid": guid}
    if guid: by_guid[guid] = rec
    by_name.setdefault(pathlib.Path(path).name, rec)

def unity_base_texture(mat_name):
    """Unity .mat(YAML) 에서 베이스 컬러 텍스처 파일 경로를 찾는다."""
    rec = by_name.get(f"{mat_name}.mat")
    if not rec or not rec["asset"].exists(): return None
    txt = rec["asset"].read_text(errors="ignore")
    for key in ("_BaseColorMap", "_BaseMap", "_MainTex"):
        m = re.search(rf"{key}:\s*\n\s*m_Texture:\s*\{{fileID:\s*\d+,\s*guid:\s*([0-9a-f]+)", txt)
        if m and m.group(1) in by_guid:
            return by_guid[m.group(1)]["asset"], pathlib.Path(by_guid[m.group(1)]["path"]).name
    return None

MAX_TEX = 1024   # ogre2(센서 카메라) 소프트웨어 렌더링 텍스처 예산 초과 방지
def _downscale(img_path):
    from PIL import Image
    im = Image.open(img_path)
    if max(im.size) <= MAX_TEX: return
    im.thumbnail((MAX_TEX, MAX_TEX), Image.LANCZOS)
    im.convert("RGB").save(img_path, optimize=True)

# 벽·지붕처럼 한 면만 있는 건물 메시는 양면으로 만든다 (면을 뒤집어 복제).
# ogre2(센서 렌더러) 는 뒷면을 컬링해 카메라·라이다에서 벽이 사라지므로 필수.
DOUBLE_SIDED = {"Factory_03", "Corridor_Wall_02"}
def _double_side(scene):
    for gname in list(scene.geometry):
        g = scene.geometry[gname]
        inv = g.copy(); inv.invert()
        scene.geometry[gname] = trimesh.util.concatenate([g, inv])

# ---- 2. FBX 변환 ----
R = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]], dtype=float)  # Y-up → Z-up
S = np.diag([0.01,0.01,0.01,1.0])                                        # cm → m
fbx_list = [r for r in by_name.values() if r["path"].lower().endswith(".fbx")]
for rec in sorted(fbx_list, key=lambda r: r["path"]):
    name = pathlib.Path(rec["path"]).stem
    if name in SKIP: continue
    sub = out_dir / name; sub.mkdir(exist_ok=True)
    fbx = sub / f"{name}.fbx"; shutil.copy(rec["asset"], fbx)
    glb = sub / f"{name}.glb"
    subprocess.run(["assimp","export",str(fbx),str(glb)], check=True, capture_output=True)
    scene = trimesh.load(str(glb), force="scene")
    scene.apply_transform(S @ R)
    for gname in list(scene.geometry):
        if gname in EXCLUDE.get(name, set()):
            scene.delete_geometry(gname); continue
        for cut in FACE_CUT.get((name, gname), []):
            g = scene.geometry[gname]
            T = next((scene.graph[n][0] for n in scene.graph.nodes_geometry if scene.graph[n][1] == gname), np.eye(4))
            c = trimesh.transform_points(g.triangles_center, T)
            nrm = (T[:3,:3] @ g.face_normals.T).T
            nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12   # T 에 스케일이 있어 재정규화 필수
            kill = ((c[:,0] > cut["xmin"]) & (c[:,0] < cut["xmax"]) & (c[:,1] > cut["ymin"]) & (c[:,1] < cut["ymax"])
                    & (c[:,2] < cut.get("zmax", 1e9)))
            if "normal_axis" in cut: kill &= np.abs(nrm[:, cut["normal_axis"]]) > 0.9
            g.update_faces(~kill); g.remove_unreferenced_vertices()
            print(f"  [{name}/{gname}] 면 {int(kill.sum())}개 절단 {cut}")
    if name in DOUBLE_SIDED: _double_side(scene)
    b = scene.bounds
    scene.export(str(sub / f"{name}.obj"), include_texture=True, include_normals=True)  # vn 필수: 없으면 ogre2(센서)가 흰색으로 렌더
    for img in sub.glob("*.png"): _downscale(img)
    fbx.unlink(); glb.unlink()

    # ---- 3. MTL 보정 ----
    mtl = sub / "material.mtl"
    blocks = re.split(r"(?m)^(?=newmtl )", mtl.read_text())
    fixed, mapped = [], []
    for blk in blocks:
        if not blk.startswith("newmtl"): fixed.append(blk); continue
        mname = blk.split()[1]
        has_map = "map_Kd" in blk
        if not has_map:
            tex = unity_base_texture(mname)
            if tex:
                src, fname = tex
                shutil.copy(src, sub / fname); has_map = True
                _downscale(sub / fname)
                blk = blk.rstrip("\n") + f"\nmap_Kd {fname}\n"; mapped.append(f"{mname}->{fname}")
        # 텍스처가 있으면 원색(1.0), 없으면 밝은 회색 — Ka 를 Kd 와 같게 두어 그늘면도 보이게
        # ogre2(센서 카메라, PBR) 는 ogre1(GUI) 보다 밝게 나와 1.0 이면 벽이 순백으로 포화된다
        ka, kd = ("0.55 0.55 0.55", "0.75 0.75 0.75") if has_map else ("0.45 0.45 0.45", "0.65 0.65 0.65")
        blk = re.sub(r"(?m)^Ka .*$", f"Ka {ka}", blk)
        blk = re.sub(r"(?m)^Kd .*$", f"Kd {kd}", blk)
        blk = re.sub(r"(?m)^Ks .*$", "Ks 0.050 0.050 0.050", blk)
        if "\nKa " not in blk: blk = blk.rstrip("\n") + f"\nKa {ka}\n"
        fixed.append(blk)
    mtl.write_text("".join(fixed))
    print(f"{name:20s} meshes={len(scene.geometry):2d} faces={sum(len(g.faces) for g in scene.geometry.values()):7d} "
          f"x[{b[0][0]:6.2f},{b[1][0]:6.2f}] y[{b[0][1]:6.2f},{b[1][1]:6.2f}] z[{b[0][2]:6.2f},{b[1][2]:6.2f}]"
          + (f"  +tex: {', '.join(mapped)}" if mapped else ""))
