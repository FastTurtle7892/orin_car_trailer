#!/usr/bin/env python3
import matplotlib.pyplot as plt
import json
import os
import glob
import math

# ==========================================
# 1. 설정 (사용자 환경 맞춤)
# ==========================================
# JSON 파일들이 있는 폴더 (현재 홈 디렉토리 기준)
PATHS_DIR = os.path.expanduser("~/trailer_paths")

# 배경 장애물 파일 경로 (있다면 로드, 없으면 무시)
MAP_FILE = os.path.expanduser("~/orin_car_trailer/PathPlanner/maps/my_map_oxoy.json")

# 주요 노드 좌표 (스크립트 내장 - 별도 파일 불필요)
NODES = {
    "ORIGIN_GOAL":   (-1.111, 0.201, -1.57),
    "NODE_1":        (-1.111, -0.707, -1.57),
    "NODE_2":        (-1.111, -1.615, -1.57),
    "NODE_3":        (1.15,   -1.615, 1.57),
    "NODE_3_WAYPOINT": (-0.195, -2.39, 0.0)
}

# ==========================================
# 2. 유틸리티 함수
# ==========================================
def load_json(path):
    if not os.path.exists(path): return None
    try:
        with open(path, 'r') as f: return json.load(f)
    except: return None

def plot_arrow(x, y, yaw, length=0.1, width=0.05, fc="r", ec="k"):
    """ 화살표 그리기 """
    if not isinstance(x, float): return
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_width=width, head_length=width, fc=fc, ec=ec)

# ==========================================
# 3. 메인 실행
# ==========================================
def main():
    print(f"📂 Loading paths from: {PATHS_DIR}")
    
    # 1. 캔버스 설정
    plt.figure(figsize=(10, 8))
    plt.title(f"Nav2 JSON Paths Viewer ({PATHS_DIR})")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.5)

    # 2. 배경 지도(장애물) 그리기
    map_data = load_json(MAP_FILE)
    if map_data:
        ox, oy = [], []
        # 리스트 형태 체크
        if isinstance(map_data, dict):
            if "ox" in map_data and "oy" in map_data:
                ox, oy = map_data["ox"], map_data["oy"]
            elif "points" in map_data:
                ox = [p["x"] for p in map_data["points"]]
                oy = [p["y"] for p in map_data["points"]]
        
        if ox and oy:
            plt.scatter(ox, oy, c='black', s=1, marker='.', label="Obstacles")
            print(f"✅ Map loaded: {len(ox)} points")
    else:
        print("⚠️ Map file not found (Skipping background)")

    # 3. 노드(지점) 그리기
    print(f"📍 Plotting {len(NODES)} nodes...")
    for name, (x, y, yaw) in NODES.items():
        plt.scatter(x, y, s=100, c='blue', marker='X', zorder=5)
        plt.text(x + 0.1, y + 0.1, name, fontsize=9, fontweight='bold', color='blue')
        plot_arrow(x, y, yaw, length=0.2, width=0.05, fc='blue', ec='blue')

    # 4. JSON 경로 파일 로드 및 그리기
    json_files = sorted(glob.glob(os.path.join(PATHS_DIR, "*.json")))
    if not json_files:
        print("❌ No .json files found in the directory!")
        return

    colors = ['r', 'g', 'm', 'c', 'y', 'orange'] # 경로별 색상
    
    for i, filepath in enumerate(json_files):
        filename = os.path.basename(filepath)
        
        # 'all_nodes.json' 등 경로 파일이 아닌 것은 건너뛰기
        if "nodes" in filename or "oxoy" in filename:
            continue

        data = load_json(filepath)
        if not data: continue

        xs = data.get("x", [])
        ys = data.get("y", [])
        
        if not xs or not ys:
            continue

        color = colors[i % len(colors)]
        label = filename.replace("path_", "").replace(".json", "").replace("_nav2", "")
        
        # 경로 선 그리기
        plt.plot(xs, ys, color=color, linewidth=2, label=label, alpha=0.8)
        
        # 시작/끝점 표시
        plt.scatter(xs[0], ys[0], c=color, marker='o', s=30) # Start
        plt.scatter(xs[-1], ys[-1], c=color, marker='s', s=30) # End
        
        print(f"   -> Plotted: {filename} ({len(xs)} pts)")

    # 범례 및 출력
    plt.legend(loc='best', fontsize='small')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    
    print("🚀 Showing plot...")
    plt.show()

if __name__ == "__main__":
    main()
