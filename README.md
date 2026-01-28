maps/my_nodes.json

# 경로생성 origin_goal -> node 1 (저장: maps/path1)
mkdir -p maps/path1
julia apps/path_planning.jl --nodes maps/my_nodes.json --oxoy maps/my_map_oxoy.json --out_dir maps/path1 --pairs list --pair "ORIGIN_GOAL,NODE_1_COORD"

# my_map.pgm에서 rviz2로 경로 확인 (PathPlanner 폴더에서 실행)
ros2 launch apps/viz.launch.py path:=maps/my_results/path_Start_to_Goal_20260127_191041.json