import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 현재 작업 디렉토리 (PathPlanner 폴더에서 실행한다고 가정)
    current_dir = os.getcwd()
    
    # 기본 파일 경로 설정
    default_map_yaml = os.path.join(current_dir, 'maps/my_map.yaml')
    # 경로 JSON은 매번 이름이 바뀌므로, 실행할 때 인자로 받거나 가장 최근 것을 넣어야 합니다.
    # 여기서는 기본값을 비워두고 실행 시 입력받도록 설정했습니다.
    
    return LaunchDescription([
        # 1. 실행 인수 설정 (map, path)
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Path to the map YAML file'
        ),
        DeclareLaunchArgument(
            'path',
            description='Path to the path JSON file (Required)'
        ),

        # 2. 지도 퍼블리셔 실행 (pub_map.py)
        ExecuteProcess(
            cmd=['python3', 'apps/pub_map.py', LaunchConfiguration('map')],
            cwd=current_dir,
            output='screen'
        ),

        # 3. 경로 퍼블리셔 실행 (pub_path.py)
        ExecuteProcess(
            cmd=['python3', 'apps/pub_path.py', LaunchConfiguration('path')],
            cwd=current_dir,
            output='screen'
        ),

        # 4. RViz2 실행
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/ubuntu/.rviz2/path.rviz'],
            output='screen'
        )
    ])
