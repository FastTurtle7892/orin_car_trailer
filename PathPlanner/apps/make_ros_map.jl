# PathPlanner/apps/make_ros_map.jl
using Images
using FileIO
using JSON
using ImageMagick  # PGM 파일 읽기 위해 필요 (없으면 ] add ImageMagick)

function main()
    # ---------------------------------------------------------
    # [사용자 설정 구역] my_map.yaml 파일을 보고 맞춰주세요!
    # ---------------------------------------------------------
    map_path = "../maps/my_map.pgm"       # PGM 파일 경로
    save_path = "../maps/my_map_oxoy.json" # 저장할 JSON 경로
    
    resolution = 0.05        # yaml 파일의 resolution 값
    origin_x   = -1.87       # yaml 파일 origin의 첫 번째 값 [x]
    origin_y   = -3.88       # yaml 파일 origin의 두 번째 값 [y]
    
    occupied_thresh = 0.65   # 이 값보다 어두우면 장애물로 인식 (0~1 사이, 보통 0.65)
    # ---------------------------------------------------------

    println("Loading map: $map_path")
    img = load(map_path)
    
    height, width = size(img)
    println("Map Size: Width=$width, Height=$height")
    
    ox = Float64[]
    oy = Float64[]

    # 이미지 픽셀을 순회하며 장애물(검은색) 찾기
    for y in 1:height
        for x in 1:width
            # Gray scale 값 추출 (0.0=검정, 1.0=흰색)
            pixel_val = Gray(img[y, x]).val
            
            # 장애물(검은색)인 경우 좌표 변환 및 저장
            if pixel_val < (1.0 - occupied_thresh) # ROS는 보통 검은색(0)이 장애물
                
                # [좌표 변환 공식]
                # ROS 맵 이미지(PGM)는 보통 (0,0)이 Top-Left이지만, 
                # World 좌표계 변환 시 Bottom-Left가 기준이 되도록 Y축을 뒤집어야 함.
                
                # x 좌표: (픽셀 인덱스) * 해상도 + 원점x
                wx = (x - 1) * resolution + origin_x
                
                # y 좌표: (전체높이 - 픽셀 인덱스) * 해상도 + 원점y (Y축 반전)
                wy = (height - y) * resolution + origin_y
                
                push!(ox, wx)
                push!(oy, wy)
            end
        end
    end

    println("Obstacles found: $(length(ox)) points")

    # JSON 저장
    data = Dict("ox" => ox, "oy" => oy)
    open(save_path, "w") do f
        write(f, JSON.json(data))
    end
    println("Saved to: $save_path")
end

main()
