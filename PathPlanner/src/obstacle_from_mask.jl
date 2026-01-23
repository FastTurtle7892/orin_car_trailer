#
# Get obstacle map(ox, oy) from mask.png
# white=drivable, black=forbidden
#
# author: siris-Kang
#

module ObstacleFromMask

using FileIO
using ImageIO
using Images
using ImageMorphology
using JSON3

export MapMeta, load_ros_map_yaml_meta, build_oxoy_from_virtual_mask
export save_bool_image, save_oxoy_json


"""맵 메타(ROS map.yaml 기준)
resolution: meters per pixel
origin_x, origin_y: world 좌표에서 (pixel 0,0)이 놓이는 기준

주의: ROS map 서버에서 쓰는 일반적인 매핑은
  world_x = origin_x + px * resolution
  world_y = origin_y + (H-1-py) * resolution
즉 이미지 y축(위->아래)을 world y축(아래->위)로 뒤집는 경우가 많음.
"""
Base.@kwdef struct MapMeta
    resolution::Float64
    origin_x::Float64
    origin_y::Float64
    flip_y::Bool = true
end


"""ROS map_server 스타일 YAML 파일에서 resolution/origin 파싱.

  resolution: 0.05
  origin: [-10.0, -5.0, 0.0]

theta(회전)은 일단 0이라고 가정. 0이 아닌 경우는 별도 처리 필요.
"""
function load_ros_map_yaml_meta(yaml_path::AbstractString)
    res = nothing
    org = nothing
    open(yaml_path, "r") do io
        for line in eachline(io)
            s = strip(line)
            isempty(s) && continue
            startswith(s, "#") && continue

            if startswith(s, "resolution:")
                v = strip(split(s, ":", limit=2)[2])
                res = parse(Float64, v)
            elseif startswith(s, "origin:")
                v = strip(split(s, ":", limit=2)[2])
                v = replace(v, "[" => "", "]" => "")
                parts = split(v, ",")
                if length(parts) >= 2
                    ox = parse(Float64, strip(parts[1]))
                    oy = parse(Float64, strip(parts[2]))
                    org = (ox, oy)
                end
            end
        end
    end

    res === nothing && error("resolution not found in yaml: $yaml_path")
    org === nothing && error("origin not found in yaml: $yaml_path")

    return MapMeta(resolution=res, origin_x=org[1], origin_y=org[2], flip_y=true)
end


# Bool grid 저장 (true=1 white, false=0 black)
function save_bool_image(path::AbstractString, grid::AbstractMatrix{Bool})
    img = Gray.(Float32.(grid))
    save(path, img)
end


# ox/oy를 JSON으로 저장
function save_oxoy_json(path::AbstractString, ox::Vector{Float64}, oy::Vector{Float64}; extra=Dict())
    @assert length(ox) == length(oy)

    d = Dict{String, Any}("ox" => ox, "oy" => oy)

    for (k, v) in extra
        d[string(k)] = v
    end

    open(path, "w") do io
        write(io, JSON3.write(d))
    end
end


function disk_se(r::Int)
    r <= 0 && return reshape(Bool[true], 1, 1)
    sz = 2r + 1
    se = falses(sz, sz)
    cx = r + 1
    cy = r + 1
    for j in 1:sz, i in 1:sz
        dx = i - cx
        dy = j - cy
        se[j,i] = (dx*dx + dy*dy) <= r*r
    end
    return se
end


# mask.png를 읽어 forbidden Bool grid 생성
function load_mask_bool(mask_png::AbstractString; thresh::Float64=0.5)
    img = load(mask_png)
    g = Gray.(img)

    a = Float32.(channelview(g))
    grid = (ndims(a) == 3) ? dropdims(a; dims=1) : a   # (H,W)로 정규화

    forbidden = grid .< Float32(thresh)  # black=0 -> forbidden(true)
    return forbidden
end


# forbidden grid에서 경계선만 추출
function extract_boundary(forbidden::AbstractMatrix{Bool})
    H, W = size(forbidden)
    boundary = falses(H, W)
    @inbounds for y in 2:H-1
        for x in 2:W-1
            if forbidden[y,x]
                if (!forbidden[y-1,x]) || (!forbidden[y+1,x]) || (!forbidden[y,x-1]) || (!forbidden[y,x+1])
                    boundary[y,x] = true
                end
            end
        end
    end
    return boundary
end


# boundary를 r_cells만큼 inflation
function inflate_boundary(boundary::AbstractMatrix{Bool}, r_cells::Int)
    se = disk_se(r_cells)
    inflated = dilate(boundary, se)
    return inflated
end


# inflated grid의 true 셀을 world 좌표 점(ox/oy)로 변환
function grid_to_oxoy(inflated::AbstractMatrix{Bool}, meta::MapMeta;
                      downsample_m::Float64=0.10)
    H, W = size(inflated)
    step_cells = max(1, Int(ceil(downsample_m / meta.resolution)))
    ox = Float64[]
    oy = Float64[]
    @inbounds for y in 1:step_cells:H
        for x in 1:step_cells:W
            if inflated[y,x]
                wx = meta.origin_x + (x-1) * meta.resolution
                wy_px = meta.flip_y ? (H - y) : (y-1)
                wy = meta.origin_y + wy_px * meta.resolution
                push!(ox, wx)
                push!(oy, wy)
            end
        end
    end
    return ox, oy, step_cells
end


function build_oxoy_from_virtual_mask(mask_png::AbstractString, meta::MapMeta;
                                     thresh::Float64=0.5,
                                     inflation_radius_m::Float64=0.7,
                                     downsample_m::Float64=0.10,
                                     save_debug_prefix::AbstractString="")
    forbidden = load_mask_bool(mask_png; thresh=thresh)
    boundary = extract_boundary(forbidden)
    r_cells = Int(ceil(inflation_radius_m / meta.resolution))
    inflated = inflate_boundary(boundary, r_cells)
    ox, oy, step_cells = grid_to_oxoy(inflated, meta; downsample_m=downsample_m)

    if !isempty(save_debug_prefix)
        mkpath(dirname(save_debug_prefix))
        save_bool_image(save_debug_prefix * "_inflated.png", inflated)
    end

    dbg = Dict(
        "H" => size(forbidden,1),
        "W" => size(forbidden,2),
        "r_cells" => r_cells,
        "step_cells" => step_cells,
        "n_points" => length(ox)
    )
    return ox, oy, dbg
end

end