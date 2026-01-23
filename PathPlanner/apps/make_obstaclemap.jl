#
# generate obstacle.json from mask.png
#
# author: siris-Kang
#

"""make_obstaclemap.jl

mask.png (white=drivable, black=forbidden)로부터
 - boundary 추출
 - inflation
 - ox/oy 생성
 - 디버그 이미지/JSON 저장

julia apps/make_obstaclemap.jl --mask maps/node01/mask.png --map_yaml maps/my_map.yaml --out_dir maps/node01 --prefix node01
"""

include("../src/obstacle_from_mask.jl")
using .ObstacleFromMask


function parse_kv_args(args::Vector{String})
    d = Dict{String,String}()
    i = 1
    while i <= length(args)
        k = args[i]
        if startswith(k, "--")
            key = replace(k, "--" => "")
            if i == length(args)
                d[key] = "true"
                break
            end
            v = args[i+1]
            if startswith(v, "--")
                d[key] = "true"
                i += 1
                continue
            end
            d[key] = v
            i += 2
        else
            i += 1
        end
    end
    return d
end


function need(d::Dict{String,String}, k::String)
    haskey(d, k) || error("missing required arg --$k")
    return d[k]
end


function getf(d::Dict{String,String}, k::String, default::Float64)
    return haskey(d, k) ? parse(Float64, d[k]) : default
end

function gets(d::Dict{String,String}, k::String, default::String)
    return haskey(d, k) ? d[k] : default
end


function main()
    d = parse_kv_args(ARGS)

    mask = need(d, "mask")
    map_yaml = need(d, "map_yaml")
    out_dir = gets(d, "out_dir", "data/precomputed")
    prefix_name = gets(d, "prefix", "airport")
    inflation = getf(d, "inflation", 0.1)
    downsample = getf(d, "downsample", 0.10)
    thresh = getf(d, "thresh", 0.5)

    mkpath(out_dir)
    meta = load_ros_map_yaml_meta(map_yaml)

    prefix = joinpath(out_dir, prefix_name)
    ox, oy, dbg = build_oxoy_from_virtual_mask(
        mask, meta;
        thresh=thresh,
        inflation_radius_m=inflation,
        downsample_m=downsample,
        save_debug_prefix=prefix
    )

    out_json = prefix * "_oxoy.json"
    save_oxoy_json(out_json, ox, oy; extra=dbg)

    println("[ok] saved:")
    println("  oxoy:  ", out_json)
    println("  debug: ", prefix, "_inflated.png")
    println("  meta:  resolution=", meta.resolution, " origin=", (meta.origin_x, meta.origin_y), " flip_y=", meta.flip_y)
    println("  stats: ", dbg)
end

main()
