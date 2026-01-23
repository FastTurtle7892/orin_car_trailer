#
# generate Hybrid A* path
#
# author: siris-Kang
#

"""
필수:
  --nodes  nodes/nodeA.json
  --oxoy   data/oxoy.json   ({"ox":[...], "oy":[...]})

옵션:
  --out_dir ../maps/node01/paths
  --log_dir ../maps/node01/planner_logs
  --pairs   all | upper | list
  --pair    "A1,A2" (pairs=list일 때 반복 지정 가능)
  --xyreso  0.5
  --yawreso 0.2617993877991494
  --styaw_mode same|zero
  --gtyaw_mode same|zero

  julia apps/path_planning.jl --nodes maps/node01/node.json --oxoy maps/node01/node01_oxoy.json --pairs upper --out_dir maps/node01/paths --log_dir maps/node01/planner_logs
"""

using JSON3
using Dates
using Random
using NearestNeighbors
using Logging

include("../src/trailerlib.jl")
include("../src/rs_path.jl")
include("../src/grid_a_star.jl")
include("../src/trailer_hybrid_a_star.jl")


function parse_kv_args(args::Vector{String})
    d = Dict{String,Any}()
    i = 1
    while i <= length(args)
        a = args[i]
        if startswith(a, "--")
            key = replace(a, "--" => "")
            if i == length(args) || startswith(args[i+1], "--")
                d[key] = true
                i += 1
                continue
            end
            val = args[i+1]
            if haskey(d, key)
                if d[key] isa Vector
                    push!(d[key], val)
                else
                    d[key] = Any[d[key], val]
                end
            else
                d[key] = val
            end
            i += 2
        else
            i += 1
        end
    end
    return d
end

need(d, k) = (haskey(d, k) ? d[k] : error("missing required arg --$k"))
gets(d, k, default::String) = (haskey(d,k) ? String(d[k]) : default)
getf(d, k, default::Float64) = (haskey(d,k) ? parse(Float64, String(d[k])) : default)

# ----------------------------
# load helpers
# ----------------------------
struct NavNode
    id::String
    x::Float64
    y::Float64
    yaw::Float64
end

function load_nodes(path::AbstractString)
    data = JSON3.read(read(path, String))
    frame_id = String(get(data, :frame_id, "map"))
    nodes_raw = get(data, :nodes, nothing)
    nodes_raw === nothing && error("nodes json missing 'nodes' array")

    nodes = NavNode[]
    for n in nodes_raw
        push!(nodes, NavNode(String(n[:id]), Float64(n[:x]), Float64(n[:y]), Float64(get(n, :yaw, 0.0))))
    end

    # dup check
    seen = Set{String}()
    for n in nodes
        (n.id in seen) && error("duplicate node id: $(n.id)")
        push!(seen, n.id)
    end
    return frame_id, nodes
end

function node_map(nodes::Vector{NavNode})
    m = Dict{String,NavNode}()
    for n in nodes
        m[n.id] = n
    end
    return m
end

function load_oxoy_json(path::AbstractString)
    data = JSON3.read(read(path, String))
    ox = Float64.(collect(get(data, :ox, Float64[])))
    oy = Float64.(collect(get(data, :oy, Float64[])))
    return ox, oy
end

function parse_pair(s::AbstractString)
    parts = split(String(s), ',')
    length(parts) == 2 || error("--pair must be like A1,A2")
    return strip(parts[1]), strip(parts[2])
end

function make_pairs(nodes::Vector{NavNode}, mode::String, pair_args)
    ids = [n.id for n in nodes]
    pairs = Tuple{String,String}[]

    if mode == "all"
        for a in ids, b in ids
            a == b && continue
            push!(pairs, (a,b))
        end
    elseif mode == "upper"
        for i in 1:length(ids)-1
            for j in i+1:length(ids)
                push!(pairs, (ids[i], ids[j]))
            end
        end
    elseif mode == "list"
        pair_args === nothing && error("--pairs list requires --pair A1,A2 (repeatable)")
        if pair_args isa Vector
            for s in pair_args
                push!(pairs, parse_pair(s))
            end
        else
            push!(pairs, parse_pair(String(pair_args)))
        end
    else
        error("unknown --pairs mode: $mode (use all|upper|list)")
    end

    return pairs
end

function yaw_mode_value(mode::String, yaw::Float64)
    mode = lowercase(mode)
    if mode == "same"
        return yaw
    elseif mode == "zero"
        return 0.0
    else
        error("unknown yaw mode: $mode (use same|zero)")
    end
end

# direction이 Bool이든 ±1이든 안전하게 int로 변환
function dir_to_int(d)
    if d isa Bool
        return d ? 1 : -1
    else
        return Int(d)   # Int8(+1/-1) 같은 경우
    end
end

function path_to_dict(p::trailer_hybrid_a_star.Path)
    dir_int = [dir_to_int(d) for d in p.direction]
    return Dict(
        "ok" => true,
        "length" => length(p.x),
        "x" => collect(p.x),
        "y" => collect(p.y),
        "yaw" => collect(p.yaw),
        "yaw1" => collect(p.yaw1),
        "direction" => dir_int,
        "cost" => p.cost
    )
end


function fail_response(msg::String)
    return Dict("ok" => false, "error" => msg)
end

function save_plan_log(req_dict::Dict, resp_dict::Dict; base_dir="planner_logs")
    mkpath(base_dir)
    run_id = Dates.format(now(), "yyyymmdd_HHMMSS") * "_" * randstring(4)
    run_dir = joinpath(base_dir, run_id)
    mkpath(run_dir)

    open(joinpath(run_dir, "request.json"), "w") do io
        write(io, JSON3.write(req_dict))
    end
    open(joinpath(run_dir, "response.json"), "w") do io
        write(io, JSON3.write(resp_dict))
    end
    return run_dir
end

# main
function main()
    d = parse_kv_args(ARGS)

    nodes_path = String(need(d, "nodes"))
    oxoy_path  = String(need(d, "oxoy"))

    out_dir = gets(d, "out_dir", "paths")
    log_dir = gets(d, "log_dir", "planner_logs")

    pairs_mode = lowercase(gets(d, "pairs", "upper"))
    pair_args = get(d, "pair", nothing)

    xyreso  = getf(d, "xyreso", trailer_hybrid_a_star.XY_GRID_RESOLUTION)
    yawreso = getf(d, "yawreso", trailer_hybrid_a_star.YAW_GRID_RESOLUTION)

    styaw_mode = gets(d, "styaw_mode", "same")
    gtyaw_mode = gets(d, "gtyaw_mode", "same")

    mkpath(out_dir)
    mkpath(log_dir)

    frame_id, nodes = load_nodes(nodes_path)
    nmap = node_map(nodes)
    pairs = make_pairs(nodes, pairs_mode, pair_args)

    ox, oy = load_oxoy_json(oxoy_path)

    # KDTree: 장애물이 0개면 생성하면 에러나니까 조건부
    kdtree = (length(ox) > 0) ? KDTree([ox'; oy']) : nothing

    idx = Any[]
    ok_cnt = 0
    ts = Dates.format(now(), "yyyymmdd_HHMMSS")

    for (sid, gid) in pairs
        s = get(nmap, sid, nothing)
        g = get(nmap, gid, nothing)
        (s === nothing || g === nothing) && error("unknown node id in pair: $sid,$gid")

        sx, sy, syaw = s.x, s.y, s.yaw
        gx, gy, gyaw = g.x, g.y, g.yaw

        styaw = yaw_mode_value(styaw_mode, syaw)
        gtyaw = yaw_mode_value(gtyaw_mode, gyaw)

        # --- server와 동일한 collision precheck ---
        ok_start = trailerlib.check_trailer_collision(ox, oy, [sx], [sy], [syaw], [styaw])
        if !ok_start
            trailerlib.check_trailer_collision(ox, oy, [sx], [sy], [syaw], [styaw]; debug=true, max_near=12)
        end

        ok_goal = if kdtree === nothing
            trailerlib.check_trailer_collision(ox, oy, [gx], [gy], [gyaw], [gtyaw])
        else
            trailerlib.check_trailer_collision(ox, oy, [gx], [gy], [gyaw], [gtyaw], kdtree=kdtree)
        end
        if !ok_goal
            trailerlib.check_trailer_collision(ox, oy, [gx], [gy], [gyaw], [gtyaw]; debug=true, max_near=12)
        end

        @info "[COLL CHECK]" ok_start=ok_start ok_goal=ok_goal sx=sx sy=sy gx=gx gy=gy

        # --- plan ---
        p = trailer_hybrid_a_star.calc_hybrid_astar_path(
            sx, sy, syaw, styaw,
            gx, gy, gyaw, gtyaw,
            ox, oy,
            xyreso, yawreso
        )

        out = if p isa trailer_hybrid_a_star.Path
            dct = path_to_dict(p)
            dct["frame_id"] = frame_id
            dct["start_id"] = sid
            dct["goal_id"] = gid
            dct
        else
            fail_response("Cannot find path (No open set). Check start/goal/obstacles.")
        end

        # request snapshot + log 저장 (server와 동일)
        req_snapshot = Dict(
            "sx"=>sx, "sy"=>sy, "syaw"=>syaw, "styaw"=>styaw,
            "gx"=>gx, "gy"=>gy, "gyaw"=>gyaw, "gtyaw"=>gtyaw,
            "xyreso"=>xyreso, "yawreso"=>yawreso,
            "ox"=>ox, "oy"=>oy,
            "nodes_file"=>nodes_path,
            "oxoy_file"=>oxoy_path,
            "pair"=>string(sid, ",", gid)
        )
        run_dir = save_plan_log(req_snapshot, out; base_dir=log_dir)
        @info "saved planner log" run_dir=run_dir

        # per-path file save
        fname = "path_$(sid)_to_$(gid)_$(ts).json"
        fpath = joinpath(out_dir, fname)
        open(fpath, "w") do io
            write(io, JSON3.write(out))
        end

        push!(idx, Dict(
            "start_id"=>sid,
            "goal_id"=>gid,
            "ok"=>get(out, "ok", false),
            "file"=>fname,
            "n"=>get(out, "length", 0),
            "cost"=>get(out, "cost", nothing),
            "log_dir"=>basename(run_dir)
        ))

        if get(out, "ok", false)
            ok_cnt += 1
        end

        println("[plan] $sid -> $gid ok=$(get(out,"ok",false)) file=$fname")
    end

    index_path = joinpath(out_dir, "index_$(ts).json")
    open(index_path, "w") do io
        write(io, JSON3.write(Dict(
            "frame_id"=>frame_id,
            "nodes_file"=>nodes_path,
            "oxoy_file"=>oxoy_path,
            "xyreso"=>xyreso,
            "yawreso"=>yawreso,
            "pairs_mode"=>pairs_mode,
            "count"=>length(pairs),
            "ok_count"=>ok_cnt,
            "items"=>idx
        )))
    end

    println("[done] saved $ok_cnt/$(length(pairs)) paths")
    println("       index: $index_path")
end

main()
