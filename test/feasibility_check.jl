# Solver-free regression tests for the AC feasibility-check preparation.
#
# When both ends of a branch sit on busbars that are split, each end is moved to
# its own auxiliary bus. `prepare_AC_feasibility_check_*_busbars` must then put
# every end back on the busbar section that its closed element switch points to.
# The switch statuses below are set by hand, so no optimisation is needed.
#
# One branch is reversed in the input data so that the from->to directions of
# the branches between busbars 1, 2 and 3 form a cycle (1->2->3->1). Whatever
# order the busbars are processed in (it follows Dict iteration order, which
# differs between Julia versions), at least one branch then has its to-end
# processed before its from-end.

"Statuses for the two hand-made topologies: all couplers closed, or all couplers open with every element on the added busbar section."
function handmade_switch_statuses(switches, split_key, to_key, closed_couplers, target)
    status = Dict{String,Any}()
    for (id, sw) in switches
        on = haskey(sw, "auxiliary") ? sw[to_key] == target(sw[split_key]) : closed_couplers
        status[id] = Dict{String,Any}("status" => on ? 1.0 : 0.0)
    end
    return status
end

@testset "AC feasibility check: branches between two split busbars" begin
    data = load_case(CASE5)
    br = data["branch"]["2"]                       # 1 -> 3, reversed to 3 -> 1
    br["f_bus"], br["t_bus"] = br["t_bus"], br["f_bus"]
    original_ends = Dict(id => (b["f_bus"], b["t_bus"]) for (id, b) in data["branch"])

    data_split, couples, extremes = _PMTP.AC_busbars_split(data, [1, 2, 3, 4, 5])
    added_section = Dict(sw["bus_split"] => sw["t_bus"]
                         for (_, sw) in data_split["switch"] if !haskey(sw, "auxiliary"))

    for (case, closed, target) in (("couplers closed", true, identity),
                                   ("couplers open", false, b -> added_section[b]))
        @testset "$case" begin
            status = handmade_switch_statuses(data_split["switch"], "bus_split", "t_bus", closed, target)
            result = Dict{String,Any}("solution" => Dict{String,Any}("switch" => status))
            data_fc = deepcopy(data_split)
            redirect_stdout(devnull) do
                _PMTP.prepare_AC_feasibility_check_AC_busbars(result, data_split, data_fc, couples, extremes, data)
            end
            for (id, (f, t)) in original_ends
                @test (data_fc["branch"][id]["f_bus"], data_fc["branch"][id]["t_bus"]) == (target(f), target(t))
            end
        end
    end
end

@testset "DC feasibility check: DC branches between two split DC busbars" begin
    data = load_case(CASE5)
    br = data["branchdc"]["3"]                     # 1 -> 3, reversed to 3 -> 1
    br["fbusdc"], br["tbusdc"] = br["tbusdc"], br["fbusdc"]
    original_ends = Dict(id => (b["fbusdc"], b["tbusdc"]) for (id, b) in data["branchdc"])

    data_split, couples, extremes = _PMTP.DC_busbars_split(data, [1, 2, 3])
    added_section = Dict(sw["busdc_split"] => sw["t_busdc"]
                         for (_, sw) in data_split["dcswitch"] if !haskey(sw, "auxiliary"))

    for (case, closed, target) in (("couplers closed", true, identity),
                                   ("couplers open", false, b -> added_section[b]))
        @testset "$case" begin
            status = handmade_switch_statuses(data_split["dcswitch"], "busdc_split", "t_busdc", closed, target)
            result = Dict{String,Any}("solution" => Dict{String,Any}("dcswitch" => status))
            data_fc = deepcopy(data_split)
            redirect_stdout(devnull) do
                _PMTP.prepare_AC_feasibility_check_DC_busbars(result, data_split, data_fc, couples, extremes, data)
            end
            for (id, (f, t)) in original_ends
                @test (data_fc["branchdc"][id]["fbusdc"], data_fc["branchdc"][id]["tbusdc"]) == (target(f), target(t))
            end
        end
    end
end
