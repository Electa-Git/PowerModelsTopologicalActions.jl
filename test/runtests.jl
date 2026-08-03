using Test

using PowerModels
using PowerModelsACDC
using PowerModelsTopologicalActions

using JuMP
using JSON
using Ipopt
using HiGHS
using Juniper
using Gurobi

const _PM   = PowerModels
const _PMACDC = PowerModelsACDC
const _PMTP = PowerModelsTopologicalActions

_PM.silence()

# ---------------------------------------------------------------------------
# Solvers
#
# Gurobi is used for the LPAC formulations, Juniper = Ipopt + Gurobi for the AC MINLP formulation.
# ---------------------------------------------------------------------------
const IPOPT = optimizer_with_attributes(Ipopt.Optimizer,
                  "tol" => 1e-6, "print_level" => 0)

const HIGHS = optimizer_with_attributes(HiGHS.Optimizer,
                  "output_flag" => false)

const GUROBI_AVAILABLE = try
    Gurobi.Env(); true
catch err
    @warn "Gurobi license unavailable — falling back to HiGHS" err
    false
end
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)


const GUROBI = GUROBI_AVAILABLE ?
    optimizer_with_attributes(Gurobi.Optimizer, "OutputFlag" => 0) : nothing

# Juniper (MINLP) = Ipopt for the NLP subproblems + a MIP solver.
# Use Gurobi as the MIP solver when a license is present, HiGHS otherwise,
# so the suite still runs without a Gurobi licence (e.g. forked-PR CI).
const JUNIPER = optimizer_with_attributes(Juniper.Optimizer,
                  "nl_solver"  => IPOPT,
                  "mip_solver" => GUROBI_AVAILABLE ? GUROBI : HIGHS,
                  "log_levels" => [])

const SETTING = Dict("output" => Dict("branch_flows" => true),
                     "conv_losses_mp" => true)

const DATA_DIR = joinpath(@__DIR__, "data_sources")
const CASE5    = joinpath(DATA_DIR, "case5_acdc.m")

"Parse a case and apply the PowerModelsACDC pre-processing every model expects."
function load_case(path)
    data = _PM.parse_file(path)
    _PMACDC.process_additional_data!(data)
    return data
end

is_open(x) = x < 0.1

# Reference values from Bastianel et al. (2026), Tables 4 and 5, case 5-buses.
# Tolerances are loose because the MINLP results are local optima and depend on
# the solver stack; they are tight enough to catch a real regression.
const OBJ_OPF     = 194.139   # AC-OPF baseline
const OBJ_BUS_FC  = 186.349   # LPAC-BuS on busbar 2, after AC feasibility check
const OBJ_BUS_AC  = 185.209   # AC-OTS, AC branches switchable

@testset "PowerModelsTopologicalActions.jl" begin

    # -----------------------------------------------------------------------
    # Fast, solver-free structural tests. These catch most data-preparation
    # regressions and run in milliseconds.
    # -----------------------------------------------------------------------
    @testset "data preparation" begin

        @testset "AC_busbars_split structure" begin
            data = load_case(CASE5)
            n_bus_before = length(data["bus"])

            data_split, couples, extremes = _PMTP.AC_busbars_split(data, 2)

            # One busbar coupler per split busbar, two switches per moved element.
            #
            # NOTE: the coupler is identified by the ABSENCE of the "auxiliary"
            # key, not by ZIL. Element switches are deepcopied from the coupler
            # and inherit ZIL => true, so ZIL is not a discriminator.
            couplers = [id for (id, sw) in data_split["switch"] if !haskey(sw, "auxiliary")]
            @test length(couplers) == 1
            @test length(data_split["switch"]) == 2 * length(couples) + 1

            # Every couple points at two distinct switches on the split busbar.
            for (_, c) in couples
                @test c["f_sw"] != c["t_sw"]
                @test c["bus_split"] == 2
                @test haskey(data_split["switch"], "$(c["f_sw"])")
                @test haskey(data_split["switch"], "$(c["t_sw"])")
            end

            # The busbar gained a second half plus one auxiliary bus per element.
            @test length(data_split["bus"]) == n_bus_before + 1 + length(couples)
            @test data_split["bus"]["2"]["split"] == true
            @test haskey(extremes, 2) || !isempty(extremes)

            # Couples are exposed on the network dict, which is where the
            # model reads them from.
            @test !isempty(data_split["switch_couples"])
        end

        @testset "AC_busbars_split does not mutate its input" begin
            data = load_case(CASE5)
            n_bus  = length(data["bus"])
            n_sw   = length(get(data, "switch", Dict()))

            _PMTP.AC_busbars_split(data, 2)

            @test length(data["bus"]) == n_bus
            @test length(get(data, "switch", Dict())) == n_sw
        end

        @testset "DC_busbars_split structure" begin
            data = load_case(CASE5)
            data_split, couples, _ = _PMTP.DC_busbars_split(data, 2)

            couplers = [id for (id, sw) in data_split["dcswitch"] if !haskey(sw, "auxiliary")]
            @test length(couplers) == 1
            @test length(data_split["dcswitch"]) == 2 * length(couples) + 1
            @test !isempty(data_split["dcswitch_couples"])
        end

        @testset "splitting multiple busbars" begin
            data = load_case(CASE5)
            _, couples_one, _ = _PMTP.AC_busbars_split(data, 2)
            _, couples_two, _ = _PMTP.AC_busbars_split(data, [2, 3])

            @test length(couples_two) > length(couples_one)
        end

    end

    # -----------------------------------------------------------------------
    # Optimisation results. Slower; these are the numbers from the paper.
    # -----------------------------------------------------------------------
    @testset "AC/DC OPF baseline" begin
        data   = load_case(CASE5)
        result = _PMACDC.solve_acdcopf(data, ACPPowerModel, IPOPT; setting = SETTING)

        @test result["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)
        @test isapprox(result["objective"], OBJ_OPF; rtol = 1e-3)
    end


    @testset "Gurobi formulations" begin
        if GUROBI_AVAILABLE
            data = load_case(CASE5)
            data_split, couples, extremes = _PMTP.AC_busbars_split(data, 2)

            result = _PMTP.run_acdc_BuS_AC(data_split, LPACCPowerModel, GUROBI)
            data_fc = deepcopy(data_split)
            _PMTP.prepare_AC_feasibility_check_AC_busbars(result, data_split, data_fc, couples, extremes, data)
            result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)

            @test result_fc["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)
            @test isapprox(result_fc["objective"], OBJ_BUS_FC; rtol = 1e-2)


            result_minlp = _PMTP.run_acdc_BuS_AC(data_split, ACPPowerModel, JUNIPER)
            @test isapprox(result_minlp["objective"], OBJ_BUS_AC; rtol = 1e-2)

        else
            data = load_case(CASE5)
            data_split, couples, extremes = _PMTP.AC_busbars_split(data, 2)

            result = JSON.parsefile(joinpath(dirname(@__DIR__),"tutorials","results","result_LPAC_BuS_AC_busbar_2.json"))
            data_fc = deepcopy(data_split)
            _PMTP.prepare_AC_feasibility_check_AC_busbars(result, data_split, data_fc, couples, extremes, data)
            result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, IPOPT; setting = s)

            @test result_fc["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)
            @test isapprox(result_fc["objective"], OBJ_BUS_FC; rtol = 1e-2)
        end
    end
end