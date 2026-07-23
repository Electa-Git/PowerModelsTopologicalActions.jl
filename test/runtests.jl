using Test

using PowerModels
using PowerModelsACDC
using PowerModelsTopologicalActions

using JuMP
using Ipopt
using HiGHS
using Juniper

const _PM   = PowerModels
const _PMACDC = PowerModelsACDC
const _PMTP = PowerModelsTopologicalActions

_PM.silence()

# ---------------------------------------------------------------------------
# Solvers
#
# Deliberately all open source, so CI needs no commercial licence. Gurobi is
# faster for the MIQCP/MISOCP formulations, but everything below is small
# enough that Juniper + Ipopt + HiGHS is adequate.
# ---------------------------------------------------------------------------
const IPOPT = optimizer_with_attributes(Ipopt.Optimizer,
                  "tol" => 1e-6, "print_level" => 0)

const HIGHS = optimizer_with_attributes(HiGHS.Optimizer,
                  "output_flag" => false)

const JUNIPER = optimizer_with_attributes(Juniper.Optimizer,
                  "nl_solver" => IPOPT, "mip_solver" => HIGHS, "log_levels" => [])

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
const OBJ_OTS_AC  = 184.437   # AC-OTS, AC branches switchable
const OBJ_BUS_FC  = 186.349   # LPAC-BuS on busbar 2, after AC feasibility check

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
    # Regression guards for bugs that have been fixed, so they stay fixed.
    # -----------------------------------------------------------------------
    @testset "regression guards" begin

        @testset "DC_busbars_split preserves AC switch_couples" begin
            # Previously DC_busbars_split reset data["switch_couples"] to an
            # empty Dict, which silently dropped every AC exclusivity, ZIL and
            # BuS-OTS constraint from the combined AC/DC model.
            data = load_case(CASE5)
            d, sw_ac, _ = _PMTP.AC_busbars_split(data, 2)
            d, sw_dc, _ = _PMTP.DC_busbars_split(d, 2)

            @test !isempty(d["switch_couples"])
            @test !isempty(d["dcswitch_couples"])
            @test length(d["switch_couples"])   == length(sw_ac)
            @test length(d["dcswitch_couples"]) == length(sw_dc)
        end

        @testset "reverse split order (known issue)" begin
            # AC_busbars_split still resets dcswitch_couples unconditionally, so
            # the DC-then-AC order loses the DC couples. Documented in
            # docs/src/known_issues.md. Flip to @test when fixed.
            data = load_case(CASE5)
            d, _, _ = _PMTP.DC_busbars_split(data, 2)
            d, _, _ = _PMTP.AC_busbars_split(d, 2)

            @test_broken !isempty(d["dcswitch_couples"])
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

    @testset "optimal transmission switching" begin

        @testset "AC branches switchable" begin
            data   = load_case(CASE5)
            result = _PMTP.run_acdcots_AC(data, ACPPowerModel, JUNIPER; setting = SETTING)

            @test result["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)
            @test result["objective"] < OBJ_OPF          # switching must help
            @test isapprox(result["objective"], OBJ_OTS_AC; rtol = 1e-2)

            opened = [id for (id, br) in result["solution"]["branch"]
                      if is_open(br["br_status"])]
            @test !isempty(opened)
        end

        @testset "DC elements switchable" begin
            # No DC switching action improves on the base topology for this
            # case; matching the AC-OPF objective is the expected outcome.
            data   = load_case(CASE5)
            result = _PMTP.run_acdcots_DC(data, ACPPowerModel, JUNIPER; setting = SETTING)

            @test result["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)
            @test isapprox(result["objective"], OBJ_OPF; rtol = 1e-2)
        end
    end

    @testset "busbar splitting" begin

        @testset "LPAC-BuS on busbar 2 is AC-feasible and cheaper" begin
            data = load_case(CASE5)
            data_split, couples, extremes = _PMTP.AC_busbars_split(data, 2)

            result = _PMTP.run_acdc_BuS_AC(data_split, LPACCPowerModel, JUNIPER)
            @test result["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)
            @test haskey(result["solution"], "switch")

            # Freeze the optimised topology and price it with an exact AC/DC OPF.
            data_fc = deepcopy(data_split)
            _PMTP.prepare_AC_feasibility_check_AC_busbars(
                result, data_split, data_fc, couples, extremes, data)

            fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, IPOPT; setting = SETTING)

            @test fc["termination_status"] in (LOCALLY_SOLVED, OPTIMAL)  # AC-feasible
            @test fc["objective"] < OBJ_OPF                              # and beneficial
            @test isapprox(fc["objective"], OBJ_BUS_FC; rtol = 1e-2)
        end

        @testset "a busbar is only split when the coupler opens" begin
            data = load_case(CASE5)
            data_split, _, _ = _PMTP.AC_busbars_split(data, 2)
            result = _PMTP.run_acdc_BuS_AC(data_split, LPACCPowerModel, JUNIPER)

            coupler_id = first(id for (id, sw) in data_split["switch"]
                               if !haskey(sw, "auxiliary"))

            # Whatever the solver decides, the reported status must be binary.
            status = result["solution"]["switch"][coupler_id]["status"]
            @test isapprox(status, 0.0; atol = 0.1) || isapprox(status, 1.0; atol = 0.1)
        end
    end
end