using PowerModels; const _PM = PowerModels
using JuMP
using Ipopt, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using HSL_jll

#######################################################################################
## Define solvers ##
#######################################################################################

mip_gap = 1e-5
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => mip_gap,"QCPDual" => 1)
gurobi_opf = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => mip_gap)#,"QCPDual" => 1)
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0,"linear_solver" => "ma97")
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case5_acdc.m"
test_case_67_acdc = "case67.m"
test_case_2300_acdc = "nem_2300bus_thermal_limits_gen_costs_hvdc.m"  
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_file_67_acdc = joinpath(@__DIR__,"data_sources",test_case_67_acdc)
data_file_2300_acdc = joinpath(@__DIR__,"data_sources",test_case_2300_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_67_acdc = _PM.parse_file(data_file_67_acdc)
_PMACDC.process_additional_data!(data_67_acdc)

data_2300_acdc = _PM.parse_file(data_file_2300_acdc)
_PMACDC.process_additional_data!(data_2300_acdc)


#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_2300_ac = _PMACDC.run_acdcopf(data_2300_acdc,ACPPowerModel,ipopt; setting = s_dual)
result_opf_2300_lpac = _PMACDC.run_acdcopf(data_2300_acdc,LPACCPowerModel,gurobi; setting = s_dual)

duals_2300 = []
duals_branches_2300 = []
for (b_id,b) in data_2300_acdc["bus"]
    push!(duals_2300,[b_id,result_opf_2300_lpac["solution"]["bus"][b_id]["lam_kcl_r"]])
end
for (b_id,b) in data_2300_acdc["branch"]
    push!(duals_branches_2300,[b_id,b["f_bus"],b["t_bus"],abs(result_opf_2300_lpac["solution"]["bus"]["$(b["f_bus"])"]["lam_kcl_r"]-result_opf_2300_lpac["solution"]["bus"]["$(b["t_bus"])"]["lam_kcl_r"])])
end
sort(duals_branches_2300, by = x -> x[4], rev = true)

# Summing elements with the same f_bus or t_bus
branch_sums = Dict{Int, Float64}()
for branch in duals_branches_2300
    f_bus, t_bus, value = branch[2], branch[3], branch[4]
    branch_sums[f_bus] = get(branch_sums, f_bus, 0.0) + value
    branch_sums[t_bus] = get(branch_sums, t_bus, 0.0) + value
end

branch_sums
for (bus, sum_value) in branch_sums
    println("Bus $bus: Sum of branch values = $sum_value")
end
sorted_branch_sums = sort(collect(branch_sums), by = x -> x[2], rev = true)

splitted_bus_ac = [1720,832]
grid_2300 = deepcopy(data_2300_acdc)
grid_2300,  switches_couples_2300,  extremes_ZILs_2300  = _PMTP.AC_busbar_split_more_buses(grid_2300,splitted_bus_ac)

grid_2300_sp = deepcopy(grid_2300)
_PMTP.prepare_starting_value_dict_lpac(result_opf_2300_lpac,grid_2300_sp)

result_switches_lpac_ac_ref_2300  = _PMTP.run_acdcsw_AC_big_M_ZIL_sp(grid_2300_sp,LPACCPowerModel,gurobi)


result_switches_lpac_ac_ref_2300["solution"]["switch"]["1"]["status"]
(result_switches_lpac_ac_ref_2300["objective"] - 1.0)/result_opf_2300_lpac["objective"]

grid_2300["switch"]["1"]

function split_one_bus_per_time(test_case,results_dict,results_dict_ac_check,results_dict_lpac_check)
    for (b_id,b) in test_case["bus"]
        results_dict["$b_id"] = Dict{String,Any}()
        results_dict_ac_check["$b_id"] = Dict{String,Any}()
        test_case_bs = deepcopy(test_case)
        splitted_bus_ac = parse(Int64,b_id)
        test_case_bs,  switches_couples_ac,  extremes_ZILs_ac  = _PMTP.AC_busbar_split_more_buses(test_case_bs,splitted_bus_ac)
        test_case_bs["switch"]["1"]["cost"] = 10.0
        results_dict["$b_id"] = _PMTP.run_acdcsw_AC_big_M_ZIL(test_case_bs,LPACCPowerModel,gurobi)
        test_case_bs_check = deepcopy(test_case_bs)
        test_case_bs_check_auxiliary = deepcopy(test_case_bs)
        if results_dict["$b_id"]["termination_status"] == JuMP.OPTIMAL
            _PMTP.prepare_AC_feasibility_check(results_dict["$b_id"],test_case_bs_check_auxiliary,test_case_bs_check,switches_couples_ac,extremes_ZILs_ac,test_case)
            results_dict_ac_check["$b_id"] = deepcopy(_PMACDC.run_acdcopf(test_case_bs_check,ACPPowerModel,ipopt; setting = s))
            results_dict_lpac_check["$b_id"] = deepcopy(_PMACDC.run_acdcopf(test_case_bs_check,LPACCPowerModel,gurobi; setting = s))
        end
    end
end

grid_2300 = deepcopy(data_2300_acdc)
result_bs = Dict{String,Any}()
results_ac_check = Dict{String,Any}()
results_lpac_check = Dict{String,Any}()
@time split_one_bus_per_time(grid_2300,result_bs,results_ac_check,results_lpac_check)

dio_boia = [[b_id,result_bs["$b_id"]["objective"]] for b_id in keys(result_bs)]
sort(dio_boia, by = x -> x[2],rev=false)

result_bs["43"]["solution"]["switch"]["1"]["status"]

result_bs["43"]["objective"]/result_opf_67_lpac["objective"]

