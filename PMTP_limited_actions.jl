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

gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => 1e-4)#,"QCPDual" => 1)
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0,"linear_solver" => "ma97")
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case5_acdc.m"
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)


#######################################################################################
## Busbar splitting models ##
#######################################################################################
###### AC Busbar splitting models ######
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_plus = deepcopy(data_5_acdc)

data_busbars_ac_split_5_acdc_more_buses = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
data_busbars_ac_split_5_acdc_plus,  switches_couples_ac_5_plus,  extremes_ZILs_5_ac_plus  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_plus,splitted_bus_ac)

ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)

result_switches_lpac_ac_ref  = _PMTP.run_acdcsw_AC_big_M_ZIL(ac_bs_lpac_ref,LPACCPowerModel,gurobi)

result_switches_lpac_ac_la  = _PMTP.run_acdcsw_AC_big_M_ZIL_limited_action(ac_bs_lpac_ref,LPACCPowerModel,gurobi)


#result_switches_AC_ac_ref_not_Line  = _PMTP.run_acdcsw_AC_big_M(ac_bs_ac_ref_Line,ACPPowerModel,juniper)

