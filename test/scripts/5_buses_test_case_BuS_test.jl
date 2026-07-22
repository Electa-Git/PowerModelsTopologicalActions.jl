using PowerModels; const _PM = PowerModels
using JuMP
using Ipopt, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using PowerModelsTopologicalActions; const _PMTP = PowerModelsTopologicalActions
#using HSL_jll

#######################################################################################
## Define solvers ##
#######################################################################################

gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => 1e-4)#,"QCPDual" => 1)
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)#,"linear_solver" => "ma97")
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
result_opf_ac = _PMACDC.solve_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.solve_acdcopf(data_5_acdc,LPACCPowerModel,ipopt; setting = s)

#######################################################################################
## Busbar splitting models ##
#######################################################################################
###### AC Busbar splitting models ######
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_dc = deepcopy(data_5_acdc)

data_busbars_ac_split_5_acdc_more_buses = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2
splitted_bus_dc = 2

# Preparing data
data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbars_split(data_5_acdc,splitted_bus_ac)
data_busbars_ac_split_5_acdc_dc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbars_split(data_5_acdc,splitted_bus_dc)

data_busbars_ac_split_5_acdc_more_buses,  switches_couples_ac_5_more_buses,  extremes_ZILs_5_ac_more_buses  = _PMTP.AC_busbars_split(data_busbars_ac_split_5_acdc_more_buses,splitted_bus_ac)
data_busbars_ac_split_5_acdc_more_buses,  switches_couples_dc_5_more_buses,  extremes_ZILs_5_dc_more_buses  = _PMTP.DC_busbars_split(data_busbars_ac_split_5_acdc_more_buses,splitted_bus_dc)


## Running grid topology optimization models and feasibility checks
ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_dc_ref = deepcopy(data_busbars_ac_split_5_acdc_dc)
ac_bs_ac_dc_ref = deepcopy(data_busbars_ac_split_5_acdc_more_buses)

# Grid topology optimization models
result_switches_ac_ac_ref  = _PMTP.run_acdc_BuS_AC(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_lpac_ac_ref  = _PMTP.run_acdc_BuS_AC(ac_bs_ac_ref,LPACCPowerModel,gurobi)

result_switches_lpac_dc_ref = _PMTP.run_acdc_BuS_DC(ac_bs_dc_ref,LPACCPowerModel,gurobi)
result_switches_lpac_ac_dc_ref  = _PMTP.run_acdc_BuS_AC_DC(ac_bs_ac_dc_ref,LPACCPowerModel,gurobi)

# Feasibility checks
feasibility_check_AC_BS_opf_fc = deepcopy(data_busbars_ac_split_5_acdc)
_PMTP.prepare_AC_feasibility_check_AC_busbars(result_switches_lpac_ac_ref,data_busbars_ac_split_5_acdc,feasibility_check_AC_BS_opf_fc,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)
result_feasibility_check_ac = _PMACDC.solve_acdcopf(feasibility_check_AC_BS_opf_fc,ACPPowerModel,ipopt; setting = s)

feasibility_check_DC_BS_opf_fc = deepcopy(data_busbars_ac_split_5_acdc_dc)
_PMTP.prepare_AC_feasibility_check_DC_busbars(result_switches_lpac_dc_ref,data_busbars_ac_split_5_acdc_dc,feasibility_check_DC_BS_opf_fc,switches_couples_dc_5,extremes_ZILs_5_dc,data_5_acdc)
result_feasibility_check_dc = _PMACDC.solve_acdcopf(feasibility_check_DC_BS_opf_fc,ACPPowerModel,ipopt; setting = s)

feasibility_check_AC_DC_BS_opf_fc = deepcopy(data_busbars_ac_split_5_acdc_more_buses)
_PMTP.prepare_AC_feasibility_check_AC_busbars(result_switches_lpac_ac_ref,data_busbars_ac_split_5_acdc,feasibility_check_AC_DC_BS_opf_fc,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)
_PMTP.prepare_AC_feasibility_check_DC_busbars(result_switches_lpac_dc_ref,data_busbars_ac_split_5_acdc_dc,feasibility_check_AC_DC_BS_opf_fc,switches_couples_dc_5,extremes_ZILs_5_dc,data_5_acdc)
result_feasibility_check_ac_dc = _PMACDC.solve_acdcopf(feasibility_check_AC_DC_BS_opf_fc,ACPPowerModel,ipopt; setting = s)



####################
