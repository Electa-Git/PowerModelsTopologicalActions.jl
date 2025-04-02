using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using Mosek, MosekTools

#######################################################################################
## Define solver ##
#######################################################################################

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-4, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
set_optimizer_attribute(gurobi, "MIPGap", 1e-4)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Input data ##
#######################################################################################

test_case = "case5_acdc.m"


#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file = joinpath(@__DIR__,"data_sources",test_case)
data_original = _PM.parse_file(data_file)
data = deepcopy(data_original)
_PMACDC.process_additional_data!(data)


#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data,ACPPowerModel,ipopt; setting = s)
result_opf_ac_lpac = _PMACDC.run_acdcopf(data,LPACCPowerModel,gurobi; setting = s)
result_opf_ac_qc = _PMACDC.run_acdcopf(data,QCRMPowerModel,gurobi; setting = s)
result_opf_ac_soc = _PMACDC.run_acdcopf(data,SOCWRPowerModel,gurobi; setting = s)


#######################################################################################
## Busbar splitting models ##
#######################################################################################
data_busbars_ac_split = deepcopy(data)
data_busbars_dc_split = deepcopy(data)

# Selecting which busbars are split
splitted_bus_ac = 2
splitted_bus_dc = 1


split_elements = _PMTP.elements_AC_busbar_split(data)
data_busbars_ac_split,  switches_couples_ac,  extremes_ZILs_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split,splitted_bus_ac)

ac_bs_ac_ref = deepcopy(data_busbars_ac_split)
ac_bs_ac_ref_orig = deepcopy(data_busbars_ac_split)
ac_bs_lpac_ref = deepcopy(data_busbars_ac_split)
ac_bs_soc_ref = deepcopy(data_busbars_ac_split)
ac_bs_qc_ref = deepcopy(data_busbars_ac_split)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches

result_switches_AC_lpac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_lpac_ref,LPACCPowerModel,gurobi)
result_switches_AC_ac_ref_orig  = _PMTP.run_acdcsw_AC(ac_bs_ac_ref_orig,ACPPowerModel,juniper)
result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_soc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_soc_ref,SOCWRPowerModel,gurobi)
result_switches_AC_qc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_qc_ref,QCRMPowerModel,gurobi)


folder_results = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting"

results_bs_lpac = JSON.json(result_switches_AC_lpac_ref)
open(joinpath(folder_results,"case_5/Results_ac_bs_2_4_lpac.json"),"w" ) do f
    write(f,results_bs_lpac)
end
results_bs_ac = JSON.json(result_switches_AC_ac_ref)
open(joinpath(folder_results,"case_5/Results_ac_bs_2_4_ac.json"),"w" ) do f
    write(f,results_bs_ac)
end


#print_connections_AC_switches(result_switches_AC_lpac_ref,ac_bs_lpac_ref)
#print_connections_AC_switches(result_switches_AC_lpac_ref,ac_bs_lpac_ref)
#print_connections_AC_switches(result_switches_AC_ac_ref,ac_bs_ac_ref)


try_ac_check = deepcopy(ac_bs_ac_ref)
try_ac_check_orig = deepcopy(ac_bs_ac_ref)
try_ac_check_lpac = deepcopy(ac_bs_lpac_ref) 
try_ac_check_qc = deepcopy(ac_bs_qc_ref) 
try_ac_check_soc = deepcopy(ac_bs_soc_ref) 


prepare_AC_feasibility_check(result_switches_AC_ac_ref,data,try_ac_check,switches_couples_ac,extremes_ZILs_ac)
prepare_AC_feasibility_check(result_switches_AC_ac_ref_orig,data,try_ac_check_orig,switches_couples_ac,extremes_ZILs_ac)
prepare_AC_feasibility_check(result_switches_AC_lpac_ref,data,try_ac_check_lpac,switches_couples_ac,extremes_ZILs_ac)
prepare_AC_feasibility_check(result_switches_AC_soc_ref,data,try_ac_check_soc,switches_couples_ac,extremes_ZILs_ac)
prepare_AC_feasibility_check(result_switches_AC_qc_ref,data,try_ac_check_qc,switches_couples_ac,extremes_ZILs_ac)


data_bs_lpac_check = JSON.json(try_ac_check_lpac)
open(joinpath(folder_results,"case_39/Data_ac_bs_13_lpac_check.json"),"w" ) do f
    write(f,data_bs_lpac_check)
end
data_bs_ac_check = JSON.json(try_ac_check)
open(joinpath(folder_results,"case_39/Results_ac_bs_13_ac_check.json"),"w" ) do f
    write(f,data_bs_ac_check)
end


result_opf_ac_check = _PMACDC.run_acdcopf(try_ac_check,ACPPowerModel,ipopt; setting = s)
result_opf_ac_check = _PMACDC.run_acdcopf(try_ac_check_orig,ACPPowerModel,ipopt; setting = s)
result_opf_ac_check_lpac = _PMACDC.run_acdcopf(try_ac_check_lpac,ACPPowerModel,ipopt; setting = s)
result_opf_ac_check_qc = _PMACDC.run_acdcopf(try_ac_check_qc,ACPPowerModel,ipopt; setting = s)
result_opf_ac_check_soc = _PMACDC.run_acdcopf(try_ac_check_soc,ACPPowerModel,ipopt; setting = s)


#=
splitted_bus_dc = 4


split_elements = _PMTP.elements_DC_busbar_split(data)
data_busbars_dc_split,  switches_couples_dc,  extremes_ZILs_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split,splitted_bus_dc)


dc_bs_ac_ref = deepcopy(data_busbars_dc_split)
dc_bs_lpac_ref = deepcopy(data_busbars_dc_split)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches

result_switches_AC_lpac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_lpac_ref,LPACCPowerModel,gurobi)
result_switches_AC_ac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_ac_ref,ACPPowerModel,juniper)


try_dc_check = deepcopy(dc_bs_ac_ref)
try_dc_check_lpac = deepcopy(dc_bs_lpac_ref) 


prepare_DC_feasibility_check(result_switches_AC_lpac_ref,data,try_dc_check_lpac,switches_couples_dc,extremes_ZILs_dc)


data_bs_lpac_check = JSON.json(try_ac_check_lpac)
open(joinpath(folder_results,"case_39/Data_ac_bs_13_lpac_check.json"),"w" ) do f
    write(f,data_bs_lpac_check)
end
data_bs_ac_check = JSON.json(try_ac_check)
open(joinpath(folder_results,"case_39/Results_ac_bs_13_ac_check.json"),"w" ) do f
    write(f,data_bs_ac_check)
end


result_opf_ac_check = _PMACDC.run_acdcopf(try_ac_check,ACPPowerModel,ipopt; setting = s)
result_opf_ac_check_lpac = _PMACDC.run_acdcopf(try_ac_check_lpac,ACPPowerModel,ipopt; setting = s)
=#