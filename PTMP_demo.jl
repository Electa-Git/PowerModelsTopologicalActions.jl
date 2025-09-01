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
splitted_bus_ac = [1,2,3,4,5]
#splitted_bus_ac = 2
#splitted_bus_ac_more_buses = [2,4]

splitted_bus_dc = 2


data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
data_busbars_ac_split_5_acdc_plus,  switches_couples_ac_5_plus,  extremes_ZILs_5_ac_plus  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_plus,splitted_bus_ac)

#data_busbars_ac_split_5_acdc_more_buses,  switches_couples_ac_5_more_buses,  extremes_ZILs_5_ac_more_buses  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_more_buses,splitted_bus_ac_more_buses)
#data_busbars_ac_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_dc)



# Duplicating the network data
ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_ac_ref_Line = deepcopy(data_busbars_ac_split_5_acdc)

ac_bs_ac_ref_plus = deepcopy(data_busbars_ac_split_5_acdc_plus)
ac_bs_ac_ref_Line_plus = deepcopy(data_busbars_ac_split_5_acdc_plus)

ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac_ref_Line = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_dc_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_dc_ref_Line = deepcopy(data_busbars_ac_split_5_acdc)

result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_lpac_ac_ref  = _PMTP.run_acdcsw_AC(ac_bs_lpac_ref,LPACCPowerModel,gurobi)
result_switches_soc_ac_ref  = _PMTP.run_acdcsw_AC(ac_bs_ac_ref,SOCWRPowerModel,gurobi)
result_switches_qc_ac_ref  = _PMTP.run_acdcsw_AC(ac_bs_ac_ref,QCRMPowerModel,gurobi)

#result_switches_AC_ac_ref_not_Line  = _PMTP.run_acdcsw_AC_big_M(ac_bs_ac_ref_Line,ACPPowerModel,juniper)

# Run models
# Feasibility check for the AC busbar splitting
feasibility_check_AC_BS_opf_ac_ref_status = deepcopy(data_busbars_ac_split_5_acdc)
_PMTP.prepare_AC_feasibility_check(result_switches_AC_ac_ref,data_busbars_ac_split_5_acdc,feasibility_check_AC_BS_opf_ac_ref_status,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)
result_feasibility_check = _PMACDC.run_acdcopf(feasibility_check_AC_BS_opf_ac_ref_status,ACPPowerModel,ipopt; setting = s)

feasibility_check_AC_BS_opf_lpac_ref_status = deepcopy(data_busbars_ac_split_5_acdc)
_PMTP.prepare_AC_feasibility_check(result_switches_lpac_ac_ref,data_busbars_ac_split_5_acdc,feasibility_check_AC_BS_opf_lpac_ref_status,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)
result_feasibility_check_lpac = _PMACDC.run_acdcopf(feasibility_check_AC_BS_opf_lpac_ref_status,ACPPowerModel,ipopt; setting = s)

for sw_id in 1:length(data_busbars_ac_split_5_acdc["switch"])
    if !haskey(ac_bs_ac_ref["switch"]["$(sw_id)"],"auxiliary")
        println("$sw_id, f_bus sw $(ac_bs_ac_ref["switch"]["$sw_id"]["f_bus"]), t_bus sw $(ac_bs_ac_ref["switch"]["$sw_id"]["t_bus"]),status $(result_switches_AC_ac_ref["solution"]["switch"]["$sw_id"]["status"])")
    else
        if ac_bs_ac_ref["switch"]["$(sw_id)"]["bus_split"] == 3
            println("$sw_id, aux $(ac_bs_ac_ref["switch"]["$sw_id"]["auxiliary"]), orig $(ac_bs_ac_ref["switch"]["$sw_id"]["original"]), f_bus sw $(ac_bs_ac_ref["switch"]["$sw_id"]["f_bus"]), t_bus sw $(ac_bs_ac_ref["switch"]["$sw_id"]["t_bus"]),status $(result_switches_AC_ac_ref["solution"]["switch"]["$sw_id"]["status"])")
        end
    end
end

for (g_id,g) in feasibility_check_AC_BS_opf_lpac_ref_status["gen"]
    println("Gen $g_id is connected to bus $(g["gen_bus"])")
end
for (l_id,l) in feasibility_check_AC_BS_opf_lpac_ref_status["load"]
    println("Load $l_id is connected to bus $(l["load_bus"])")
end
for (br_id,br) in feasibility_check_AC_BS_opf_lpac_ref_status["branch"]
    println("Branch $br_id is connected to bus $(br["f_bus"])")
    println("Branch $br_id is connected to bus $(br["t_bus"])")
end
for (cv_id,cv) in feasibility_check_AC_BS_opf_lpac_ref_status["convdc"]
    println("Conv $cv_id is connected to bus $(cv["busac_i"])")
end


for (i,sw) in ac_bs_lpac_ref["switch"]
    if haskey(sw,"auxiliary")
        println([i," ",ac_bs_lpac_ref["switch"]["$i"]["original"]," ",ac_bs_lpac_ref["switch"]["$i"]["auxiliary"]])
    else
        println(i," ")
    end
end

