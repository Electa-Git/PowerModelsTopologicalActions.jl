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
set_optimizer_attribute(gurobi, "MIPGap", 0.001)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Input data ##
#######################################################################################

test_case_5_acdc = "case5_acdc.m"
test_case_67_acdc = "case67.m"


data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)
data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_file_67_acdc = joinpath(@__DIR__,"data_sources",test_case_67_acdc)
data_original_67_acdc = _PM.parse_file(data_file_67_acdc)
data_67_acdc = deepcopy(data_original_67_acdc)
_PMACDC.process_additional_data!(data_67_acdc)


#=
for (l_id,l) in data_39_acdc["load"]
    if l_id != "4"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*6
        l["qd"] = l["qd"]*6
    end
end
=#

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)

data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)


#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac_5 = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_ac_5_qc = _PMACDC.run_acdcopf(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_opf_ac_5_soc = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_opf_ac_5_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)


#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
#splitted_bus_ac = collect(1:5)
#splitted_bus_ac = [23,57,48]
splitted_bus_ac = [2,4]


split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)
data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)

ac_bs_ac = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_soc = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_qc = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac = deepcopy(data_busbars_ac_split_5_acdc)

ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_soc_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_qc_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_ac  = _PMTP.run_acdcsw_AC(ac_bs_ac,ACPPowerModel,juniper)

result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_soc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_soc_ref,SOCWRPowerModel,gurobi)
result_switches_AC_qc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_qc_ref,QCRMPowerModel,gurobi)
result_switches_AC_lpac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_lpac_ref,LPACCPowerModel,gurobi)

try_ac_check = deepcopy(ac_bs_ac)
try_ac_check_orig = deepcopy(ac_bs_ac_ref)
try_ac_check_lpac = deepcopy(ac_bs_lpac_ref) 
try_ac_check_soc = deepcopy(ac_bs_soc_ref)
try_ac_check_qc = deepcopy(ac_bs_qc_ref)


prepare_AC_feasibility_check(result_switches_AC_ac_ref,data_5_acdc,try_ac_check,switches_couples_ac_5,extremes_ZILs_5_ac)
prepare_AC_feasibility_check(result_switches_AC_ac,data_5_acdc,try_ac_check_orig,switches_couples_ac_5,extremes_ZILs_5_ac)
try_ac_check_orig["branch"]["4"]["t_bus"] = 7

prepare_AC_feasibility_check(result_switches_AC_lpac_ref,data_5_acdc,try_ac_check_lpac,switches_couples_ac_5,extremes_ZILs_5_ac)
try_ac_check_lpac["branch"]["4"]["t_bus"] = 7

prepare_AC_feasibility_check(result_switches_AC_qc_ref,data_5_acdc,try_ac_check_qc,switches_couples_ac_5,extremes_ZILs_5_ac)
prepare_AC_feasibility_check(result_switches_AC_soc_ref,data_5_acdc,try_ac_check_soc,switches_couples_ac_5,extremes_ZILs_5_ac)


function prepare_DC_feasibility_check(result_dict, input_dict, input_ac_check, switch_couples, extremes_dict)
    orig_buses = length(input_dict["bus"]) # original bus length
    for (br_id,br) in result_dict["solution"]["dcswitch"]
        if haskey(br,"ZIL") && br["ZIL"] == true
            if result_dict["solution"]["dcswitch"][br_id]["status"] >= 0.9
                delete!(input_ac_check["bus"],"$(extremes_dict[br_id][2])")
            else
                delete!(input_ac_check["dcswitch"],br_id)
            end
        end
    end
    for l in eachindex(switch_couples)
        if result_dict["solution"]["dcswitch"]["$(switch_couples["$l"]["dcswitch_split"])"]["status"] >= 0.9
            aux =  deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["auxiliary"])
            orig = deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["original"])
            if aux == "convdc"
                input_ac_check["convdc"]["$(orig)"]["busdc_i"] = deepcopy(switch_couples[l]["busdc_split"])
            elseif aux == "branchdc"                
                if input_ac_check["branchdc"]["$(orig)"]["fbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["t_bus"] == switch_couples[l]["busdc_split"]
                    input_ac_check["branchdc"]["$(orig)"]["fbusdc"] = deepcopy(switch_couples[l]["busdc_split"])
                elseif input_ac_check["branchdc"]["$(orig)"]["tbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["tbusdc"] == switch_couples[l]["busdc_split"]
                    input_ac_check["branchdc"]["$(orig)"]["tbusdc"] = deepcopy(switch_couples[l]["busdc_split"])
                end
            end
        elseif result_dict["solution"]["dcswitch"]["$(switch_couples["$l"]["dcswitch_split"])"]["status"] <= 0.1
            switch_t = deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]) 
            aux_t = switch_t["auxiliary"]
            orig_t = switch_t["original"]
            print([l,aux_t,orig_t],"\n")
            if result_dict["solution"]["dcswitch"]["$(switch_t["index"])"]["status"] <= 0.1
                delete!(input_ac_check["dcswitch"],"$(switch_t["index"])")
            elseif result_dict["solution"]["dcswitch"]["$(switch_t["index"])"]["status"] >= 0.9
                if aux_t == "convdc"
                    input_ac_check["convdc"]["$(orig_t)"]["busac_i"] = deepcopy(input_ac_check["dcswitch"]["$(switch_t["index"])"]["tbusdc"])
                    print([l,aux_t,orig_t,input_ac_check["convdc"]["$(orig_t)"]["busdc_i"]],"\n")
                elseif aux_t == "branchdc" 
                    if input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_t["index"])"]["tbusdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"]],"\n")
                    elseif input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_t["index"])"]["tbusdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"]],"\n")
                    end
                end
            end

            switch_f = deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["f_sw"])"]) 
            aux_f = switch_f["auxiliary"]
            orig_f = switch_f["original"]
            print([l,aux_f,orig_f],"\n")
            if result_dict["solution"]["dcswitch"]["$(switch_f["index"])"]["status"] <= 0.1
                delete!(input_ac_check["dcswitch"],"$(switch_t["index"])")
            else
                if aux_f == "convdc"
                    input_ac_check["convdc"]["$(orig_f)"]["busac_i"] = deepcopy(input_ac_check["dcswitch"]["$(switch_f["index"])"]["tbusdc"])
                    print([l,aux_t,orig_t,input_ac_check["convdc"]["$(orig_t)"]["busac_i"]],"\n")
                elseif aux_f == "branchdc"
                    if input_ac_check["branchdc"]["$(orig_f)"]["fbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["f_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_f)"]["fbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_f["index"])"]["tbusdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"]],"\n")
                    elseif input_ac_check["branchdc"]["$(orig_f)"]["tbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["f_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_f)"]["tbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_f["index"])"]["tbusdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"]],"\n")
                    end
                end
            end
        end
    end
end


result_opf_ac_5_check = _PMACDC.run_acdcopf(try_ac_check,ACPPowerModel,ipopt; setting = s)
result_opf_ac_5_check_ori = _PMACDC.run_acdcopf(try_ac_check_orig,ACPPowerModel,ipopt; setting = s)
result_opf_ac_5_check_lpac = _PMACDC.run_acdcopf(try_ac_check_lpac,ACPPowerModel,ipopt; setting = s)
result_opf_ac_5_check_qc = _PMACDC.run_acdcopf(try_ac_check_qc,ACPPowerModel,ipopt; setting = s)
result_opf_ac_5_check_soc = _PMACDC.run_acdcopf(try_ac_check_soc,ACPPowerModel,ipopt; setting = s)


function print_connections_AC_feasibility_check(result_dict,input_ac_check)    
    print("The objective function is $(result_dict["objective"])","\n")
    print("Status $(result_dict["termination_status"])","\n")

    print("The branches have the following connections:","\n")
    for i in 1:length(result_dict["solution"]["branch"])
        print(["Branch $(i)",input_ac_check["branch"]["$i"]["f_bus"],input_ac_check["branch"]["$i"]["t_bus"]]," P_fr is $(result_dict["solution"]["branch"]["$i"]["pf"]), $(abs(result_dict["solution"]["branch"]["$i"]["pf"]/input_ac_check["branch"]["$(i)"]["rate_a"])) %,"," Q_fr is $(result_dict["solution"]["branch"]["$i"]["qt"])","\n")
    end
    
    print("The generators have the following connections and results:","\n")
    for i in 1:length(result_dict["solution"]["gen"])
        print(["Gen $(i)",input_ac_check["gen"]["$i"]["gen_bus"],result_dict["solution"]["gen"]["$i"]["pg"]],"\n")
    end

    print("The loads have the following connections:","\n")
    for i in 1:length(input_ac_check["load"])
        print(["Load $(i)",input_ac_check["load"]["$i"]["load_bus"]],"\n")
    end
    
    print("The AC/DC conv have the following connections and results:","\n")
    for i in 1:length(result_dict["solution"]["convdc"])
        print(["Conv AC/DC $(i)","AC bus $(input_ac_check["convdc"]["$i"]["busac_i"])"],"\n")
    end

    print("The DC branches have the following connections and results:","\n")
    for i in 1:length(result_dict["solution"]["convdc"])
        print(["Branch DC $(i)",input_ac_check["branchdc"]["$i"]["fbusdc"],input_ac_check["branchdc"]["$i"]["tbusdc"]," P_fr is $(result_dict["solution"]["branchdc"]["$i"]["pf"]), $(abs(result_dict["solution"]["branchdc"]["$i"]["pf"]/input_ac_check["branchdc"]["$(i)"]["rateA"])) %"],"\n")
    end

    print("The AC buses have the following results:","\n")
    for i in 1:length(result_dict["solution"]["bus"])
        print(["Bus $(i)","Voltage angle $(result_dict["solution"]["bus"]["$i"]["va"])","Voltage magnitude $(result_dict["solution"]["bus"]["$i"]["vm"])"],"\n")
    end
end

print_connections_AC_feasibility_check(result_opf_ac_5_check,try_ac_check)  
print_connections_AC_feasibility_check(result_opf_ac_5_check_ori,try_ac_check_orig) 
print_connections_AC_feasibility_check(result_opf_ac_5,data_5_acdc)

print_connections_AC_feasibility_check(result_opf_ac_5_check_lpac,try_ac_check_lpac)  
print_connections_AC_feasibility_check(result_opf_ac_5_check_qc,try_ac_check_qc)  
print_connections_AC_feasibility_check(result_opf_ac_5_check_soc,try_ac_check_soc)  

print_connections_AC_feasibility_check(result_opf_ac_5,data_5_acdc)

function print_connections_AC_switches(result_switches,input_data)    
    for i in 1:length(input_data["switch"])
        if haskey(input_data["switch"]["$i"],"auxiliary") #&& data_busbars_ac_split_5_acdc["switch"]["$i"]["bus_split"] == 3
            print(["Switch $(i)","Split bus $(input_data["switch"]["$i"]["bus_split"])","Aux $(input_data["switch"]["$i"]["auxiliary"])"*"_$(input_data["switch"]["$i"]["original"])","f_bus $(input_data["switch"]["$i"]["f_bus"])","t_bus $(input_data["switch"]["$i"]["t_bus"])",result_switches["solution"]["switch"]["$i"]["status"]],"\n")
        else
            print("ZIL switch $(i) for split bus $(input_data["switch"]["$i"]["bus_split"]) is $(result_switches["solution"]["switch"]["$i"]["status"])","\n")
        end
    end
end

function print_connections_DC_switches(result_switches,input_data)    
    for i in 1:length(input_data["dcswitch"])
        if haskey(input_data["dcswitch"]["$i"],"auxiliary") #&& data_busbars_ac_split_5_acdc["switch"]["$i"]["bus_split"] == 3
            print(["DC switch $(i)","Split bus $(input_data["dcswitch"]["$i"]["busdc_split"])","Aux $(input_data["dcswitch"]["$i"]["auxiliary"])"*"_$(input_data["dcswitch"]["$i"]["original"])","f_busdc $(input_data["dcswitch"]["$i"]["f_busdc"])","t_busdc $(input_data["dcswitch"]["$i"]["t_busdc"])",result_switches["solution"]["dcswitch"]["$i"]["status"]],"\n")
        else
            print("ZIL switch $(i) for split bus $(input_data["dcswitch"]["$i"]["busdc_split"]) is $(result_switches["solution"]["dcswitch"]["$i"]["status"])","\n")
        end
    end
end

print_connections_AC_switches(result_switches_AC_ac_ref,ac_bs_ac_ref)
print_connections_AC_switches(result_switches_AC_ac,ac_bs_ac)
print_connections_AC_switches(result_switches_AC_lpac_ref,ac_bs_lpac_ref)
print_connections_AC_switches(result_switches_AC_qc_ref,ac_bs_qc_ref)
print_connections_AC_switches(result_switches_AC_soc_ref,ac_bs_soc_ref)


function print_voltage_angles_and_magnitudes_OPF(result_switches,input_data)    
    for i in 1:length(input_data["bus"])
        print(["AC bus $(i)","Voltage angle $(result_switches["solution"]["bus"]["$i"]["va"])","Voltage magnitude $(result_switches["solution"]["bus"]["$i"]["vm"])"],"\n")        
    end
end

print_voltage_angles_and_magnitudes_OPF(result_opf_ac_5_check,try_ac_check)
print_voltage_angles_and_magnitudes_OPF(result_opf_ac_5_check_ori,try_ac_check_orig)
print_voltage_angles_and_magnitudes_OPF(result_opf_ac_5_check_lpac,try_ac_check_lpac)
print_voltage_angles_and_magnitudes_OPF(result_opf_ac_5_check_qc,try_ac_check_qc)
print_voltage_angles_and_magnitudes_OPF(result_opf_ac_5_check_soc,try_ac_check_soc)

















