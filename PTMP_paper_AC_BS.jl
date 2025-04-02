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

gurobi_opf = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => 1e-5)#,"QCPDual" => 1)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => 1e-4,"QCPDual" => 1,"BarHomogeneous" => -1)#,"time_limit" => 300)

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0,"linear_solver" => "ma97")
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case3120sp_mcdc.m"
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)



for (b_id,b) in data_5_acdc["bus"]
    b["vmin"] = 0.9
    b["vmax"] = 1.1
end
#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)
result_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi_opf; setting = s)

duals = []
for (b_id,b) in data_5_acdc["bus"]
    println([b_id,result_opf_ac["solution"]["bus"]["$b_id"]["lam_kcl_r"]])
    push!(duals,[b_id,result_opf_ac["solution"]["bus"]["$b_id"]["lam_kcl_r"]])
end
sort(duals, by = x -> x[2])

for (b_id,b) in data_5_acdc["branch"]
    println([b_id,abs(result_opf_lpac["solution"]["bus"]["$(b["f_bus"])"]["lam_kcl_r"]-result_opf_lpac["solution"]["bus"]["$(b["t_bus"])"]["lam_kcl_r"])])
end

result_bs = Dict{String,Any}()
result_bs_ac_check = Dict{String,Any}()
result_bs_lpac_check = Dict{String,Any}()

function prepare_AC_feasibility_check(result_dict, input_dict, input_ac_check, switch_couples, extremes_dict,input_base)
    orig_buses = maximum(parse.(Int, keys(input_base["bus"]))) # maximum value before splitting (in case the buses are not in numerical order)
    for (sw_id,sw) in input_dict["switch"]
        if !haskey(sw,"auxiliary")
            println("SWITCH $sw_id, BUS from $(sw["f_bus"]), BUS to $(sw["t_bus"])")
            if result_dict["solution"]["switch"][sw_id]["status"] >= 0.9 # Just reconnecting stuff
                println("Switch $sw_id is closed, Connecting everything back, no busbar splitting on bus $(sw["bus_split"])")
                for l in keys(switch_couples)
                    #if haskey(switch_couples,sw_id) && switch_couples[l]["bus_split"] == sw["bus_split"] -> wrong, you are checking the ZIL sw here
                    if switch_couples[l]["bus_split"] == sw["bus_split"] # coupling the switch couple to their split bus, if it closed, just connect everything back to the original switch
                        println("SWITCH COUPLE IS $l")

                        switch_t = deepcopy(input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"])
                        switch_f = deepcopy(input_dict["switch"]["$(switch_couples["$l"]["f_sw"])"])
                        
                        if switch_t["t_bus"] == switch_couples[l]["bus_split"]
                            aux_t = switch_t["auxiliary"]
                            orig_t = switch_t["original"]
                            println("Element $aux_t $orig_t")
                            if aux_t == "gen"
                                input_ac_check["gen"]["$(orig_t)"]["gen_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"]) # here it needs to be the bus of the switch
                                delete!(input_ac_check["bus"],input_ac_check["gen"]["$(orig_t)"]["gen_bus"])
                                println("Element $aux_t $orig_t connected to $(input_ac_check["gen"]["$(orig_t)"]["gen_bus"])")
                            elseif aux_t == "load"
                                input_ac_check["load"]["$(orig_t)"]["load_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                delete!(input_ac_check["bus"],input_ac_check["load"]["$(orig_t)"]["load_bus"])
                                println("Element $aux_t $orig_t connected to $(input_ac_check["load"]["$(orig_t)"]["load_bus"])")
                            elseif aux_t == "convdc"
                                input_ac_check["convdc"]["$(orig_t)"]["busac_i"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                delete!(input_ac_check["bus"],input_ac_check["convdc"]["$(orig_t)"]["busac_i"])
                                println("Element $aux_t $orig_t connected to $(input_ac_check["convdc"]["$(orig_t)"]["busac_i"])")
                            elseif aux_t == "branch" 
                                if input_ac_check["branch"]["$(orig_t)"]["f_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"]["bus_split"] == switch_couples[l]["bus_split"] # useful if both ends of a branch are busbars being split
                                    input_ac_check["branch"]["$(orig_t)"]["f_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_t)"]["f_bus"])
                                    println("Element $aux_t $orig_t connected to $(input_ac_check["branch"]["$(orig_t)"]["f_bus"])")
                                elseif input_ac_check["branch"]["$(orig_t)"]["t_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"]["bus_split"] == switch_couples[l]["bus_split"]
                                    input_ac_check["branch"]["$(orig_t)"]["t_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_t)"]["t_bus"])
                                    println("Element $aux_t $orig_t connected to $(input_ac_check["branch"]["$(orig_t)"]["t_bus"])")
                                end
                            end
                        elseif switch_f["t_bus"] == switch_couples[l]["bus_split"]
                            aux_f = switch_f["auxiliary"]
                            orig_f = switch_f["original"]
                            println("Element $aux_f $orig_f")
                            if aux_f == "gen"
                                input_ac_check["gen"]["$(orig_f)"]["gen_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"]) # here it needs to be the bus of the switch
                                delete!(input_ac_check["bus"],input_ac_check["gen"]["$(orig_f)"]["gen_bus"])
                                println("Element $aux_f $orig_f connected to $(input_ac_check["gen"]["$(orig_f)"]["gen_bus"])")
                            elseif aux_f == "load"
                                input_ac_check["load"]["$(orig_f)"]["load_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                delete!(input_ac_check["bus"],input_ac_check["load"]["$(orig_f)"]["load_bus"])
                                println("Element $aux_f $orig_f connected to $(input_ac_check["load"]["$(orig_f)"]["load_bus"])")
                            elseif aux_f == "convdc"
                                input_ac_check["convdc"]["$(orig_f)"]["busac_i"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                delete!(input_ac_check["bus"],input_ac_check["convdc"]["$(orig_f)"]["busac_i"])
                                println("Element $aux_f $orig_f connected to $(input_ac_check["convdc"]["$(orig_f)"]["busac_i"])")
                            elseif aux_f == "branch" 
                                if input_ac_check["branch"]["$(orig_f)"]["f_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["f_sw"])"]["bus_split"] == switch_couples[l]["bus_split"] # useful if both ends of a branch are busbars being split
                                    input_ac_check["branch"]["$(orig_f)"]["f_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_f)"]["f_bus"])
                                    println("Element $aux_f $orig_f connected to $(input_ac_check["branch"]["$(orig_f)"]["f_bus"])")
                                elseif input_ac_check["branch"]["$(orig_f)"]["t_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["f_sw"])"]["bus_split"] == switch_couples[l]["bus_split"]
                                    input_ac_check["branch"]["$(orig_f)"]["t_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_f)"]["t_bus"])
                                    println("Element $aux_f $orig_f connected to $(input_ac_check["branch"]["$(orig_f)"]["t_bus"])")
                                end
                            end
                        end
                    end
                end
            elseif result_dict["solution"]["switch"][sw_id]["status"] <= 0.1 # Connect elements to the right bus
                println("Switch $sw_id is open, busbar splitting on bus $(sw["bus_split"])")
                delete!(input_ac_check["switch"],sw_id)
                for l in keys(switch_couples)
                    #if haskey(switch_couples,sw_id) && switch_couples[l]["bus_split"] == sw["bus_split"] -> wrong, you are checking the ZIL sw here
                    if switch_couples[l]["bus_split"] == sw["bus_split"] # coupling the switch couple to their split bus
                        println("SWITCH COUPLE IS $l")
                            switch_t = deepcopy(input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"]) 
                            aux_t = switch_t["auxiliary"]
                            orig_t = switch_t["original"]
                            println("Element $aux_t $orig_t")
                            if result_dict["solution"]["switch"]["$(switch_t["index"])"]["status"] <= 0.1
                                println("Deleting switch $(switch_t["index"])")
                                delete!(input_ac_check["switch"],"$(switch_t["index"])")
                            elseif result_dict["solution"]["switch"]["$(switch_t["index"])"]["status"] >= 0.9
                                if aux_t == "gen"
                                    input_ac_check["gen"]["$(orig_t)"]["gen_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"]) # here it needs to be the bus of the switch
                                    delete!(input_ac_check["bus"],input_ac_check["gen"]["$(orig_t)"]["gen_bus"])
                                    println("Element $aux_t $orig_t connected to $(input_ac_check["gen"]["$(orig_t)"]["gen_bus"])")
                                elseif aux_t == "load"
                                    input_ac_check["load"]["$(orig_t)"]["load_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["load"]["$(orig_t)"]["load_bus"])
                                    println("Element $aux_t $orig_t connected to $(input_ac_check["load"]["$(orig_t)"]["load_bus"])")
                                elseif aux_t == "convdc"
                                    input_ac_check["convdc"]["$(orig_t)"]["busac_i"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["convdc"]["$(orig_t)"]["busac_i"])
                                    println("Element $aux_t $orig_t connected to $(input_ac_check["convdc"]["$(orig_t)"]["busac_i"])")
                                elseif aux_t == "branch" 
                                    if input_ac_check["branch"]["$(orig_t)"]["f_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"]["bus_split"] == switch_couples[l]["bus_split"] # useful if both ends of a branch are busbars being split
                                        input_ac_check["branch"]["$(orig_t)"]["f_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                        delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_t)"]["f_bus"])
                                        println("Element $aux_t $orig_t connected to $(input_ac_check["branch"]["$(orig_t)"]["f_bus"])")
                                    elseif input_ac_check["branch"]["$(orig_t)"]["t_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"]["bus_split"] == switch_couples[l]["bus_split"]
                                        input_ac_check["branch"]["$(orig_t)"]["t_bus"] = deepcopy(input_dict["switch"]["$(switch_t["index"])"]["t_bus"])
                                        delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_t)"]["t_bus"])
                                        println("Element $aux_t $orig_t connected to $(input_ac_check["branch"]["$(orig_t)"]["t_bus"])")
                                    end
                                end
                            end
                        

                            switch_f = deepcopy(input_dict["switch"]["$(switch_couples["$l"]["f_sw"])"]) 
                            aux_f = switch_f["auxiliary"]
                            orig_f = switch_f["original"]
                            println("Element $aux_f $orig_f")
                            if result_dict["solution"]["switch"]["$(switch_f["index"])"]["status"] <= 0.1
                                println("Deleting switch $(switch_f["index"])")
                                delete!(input_ac_check["switch"],"$(switch_f["index"])")
                            elseif result_dict["solution"]["switch"]["$(switch_f["index"])"]["status"] >= 0.9
                                if aux_f == "gen"
                                    input_ac_check["gen"]["$(orig_f)"]["gen_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"]) # here it needs to be the bus of the switch
                                    delete!(input_ac_check["bus"],input_ac_check["gen"]["$(orig_f)"]["gen_bus"])
                                    println("Element $aux_f $orig_f connected to $(input_ac_check["gen"]["$(orig_f)"]["gen_bus"])")
                                elseif aux_f == "load"
                                    input_ac_check["load"]["$(orig_f)"]["load_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["load"]["$(orig_f)"]["load_bus"])
                                    println("Element $aux_f $orig_f connected to $(input_ac_check["load"]["$(orig_f)"]["load_bus"])")
                                elseif aux_f == "convdc"
                                    input_ac_check["convdc"]["$(orig_f)"]["busac_i"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                    delete!(input_ac_check["bus"],input_ac_check["convdc"]["$(orig_f)"]["busac_i"])
                                    println("Element $aux_f $orig_f connected to $(input_ac_check["convdc"]["$(orig_f)"]["busac_i"])")
                                elseif aux_f == "branch" 
                                    if input_ac_check["branch"]["$(orig_f)"]["f_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["f_sw"])"]["bus_split"] == switch_couples[l]["bus_split"] # useful if both ends of a branch are busbars being split
                                        input_ac_check["branch"]["$(orig_f)"]["f_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                        delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_f)"]["f_bus"])
                                        println("Element $aux_f $orig_f connected to $(input_ac_check["branch"]["$(orig_f)"]["f_bus"])")
                                    elseif input_ac_check["branch"]["$(orig_f)"]["t_bus"] > orig_buses && input_dict["switch"]["$(switch_couples["$l"]["f_sw"])"]["bus_split"] == switch_couples[l]["bus_split"]
                                        input_ac_check["branch"]["$(orig_f)"]["t_bus"] = deepcopy(input_dict["switch"]["$(switch_f["index"])"]["t_bus"])
                                        delete!(input_ac_check["bus"],input_ac_check["branch"]["$(orig_f)"]["t_bus"])
                                        println("Element $aux_f $orig_f connected to $(input_ac_check["branch"]["$(orig_f)"]["t_bus"])")
                                    end
                                end
                            end
                    end
                end
            end
            input_ac_check["switch"] = Dict{String,Any}()
            input_ac_check["switch_couples"] = Dict{String,Any}()
        end
    end
end

function split_one_bus_per_time(test_case,results_dict,results_dict_ac_check,results_dict_lpac_check)
    for (b_id,b) in test_case["bus"]
        results_dict["$b_id"] = Dict{String,Any}()
        results_dict_ac_check["$b_id"] = Dict{String,Any}()
        test_case_bs = deepcopy(test_case)
        splitted_bus_ac = parse(Int64,b_id)
        test_case_bs,  switches_couples_ac,  extremes_ZILs_ac  = _PMTP.AC_busbar_split_more_buses(test_case_bs,splitted_bus_ac)
        test_case_bs["switch"]["1"]["cost"] = 10.0
        results_dict["$b_id"] = _PMTP.run_acdcsw_AC_big_M(test_case_bs,LPACCPowerModel,gurobi)
        test_case_bs_check = deepcopy(test_case_bs)
        test_case_bs_check_auxiliary = deepcopy(test_case_bs)
        if results_dict["$b_id"]["termination_status"] == JuMP.OPTIMAL
            prepare_AC_feasibility_check(results_dict["$b_id"],test_case_bs_check_auxiliary,test_case_bs_check,switches_couples_ac,extremes_ZILs_ac,test_case)
            results_dict_ac_check["$b_id"] = _PMACDC.run_acdcopf(test_case_bs_check,ACPPowerModel,ipopt; setting = s)
            results_dict_lpac_check["$b_id"] = _PMACDC.run_acdcopf(test_case_bs_check,LPACCPowerModel,gurobi; setting = s)
        end
    end
end
@time split_one_bus_per_time(data_5_acdc,result_bs,result_bs_ac_check,result_bs_lpac_check)

obj_bs = []
for (b_id,b) in data_5_acdc["bus"]
    push!(obj_bs,[b_id,result_bs["$b_id"]["objective"]])
end
obj_bs
sort(obj_bs, by = x -> x[2])
result_bs["43"]

obj_bs_lpac = []
for (b_id,b) in data_5_acdc["bus"]
    if haskey(result_bs_lpac_check["$b_id"],"objective")
        push!(obj_bs_lpac,[b_id,result_bs_lpac_check["$b_id"]["objective"]])
    end
end
result_bs_lpac_check["43"]
result_bs_lpac_check["2"]["objective"]/result_opf_lpac["objective"]

obj_bs = []
for (b_id,b) in data_5_acdc["bus"]
    if haskey(result_bs_ac_check["$b_id"],"objective")
        push!(obj_bs,[b_id,result_bs_ac_check["$b_id"]["objective"]])
    end
end
obj_bs
result_bs_ac_check["43"]

#######################################################################################
## Busbar splitting models ##
#######################################################################################
###### AC Busbar splitting models ######
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_plus = deepcopy(data_5_acdc)

data_busbars_ac_split_5_acdc_more_buses = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 38

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
data_busbars_ac_split_5_acdc_plus,  switches_couples_ac_5_plus,  extremes_ZILs_5_ac_plus  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_plus,splitted_bus_ac)

# Duplicating the network data
ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_ac_ref_Line = deepcopy(data_busbars_ac_split_5_acdc)

ac_bs_ac_ref_plus = deepcopy(data_busbars_ac_split_5_acdc_plus)
ac_bs_ac_ref_Line_plus = deepcopy(data_busbars_ac_split_5_acdc_plus)

ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac_ref_Line = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_dc_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_dc_ref_Line = deepcopy(data_busbars_ac_split_5_acdc)

result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_big_M_ZIL(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_lpac_ac_ref  = _PMTP.run_acdcsw_AC_big_M_ZIL(ac_bs_lpac_ref,LPACCPowerModel,gurobi_opf)
result_switches_soc_ac_ref  = _PMTP.run_acdcsw_AC_big_M_ZIL(ac_bs_ac_ref,SOCWRPowerModel,gurobi)
result_switches_qc_ac_ref  = _PMTP.run_acdcsw_AC_big_M_ZIL(ac_bs_ac_ref,QCRMPowerModel,gurobi)

#result_switches_AC_ac_ref_not_Line  = _PMTP.run_acdcsw_AC_big_M(ac_bs_ac_ref_Line,ACPPowerModel,juniper)


# Run models
# Feasibility check for the AC busbar splitting
feasibility_check_AC_BS_opf_ac_ref_status = deepcopy(data_busbars_ac_split_5_acdc)
prepare_AC_feasibility_check(result_switches_lpac_ac_ref,data_busbars_ac_split_5_acdc,feasibility_check_AC_BS_opf_ac_ref_status,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)
result_feasibility_check = _PMACDC.run_acdcopf(feasibility_check_AC_BS_opf_ac_ref_status,ACPPowerModel,ipopt; setting = s)

feasibility_check_AC_BS_opf_lpac_ref_status = deepcopy(data_busbars_ac_split_5_acdc)
prepare_AC_feasibility_check(result_switches_lpac_ac_ref,data_busbars_ac_split_5_acdc,feasibility_check_AC_BS_opf_lpac_ref_status,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)
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

