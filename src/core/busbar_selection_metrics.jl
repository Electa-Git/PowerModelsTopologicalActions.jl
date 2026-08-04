function split_one_bus_per_time(test_case,sw_cost,formulation_BuS,formulation_i,formulation_ii,optimizer_BuS,optimizer_i,optimizer_ii,settings)
    results_dict = Dict{String,Any}()
    for (b_id,b) in test_case["bus"]
        results_dict["$b_id"] = Dict{String,Any}()
        test_case_bs = deepcopy(test_case)
        splitted_bus_ac = parse(Int64,b_id)
        test_case_bs,  switches_couples_ac,  extremes_ZILs_ac  = AC_busbars_split(test_case_bs,splitted_bus_ac)
        test_case_bs["switch"]["1"]["cost"] = sw_cost
        results_dict["$b_id"]["BuS"] = Dict{String,Any}()
        results_dict["$b_id"]["AC_FC"] = Dict{String,Any}()
        results_dict["$b_id"]["LPAC_FC"] = Dict{String,Any}()

        results_dict["$b_id"]["BuS"] = run_acdc_BuS_AC(test_case_bs,formulation_BuS,optimizer_BuS)
        test_case_bs_check = deepcopy(test_case_bs)
        test_case_bs_check_auxiliary = deepcopy(test_case_bs)
        if results_dict["$b_id"]["BuS"]["termination_status"] == JuMP.OPTIMAL || results_dict["$b_id"]["BuS"]["termination_status"] == JuMP.LOCALLY_SOLVED
            prepare_AC_feasibility_check_AC_busbars(results_dict["$b_id"]["BuS"],test_case_bs_check_auxiliary,test_case_bs_check,switches_couples_ac,extremes_ZILs_ac,test_case)
            results_dict["$b_id"]["AC_FC"] = deepcopy(_PMACDC.solve_acdcopf(test_case_bs_check,formulation_i,optimizer_i; setting = settings))
            results_dict["$b_id"]["LPAC_FC"] = deepcopy(_PMACDC.solve_acdcopf(test_case_bs_check,formulation_ii,optimizer_ii; setting = settings))
        end
    end
    return results_dict
end

function compute_metrics_per_bus(test_case,formulation,optimizer,settings)
    dict = Dict{String,Any}()
    result_opf = _PMACDC.solve_acdcopf(test_case,formulation,optimizer; setting = settings)
    for (b_id,b) in test_case["bus"]
        dict["$b_id"] = Dict{String,Any}()
        dict["$b_id"]["lam_kcl_r"] = abs(result_opf["solution"]["bus"][b_id]["lam_kcl_r"])
        dict["$b_id"]["diff"] = 0.0
        dict["$b_id"]["n_branches"] = 0
        dict["$b_id"]["connected_branches"] = Dict{String,Any}()
    end
    for (b_id,b) in test_case["branch"]
        f_bus = b["f_bus"]
        t_bus = b["t_bus"]
        dict["$f_bus"]["connected_branches"]["$b_id"] = Dict{String,Any}()
        dict["$t_bus"]["connected_branches"]["$b_id"] = Dict{String,Any}()
        dict["$f_bus"]["connected_branches"]["$b_id"]["utilization"] = abs(result_opf["solution"]["branch"][b_id]["pf"])/ b["rate_a"]
        dict["$t_bus"]["connected_branches"]["$b_id"]["utilization"] = abs(result_opf["solution"]["branch"][b_id]["pf"])/ b["rate_a"]
        dict["$f_bus"]["connected_branches"]["$b_id"]["va_fr"] = result_opf["solution"]["bus"]["$(f_bus)"]["va"]
        dict["$t_bus"]["connected_branches"]["$b_id"]["va_fr"] = result_opf["solution"]["bus"]["$(f_bus)"]["va"]
        dict["$f_bus"]["connected_branches"]["$b_id"]["va_to"] = result_opf["solution"]["bus"]["$(t_bus)"]["va"]
        dict["$t_bus"]["connected_branches"]["$b_id"]["va_to"] = result_opf["solution"]["bus"]["$(t_bus)"]["va"]
        dict["$f_bus"]["connected_branches"]["$b_id"]["va_fr_minus_va_to"] = result_opf["solution"]["bus"]["$(f_bus)"]["va"] - result_opf["solution"]["bus"]["$(t_bus)"]["va"]
        dict["$t_bus"]["connected_branches"]["$b_id"]["va_fr_minus_va_to"] = result_opf["solution"]["bus"]["$(f_bus)"]["va"] - result_opf["solution"]["bus"]["$(t_bus)"]["va"]

        diff = abs(dict["$f_bus"]["lam_kcl_r"] - dict["$t_bus"]["lam_kcl_r"])
        dict["$f_bus"]["diff"] += diff
        dict["$t_bus"]["diff"] += diff
        dict["$f_bus"]["n_branches"] += 1
        dict["$t_bus"]["n_branches"] += 1
    end
    return dict
end
