function run_hourly_redispatch(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:length(grid["nw"])
        result_feasibility_checks["$hour"] = Dict{String,Any}()
        feasibility_check = deepcopy(grid["nw"]["$hour"])
        feasibility_check_input = deepcopy(grid["nw"]["$hour"])
        _PMTP.prepare_AC_feasibility_check(result_bs["$hour"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)

        # Adding set points
        for (g_id,g) in feasibility_check["gen"]
            g["pg_start"] = result_bs["$hour"]["solution"]["gen"][g_id]["pg"]
            g["qg_start"] = result_bs["$hour"]["solution"]["gen"][g_id]["qg"]
            if length(g["cost"]) > 0
                g["redispatch_cost_up"] = g["cost"][1]
                g["redispatch_cost_down"] = g["cost"][1]
            else
                g["redispatch_cost_up"] = 0.0
                g["redispatch_cost_down"] = 0.0
            end
        end
        result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
    end
    return result_feasibility_checks
end

function run_hourly_redispatch_one_topology(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:length(grid["nw"])
        result_feasibility_checks["$hour"] = Dict{String,Any}()
        feasibility_check = deepcopy(grid["nw"]["$hour"])
        feasibility_check_input = deepcopy(grid["nw"]["$hour"])
        _SPMTA.prepare_AC_feasibility_check_stochastic(result_bs["solution"]["nw"]["$hour"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)

        for (g_id,g) in feasibility_check["gen"]
            g["pg_start"] = result_bs["solution"]["nw"]["$hour"]["gen"][g_id]["pg"]
            g["qg_start"] = result_bs["solution"]["nw"]["$hour"]["gen"][g_id]["qg"]

            if length(g["cost"]) > 0
                g["redispatch_cost_up"] = g["cost"][1]
                g["redispatch_cost_down"] = g["cost"][1]
            else
                g["redispatch_cost_up"] = 0.0
                g["redispatch_cost_down"] = 0.0
            end
        end

        result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
    end
    return result_feasibility_checks
end

function prepare_AC_feasibility_check(result_dict, input_dict, input_ac_check, switch_couples, extremes_dict,input_base)
    orig_buses = maximum(parse.(Int, keys(input_base["bus"]))) # maximum value before splitting (in case the buses are not in numerical order)
    for (sw_id,sw) in input_dict["switch"]
        if !haskey(sw,"auxiliary")
            println("SWITCH $sw_id, BUS from $(sw["f_bus"]), BUS to $(sw["t_bus"])")
            if haskey(result_dict["solution"],"switch")
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
                elseif result_dict["solution"]["switch"][sw_id]["status"] <= 0.1 # Connect elements to t    he right bus
                println("Switch $sw_id is open, busbar splitting on bus $(sw["bus_split"])")
                delete!(input_ac_check["switch"],sw_id)
                for l in keys(switch_couples)
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
            elseif haskey(result_dict["solution"],"nw")
                if result_dict["solution"]["nw"]["1"]["switch"][sw_id]["status"] >= 0.9 # Just reconnecting stuff
                    println("Switch $sw_id is closed, Connecting everything back, no busbar splitting on bus $(sw["bus_split"])")
                    for l in keys(switch_couples)
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
                elseif result_dict["solution"]["nw"]["1"]["switch"][sw_id]["status"] <= 0.1 # Connect elements to the right bus
                    println("Switch $sw_id is open, busbar splitting on bus $(sw["bus_split"])")
                    delete!(input_ac_check["switch"],sw_id)
                    for l in keys(switch_couples)
                        if switch_couples[l]["bus_split"] == sw["bus_split"] # coupling the switch couple to their split bus
                            println("SWITCH COUPLE IS $l")
                                switch_t = deepcopy(input_dict["switch"]["$(switch_couples["$l"]["t_sw"])"]) 
                                aux_t = switch_t["auxiliary"]
                                orig_t = switch_t["original"]
                                println("Element $aux_t $orig_t")
                                if result_dict["solution"]["nw"]["1"]["switch"]["$(switch_t["index"])"]["status"] <= 0.1
                                    println("Deleting switch $(switch_t["index"])")
                                    delete!(input_ac_check["switch"],"$(switch_t["index"])")
                                elseif result_dict["solution"]["nw"]["1"]["switch"]["$(switch_t["index"])"]["status"] >= 0.9
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
                                if result_dict["solution"]["nw"]["1"]["switch"]["$(switch_f["index"])"]["status"] <= 0.1
                                    println("Deleting switch $(switch_f["index"])")
                                    delete!(input_ac_check["switch"],"$(switch_f["index"])")
                                elseif result_dict["solution"]["nw"]["1"]["switch"]["$(switch_f["index"])"]["status"] >= 0.9
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
end

function run_redispatch_scenarios(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings,n_hours,n_scenarios)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:n_hours
        for i in 1:n_scenarios
            if i == 1
                timestep = (hour - 1)*n_scenarios + i
                result_feasibility_checks["$hour"] = Dict{String,Any}()
                feasibility_check = deepcopy(grid["nw"]["$hour"])
                feasibility_check_input = deepcopy(grid["nw"]["$hour"])
                _SPMTA.prepare_AC_feasibility_check_stochastic(result_bs["solution"]["nw"]["$timestep"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)
                for (g_id,g) in feasibility_check["gen"]
                    g["pg_start"] = result_bs["solution"]["nw"]["$timestep"]["gen"][g_id]["pg"]
                    g["qg_start"] = result_bs["solution"]["nw"]["$timestep"]["gen"][g_id]["qg"]
                
                    if length(g["cost"]) > 0
                        g["redispatch_cost_up"] = g["cost"][1]
                        g["redispatch_cost_down"] = g["cost"][1]
                    else
                        g["redispatch_cost_up"] = 0.0
                        g["redispatch_cost_down"] = 0.0
                    end
                end    
                result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
            end
        end
    end
    return result_feasibility_checks
end

function run_hourly_redispatch_scenarios(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings,n_hours,n_scenarios)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:n_hours
        for i in 1:n_scenarios
            if i == 1
                timestep = (hour - 1)*n_scenarios + i
                result_feasibility_checks["$hour"] = Dict{String,Any}()
                feasibility_check = deepcopy(grid["nw"]["$hour"])
                feasibility_check_input = deepcopy(grid["nw"]["$hour"])
                prepare_AC_feasibility_check(result_bs["$hour"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)
                for (g_id,g) in feasibility_check["gen"]
                    g["pg_start"] = result_bs["$hour"]["solution"]["nw"]["1"]["gen"][g_id]["pg"]
                    g["qg_start"] = result_bs["$hour"]["solution"]["nw"]["1"]["gen"][g_id]["qg"]
                
                    if length(g["cost"]) > 0
                        g["redispatch_cost_up"] = g["cost"][1]
                        g["redispatch_cost_down"] = g["cost"][1]
                    else
                        g["redispatch_cost_up"] = 0.0
                        g["redispatch_cost_down"] = 0.0
                    end
                end
                result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
            end
        end
    end
    return result_feasibility_checks
end

function print_gen_redispatch(grid,results,hours)
    for h in 1:hours
        println("------------------")
        println("Hour $h")
        for (g_id,g) in grid["gen"]
            if abs(results["$h"]["solution"]["gen"][g_id]["pg_up"]) > 0.001 
                println("Generator $g_id, pg_up $(results["$h"]["solution"]["gen"][g_id]["pg_up"]), qg_up $(results["$h"]["solution"]["gen"][g_id]["qg_up"])")
            elseif abs(results["$h"]["solution"]["gen"][g_id]["pg_down"]) > 0.001
                println("Generator $g_id, pg_down $(results["$h"]["solution"]["gen"][g_id]["pg_down"]), qg_down $(results["$h"]["solution"]["gen"][g_id]["qg_down"])")
            end
        end
    end
end