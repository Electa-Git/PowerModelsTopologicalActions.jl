# Function to remove the switches from the optimized grid topology

function prepare_AC_feasibility_check(result_dict, input_dict, input_ac_check, switch_couples, extremes_dict,input_base)    
    orig_buses = maximum(parse.(Int, keys(input_base["bus"]))) + length(extremes_dict)
    #print("Orig buses is $orig_buses","\n")
    for (sw_id,sw) in input_dict["switch"]
        if haskey(sw,"auxiliary") && haskey(sw,"auxiliary") # Make sure ZILs are not included 
            aux =  deepcopy(input_ac_check["switch"][sw_id]["auxiliary"])
            orig = deepcopy(input_ac_check["switch"][sw_id]["original"])  
            for zil in eachindex(extremes_dict)
                if sw["bus_split"] == extremes_dict[zil][1] && result_dict["solution"]["switch"]["$(switch_couples[sw_id]["switch_split"])"]["status"] == 1.0  # Making sure to reconnect everything to the original if the ZIL is connected
                    if result_dict["solution"]["switch"][sw_id]["status"] >= 0.9
                        if aux == "gen"
                            input_ac_check["gen"]["$(orig)"]["gen_bus"] = deepcopy(extremes_dict[zil][1])
                        elseif aux == "load"
                            input_ac_check["load"]["$(orig)"]["load_bus"] = deepcopy(extremes_dict[zil][1])
                        elseif aux == "convdc"
                            input_ac_check["convdc"]["$(orig)"]["busac_i"] = deepcopy(extremes_dict[zil][1])
                        elseif aux == "branch"  
                            #print("Sw_id is $(sw_id), status is $(result_dict["solution"]["switch"][sw_id]["status"]), Aux is $(aux), Orig is $(orig), sw_id is $(sw_id)","\n")                
                            if input_ac_check["branch"]["$(orig)"]["f_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil)
                                    #print("DIO CANE F_BUS $(sw_id)","\n")
                                    input_ac_check["branch"]["$(orig)"]["f_bus"] = deepcopy(switch_couples[sw_id]["bus_split"])
                            elseif input_ac_check["branch"]["$(orig)"]["t_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil)
                                    #print("DIO CANE T_BUS $(sw_id)","\n")
                                    input_ac_check["branch"]["$(orig)"]["t_bus"] = deepcopy(switch_couples[sw_id]["bus_split"])
                            end
                        end
                        delete!(input_ac_check["switch"],sw_id)
                    else
                        delete!(input_ac_check["switch"],sw_id)
                    end
                elseif sw["bus_split"] == extremes_dict[zil][1] && result_dict["solution"]["switch"]["$(switch_couples[sw_id]["switch_split"])"]["status"] == 0.0 # Reconnect everything to the split busbar
                    if result_dict["solution"]["switch"][sw_id]["status"] >= 0.9
                        if aux == "gen"
                            input_ac_check["gen"]["$(orig)"]["gen_bus"] = deepcopy(sw["t_bus"])
                        elseif aux == "load"
                            input_ac_check["load"]["$(orig)"]["load_bus"] = deepcopy(sw["t_bus"])
                        elseif aux == "convdc"
                            input_ac_check["convdc"]["$(orig)"]["busac_i"] = deepcopy(sw["t_bus"])
                        elseif aux == "branch" 
                            #print("ZIL is $(zil), Sw_id is $(sw_id), status is $(result_dict["solution"]["switch"][sw_id]["status"]), Aux is $(aux), Orig is $(orig), sw_id is $(sw_id)","\n")                               
                            if input_ac_check["branch"]["$(orig)"]["f_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil) 
                                    #print("ORIGINAL AC check branch f_bus is $(input_ac_check["branch"]["$(orig)"]["f_bus"])","\n")
                                    input_ac_check["branch"]["$(orig)"]["f_bus"] = deepcopy(sw["t_bus"])
                                    #print("New F BUS IS $(input_ac_check["branch"]["$(orig)"]["f_bus"])","\n")
                                    #print("AC check branch f_bus is $(input_ac_check["branch"]["$(orig)"]["f_bus"])","\n")
                                    #print("-------------------","\n")
                            elseif input_ac_check["branch"]["$(orig)"]["t_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil)
                                if !haskey(input_ac_check["branch"]["$(orig)"],"checked")
                                    #print("ORIGINAL AC check branch t_bus is $(input_ac_check["branch"]["$(orig)"]["t_bus"])","\n")
                                    input_ac_check["branch"]["$(orig)"]["t_bus"] = deepcopy(sw["t_bus"])
                                    #print("T BUS IS $(input_ac_check["branch"]["$(orig)"]["t_bus"])","\n")
                                    #print("AC check branch t_bus is $(input_ac_check["branch"]["$(orig)"]["t_bus"])","\n")
                                    #print("-------------------","\n")
                                end
                            end
                        end
                        delete!(input_ac_check["switch"],sw_id)
                    else
                        delete!(input_ac_check["switch"],sw_id)
                    end
                end
            end
        end
    end
    return input_ac_check

    #for (br_id,br) in input_dict["switch"] # Make sure to include ZILs first  
    #    if !haskey(br,"auxiliary") && !haskey(br,"original")
    #        print("br_id is $br_id","\n")
    #        if result_dict["solution"]["switch"][br_id]["status"] >= 0.9
    #            delete!(input_ac_check["bus"],"$(extremes_dict[br_id][2])")
    #        else
    #            delete!(input_ac_check["switch"],br_id)
    #        end
    #    end
    #end
end

function prepare_AC_grid_feasibility_check(result_dict, input_dict, input_ac_check, switch_couples, extremes_dict,input_base)    
    orig_buses = maximum(parse.(Int, keys(input_base["bus"]))) + length(extremes_dict)
    for (sw_id,sw) in input_dict["switch"]
        if haskey(sw,"auxiliary") # Make sure ZILs are not included 
            println("sw is $(sw_id)")
            aux =  deepcopy(input_ac_check["switch"][sw_id]["auxiliary"])
            orig = deepcopy(input_ac_check["switch"][sw_id]["original"])  
            for zil in eachindex(extremes_dict)
                println("ZIL is $(zil)")
                if sw["bus_split"] == extremes_dict[zil][1] && result_dict["solution"]["switch"]["$(switch_couples[sw_id]["switch_split"])"]["status"] == 1.0  # Making sure to reconnect everything to the original if the ZIL is connected
                    if result_dict["solution"]["switch"][sw_id]["status"] >= 0.9
                        if aux == "gen"
                            input_ac_check["gen"]["$(orig)"]["gen_bus"] = deepcopy(extremes_dict[zil][1])
                        elseif aux == "load"
                            input_ac_check["load"]["$(orig)"]["load_bus"] = deepcopy(extremes_dict[zil][1])
                        elseif aux == "branch"  
                            if input_ac_check["branch"]["$(orig)"]["f_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil)
                                input_ac_check["branch"]["$(orig)"]["f_bus"] = deepcopy(switch_couples[sw_id]["bus_split"])
                            elseif input_ac_check["branch"]["$(orig)"]["t_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil)
                                input_ac_check["branch"]["$(orig)"]["t_bus"] = deepcopy(switch_couples[sw_id]["bus_split"])
                            end
                        end
                        delete!(input_ac_check["switch"],sw_id)
                    else
                        delete!(input_ac_check["switch"],sw_id)
                    end
                elseif sw["bus_split"] == extremes_dict[zil][1] && result_dict["solution"]["switch"]["$(switch_couples[sw_id]["switch_split"])"]["status"] == 0.0 # Reconnect everything to the split busbar
                    if result_dict["solution"]["switch"][sw_id]["status"] >= 0.9
                        if aux == "gen"
                            input_ac_check["gen"]["$(orig)"]["gen_bus"] = deepcopy(sw["t_bus"])
                        elseif aux == "load"
                            input_ac_check["load"]["$(orig)"]["load_bus"] = deepcopy(sw["t_bus"])
                        elseif aux == "branch" 
                            if input_ac_check["branch"]["$(orig)"]["f_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil) 
                                    input_ac_check["branch"]["$(orig)"]["f_bus"] = deepcopy(sw["t_bus"])
                            elseif input_ac_check["branch"]["$(orig)"]["t_bus"] > orig_buses && switch_couples[sw_id]["bus_split"] == parse(Int64,zil)
                                if !haskey(input_ac_check["branch"]["$(orig)"],"checked")
                                    input_ac_check["branch"]["$(orig)"]["t_bus"] = deepcopy(sw["t_bus"])
                                end
                            end
                        end
                        delete!(input_ac_check["switch"],sw_id)
                    else
                        delete!(input_ac_check["switch"],sw_id)
                    end
                end
            end
        end
    end
    return input_ac_check
end
