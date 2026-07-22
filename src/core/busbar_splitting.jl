function AC_busbars_split(data_original,bus_to_be_split) 
    data = deepcopy(data_original)
    min_bus_original = minimum([bus["index"] for (b, bus) in data["bus"]]) 
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in keys(data["bus"])
            if parse(Int64,i) != bus_to_be_split
                data["bus"]["$i"]["split"] = false
            end
        end
        data["bus"]["$bus_to_be_split"]["split"] = true
        data["bus"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["bus"])
            #if parse(Int64,i) != n #&& !haskey(data["bus"][i],"ZIL")
                data["bus"]["$i"]["split"] = false
                data["bus"]["$i"]["ZIL"] = false
            #end
        end
        for n in bus_to_be_split # selecting only split buses
            data["bus"]["$n"]["split"] = true
            data["bus"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original = maximum([bus["index"] for (b, bus) in data["bus"]]) 
    count_ = 0
    for (b_id,b) in data["bus"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original # make sure we include only the original buses
            count_ += 1
            b["bus_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            added_bus = n_buses_original + count_
            data["bus"]["$added_bus"] = deepcopy(b)
            data["bus"]["$added_bus"]["bus_type"] = 1
            data["bus"]["$added_bus"]["bus_i"] = added_bus 
            data["bus"]["$added_bus"]["source_id"][2] = added_bus 
            data["bus"]["$added_bus"]["index"] = added_bus
            data["bus"]["$added_bus"]["split"] = false
            data["bus"]["$added_bus"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if b["split"] == true && !haskey(extremes_ZIL,b["bus_split"]) # isolating only the original buses being split
            extremes_ZIL["$(b["bus_split"])"] = []
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] <= n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] > n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses, assuming there are no switches already existing in the test case. Using PowerModels format
    switch_id = 0
    for i in eachindex(extremes_ZIL)
        switch_id += 1
        data["switch"]["$switch_id"] = Dict{String,Any}()
        data["switch"]["$switch_id"]["bus_split"] = deepcopy(extremes_ZIL[i][1]) # the original bus being split is always the first elements in the vector of each key of extremes_ZIL
        data["switch"]["$switch_id"]["f_bus"] = deepcopy(extremes_ZIL[i][1]) # assigning from and to bus to each switch. One switch for each key in the extremes_ZIL
        data["switch"]["$switch_id"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
        data["switch"]["$switch_id"]["index"] = switch_id
        data["switch"]["$switch_id"]["psw"] = 100.0 # assuming a maximum active power for the switch
        data["switch"]["$switch_id"]["qsw"] = 100.0 # assuming a maximum reactive power for the switch
        data["switch"]["$switch_id"]["thermal_rating"] = 100.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        data["switch"]["$switch_id"]["cost"] = 1.0
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
        data["switch"]["$switch_id"]["ZIL"] = true
    end
     
    # Add a bus for each grid element connected to the bus being split
    # Gen
    for (g_id,g) in data["gen"] 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if g["gen_bus"] == parse(Int64,i) # isolating the generators connected to the split busbar. Adding new buses in PowerModels format
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_gen_bus"]["original"] = deepcopy(parse(Int64,g_id)) # element to indicate the original number of the grid elements before the split
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen" # type of grid element linked to the busbar being split
                data["bus"]["$added_gen_bus"]["split"] = false # indicating that the bus is not created because of the busbar split. It is an auxiliary bus for the grid element connected to the busbar being split
                data["bus"]["$added_gen_bus"]["bus_split"] = deepcopy(parse(Int64,i)) # number of the bus to which the grid element was originally attached to
                g["gen_bus"] = added_gen_bus # updating the number of the generator. The original number is indicated by the "original" element
            end
        end
    end 
    
    # Load -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    for (l_id,l) in data["load"] 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_load_bus"]["original"] = deepcopy(parse(Int64,l_id))  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    for (br_id,br) in data["branch"] 
        for i in eachindex(extremes_ZIL)
            #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, f_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id))
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                if haskey(data["bus"]["$added_branch_bus"],"split")
                    delete!(data["bus"]["$added_branch_bus"],"split")
                end
                br["f_bus"] = added_branch_bus
            elseif br["t_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, t_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id)) 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_branch_bus"]["split"] = false
                br["t_bus"] = added_branch_bus
            end
        end
    end
    
    # Converters -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
            for i in eachindex(extremes_ZIL)
                n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
                if cv["busac_i"] == parse(Int64,i)
                    added_conv_bus = n_buses + 1
                    data["bus"]["$added_conv_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                    data["bus"]["$added_conv_bus"]["bus_type"] = 1
                    data["bus"]["$added_conv_bus"]["bus_i"] = added_conv_bus 
                    data["bus"]["$added_conv_bus"]["index"] = added_conv_bus 
                    data["bus"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                    data["bus"]["$added_conv_bus"]["auxiliary_bus"] = true 
                    data["bus"]["$added_conv_bus"]["original"] = deepcopy(parse(Int64,cv_id)) 
                    data["bus"]["$added_conv_bus"]["auxiliary"] = "convdc"
                    data["bus"]["$added_conv_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                    data["bus"]["$added_conv_bus"]["split"] = false
                    cv["busac_i"] = added_conv_bus
                end
            end
        end
    end
    
    
    # Linking the auxiliary buses to both split buses with switches. Two switches for each new auxiliary bus that was created before (close to reality representation)
    for (b_id,b) in data["bus"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    number_switches = length(data["switch"])
                    #first switch 
                    added_switch_1 = number_switches + 1
                    data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                    data["switch"]["$added_switch_1"]["cost"] = 0.0
                    data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                    data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                    data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                    data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                    #data["switch"]["$added_switch_1"]["ZIL"] = false

                    #second switch
                    added_switch_2 = added_switch_1 + 1
                    data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                    data["switch"]["$added_switch_2"]["cost"] = 0.0
                    data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                    data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                    data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                    data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                    #data["switch"]["$switch_id"]["ZIL"] = false
                end
            end
        end
    end
    # The total number of switches is sum forall b in B of (2∗nb +1) where B is the number of substations being split and nb is the number of grid elements connected to each substation
    switch_couples = compute_couples_of_switches(data) # using the function to check the couples of switches linking each grid element to both parts of the split busbar  
    data["switch_couples"] = Dict{String,Any}()
    data["switch_couples"] = deepcopy(switch_couples)
    data["dcswitch_couples"] = Dict{String,Any}()

    return data, switch_couples, extremes_ZIL
end

function AC_busbars_split_ordered(data,bus_to_be_split) 
    min_bus_original = minimum([bus["index"] for (b, bus) in data["bus"]]) 
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in keys(data["bus"])
            if parse(Int64,i) != bus_to_be_split
                data["bus"]["$i"]["split"] = false
            end
        end
        data["bus"]["$bus_to_be_split"]["split"] = true
        data["bus"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["bus"])
            #if parse(Int64,i) != n #&& !haskey(data["bus"][i],"ZIL")
                data["bus"]["$i"]["split"] = false
                data["bus"]["$i"]["ZIL"] = false
            #end
        end
        for n in bus_to_be_split # selecting only split buses
            data["bus"]["$n"]["split"] = true
            data["bus"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original = maximum([bus["index"] for (b, bus) in data["bus"]]) 
    count_ = 0
    n_buses = [parse(Int64,b_id) for (b_id,b) in data["bus"]] # make sure we include only the original buses being split
    for b_id in 1:length(data["bus"])
        b = data["bus"]["$(n_buses[b_id])"]
        if b["split"] == true && n_buses[b_id] <= n_buses_original # make sure we include only the original buses
            count_ += 1
            b["bus_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            added_bus = n_buses_original + count_
            data["bus"]["$added_bus"] = deepcopy(b)
            data["bus"]["$added_bus"]["bus_type"] = 1
            data["bus"]["$added_bus"]["bus_i"] = added_bus 
            data["bus"]["$added_bus"]["source_id"][2] = added_bus 
            data["bus"]["$added_bus"]["index"] = added_bus
            data["bus"]["$added_bus"]["split"] = false
            data["bus"]["$added_bus"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if b["split"] == true && !haskey(extremes_ZIL,b["bus_split"]) # isolating only the original buses being split
            extremes_ZIL["$(b["bus_split"])"] = []
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] <= n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] > n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses, assuming there are no switches already existing in the test case. Using PowerModels format
    switch_id = 0
    for i in eachindex(extremes_ZIL)
        switch_id += 1
        println("Adding bus $switch_id")
        data["switch"]["$switch_id"] = Dict{String,Any}()
        data["switch"]["$switch_id"]["bus_split"] = deepcopy(extremes_ZIL[i][1]) # the original bus being split is always the first elements in the vector of each key of extremes_ZIL
        data["switch"]["$switch_id"]["f_bus"] = deepcopy(extremes_ZIL[i][1]) # assigning from and to bus to each switch. One switch for each key in the extremes_ZIL
        data["switch"]["$switch_id"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
        data["switch"]["$switch_id"]["index"] = switch_id
        data["switch"]["$switch_id"]["psw"] = 100.0 # assuming a maximum active power for the switch
        data["switch"]["$switch_id"]["qsw"] = 100.0 # assuming a maximum reactive power for the switch
        data["switch"]["$switch_id"]["thermal_rating"] = 100.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        data["switch"]["$switch_id"]["cost"] = 1.0
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
        data["switch"]["$switch_id"]["ZIL"] = true
    end
     
    # Add a bus for each grid element connected to the bus being split
    # Gen

    n_gens = [parse(Int64,g_id) for (g_id,g) in data["gen"]] # make sure we include only the original buses being split
    for g_id in 1:length(data["gen"]) 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        g = data["gen"]["$(n_gens[g_id])"]
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if g["gen_bus"] == parse(Int64,i) # isolating the generators connected to the split busbar. Adding new buses in PowerModels format
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_gen_bus"]["original"] = deepcopy(n_gens[g_id]) # element to indicate the original number of the grid elements before the split
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen" # type of grid element linked to the busbar being split
                data["bus"]["$added_gen_bus"]["split"] = false # indicating that the bus is not created because of the busbar split. It is an auxiliary bus for the grid element connected to the busbar being split
                data["bus"]["$added_gen_bus"]["bus_split"] = deepcopy(parse(Int64,i)) # number of the bus to which the grid element was originally attached to
                g["gen_bus"] = added_gen_bus # updating the number of the generator. The original number is indicated by the "original" element
            end
        end
    end 
    
    # Load -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    n_loads = [parse(Int64,l_id) for (l_id,l) in data["load"]] # make sure we include only the original buses being split
    for l_id in 1:length(data["load"])
        l = data["load"]["$(n_loads[l_id])"]
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_load_bus"]["original"] = deepcopy(n_loads[l_id])  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    n_branches = [parse(Int64,br_id) for (br_id,br) in data["branch"]] # make sure we include only the original buses being split
    for br_id in 1:length(data["branch"])
        br = data["branch"]["$(n_branches[br_id])"]
        for i in eachindex(extremes_ZIL)
            #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, f_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(n_branches[br_id])
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                if haskey(data["bus"]["$added_branch_bus"],"split")
                    delete!(data["bus"]["$added_branch_bus"],"split")
                end
                br["f_bus"] = added_branch_bus
            elseif br["t_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, t_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(n_branches[br_id]) 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_branch_bus"]["split"] = false
                br["t_bus"] = added_branch_bus
            end
        end
    end
    
    # Converters -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    if haskey(data,"convdc")
        n_convs = [parse(Int64,cv_id) for (cv_id,cv) in data["convdc"]] # make sure we include only the original buses being split
        for cv_id in 1:length(data["convdc"])
            cv = data["convdc"]["$(n_convs[cv_id])"]
            for i in eachindex(extremes_ZIL)
                n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
                if cv["busac_i"] == parse(Int64,i)
                    added_conv_bus = n_buses + 1
                    data["bus"]["$added_conv_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                    data["bus"]["$added_conv_bus"]["bus_type"] = 1
                    data["bus"]["$added_conv_bus"]["bus_i"] = added_conv_bus 
                    data["bus"]["$added_conv_bus"]["index"] = added_conv_bus 
                    data["bus"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                    data["bus"]["$added_conv_bus"]["auxiliary_bus"] = true 
                    data["bus"]["$added_conv_bus"]["original"] = deepcopy(n_convs[cv_id]) 
                    data["bus"]["$added_conv_bus"]["auxiliary"] = "convdc"
                    data["bus"]["$added_conv_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                    data["bus"]["$added_conv_bus"]["split"] = false
                    cv["busac_i"] = added_conv_bus
                end
            end
        end
    end
    
    n_buses_expanded = [parse(Int64,b_id) for (b_id,b) in data["bus"]] # make sure we include only the original buses being split

    # Linking the auxiliary buses to both split buses with switches. Two switches for each new auxiliary bus that was created before (close to reality representation)
    for b_id in 1:length(data["bus"])
        b = data["bus"]["$(n_buses_expanded[b_id])"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            println("Checking bus $(n_buses_expanded[b_id]), position $(b_id)")
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    if b["auxiliary"] == "gen"
                        number_switches = length(data["switch"])
                        #first switch 
                        added_switch_1 = number_switches + 1
                        println("Adding switch $added_switch_1")
                        data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_1"]["cost"] = 0.0
                        data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                        data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                        data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                        data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$added_switch_1"]["ZIL"] = false

                        #second switch
                        added_switch_2 = added_switch_1 + 1
                        println("Adding switch $added_switch_2")
                        data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_2"]["cost"] = 0.0
                        data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                        data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                        data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                        data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$switch_id"]["ZIL"] = false
                    end
                end
            end
        end
    end

    for b_id in 1:length(data["bus"])
        b = data["bus"]["$(n_buses_expanded[b_id])"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            println("Checking bus $(n_buses_expanded[b_id]), position $(b_id)")
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    if b["auxiliary"] == "load"
                        number_switches = length(data["switch"])
                        #first switch 
                        added_switch_1 = number_switches + 1
                        println("Adding switch $added_switch_1")
                        data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_1"]["cost"] = 0.0
                        data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                        data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                        data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                        data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$added_switch_1"]["ZIL"] = false

                        #second switch
                        added_switch_2 = added_switch_1 + 1
                        println("Adding switch $added_switch_2")
                        data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_2"]["cost"] = 0.0
                        data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                        data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                        data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                        data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$switch_id"]["ZIL"] = false
                    end
                end
            end
        end
    end

    for b_id in 1:length(data["bus"])
        b = data["bus"]["$(n_buses_expanded[b_id])"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            println("Checking bus $(n_buses_expanded[b_id]), position $(b_id)")
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    if b["auxiliary"] == "branch"
                        number_switches = length(data["switch"])
                        #first switch 
                        added_switch_1 = number_switches + 1
                        println("Adding switch $added_switch_1")
                        data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_1"]["cost"] = 0.0
                        data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                        data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                        data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                        data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$added_switch_1"]["ZIL"] = false

                        #second switch
                        added_switch_2 = added_switch_1 + 1
                        println("Adding switch $added_switch_2")
                        data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_2"]["cost"] = 0.0
                        data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                        data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                        data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                        data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$switch_id"]["ZIL"] = false
                    end
                end
            end
        end
    end

    for b_id in 1:length(data["bus"])
        b = data["bus"]["$(n_buses_expanded[b_id])"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            println("Checking bus $(n_buses_expanded[b_id]), position $(b_id)")
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    if b["auxiliary"] == "convdc"
                        number_switches = length(data["switch"])
                        #first switch 
                        added_switch_1 = number_switches + 1
                        println("Adding switch $added_switch_1")
                        data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_1"]["cost"] = 0.0
                        data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                        data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                        data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                        data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$added_switch_1"]["ZIL"] = false

                        #second switch
                        added_switch_2 = added_switch_1 + 1
                        println("Adding switch $added_switch_2")
                        data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                        data["switch"]["$added_switch_2"]["cost"] = 0.0
                        data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(n_buses_expanded[b_id]) 
                        data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                        data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                        data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                        data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                        data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                        #data["switch"]["$switch_id"]["ZIL"] = false
                    end
                end
            end
        end
    end

    # The total number of switches is sum forall b in B of (2∗nb +1) where B is the number of substations being split and nb is the number of grid elements connected to each substation
    switch_couples = compute_couples_of_switches(data) # using the function to check the couples of switches linking each grid element to both parts of the split busbar  
    data["switch_couples"] = Dict{String,Any}()
    data["switch_couples"] = deepcopy(switch_couples)
    data["dcswitch_couples"] = Dict{String,Any}()

    return data, switch_couples, extremes_ZIL
end

function DC_busbars_split(data_original,bus_to_be_split)
    data = deepcopy(data_original)
    min_bus_original = minimum([bus["index"] for (b, bus) in data["busdc"]]) 
    if length(bus_to_be_split) == 1
        for i in keys(data["busdc"])
            if parse(Int64,i) != bus_to_be_split
                data["busdc"]["$i"]["split"] = false
            end
        end
        data["busdc"]["$bus_to_be_split"]["split"] = true
        data["busdc"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["busdc"])
            data["busdc"]["$i"]["split"] = false
            data["busdc"]["$i"]["ZIL"] = false
        end
        for n in bus_to_be_split
            data["busdc"]["$n"]["split"] = true
            data["busdc"]["$n"]["ZIL"] = true
        end
    end

    # Adding a busdc next to the split
    n_buses_original_dc = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
    count_dc = 0
    for (b_id,b) in data["busdc"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original_dc # make sure we include only the original buses
            count_dc += 1
            b["busdc_split"] = deepcopy(b["index"])
            added_bus_dc = n_buses_original_dc + count_dc
            data["busdc"]["$added_bus_dc"] = deepcopy(b)
            data["busdc"]["$added_bus_dc"]["busdc_i"] = added_bus_dc
            data["busdc"]["$added_bus_dc"]["source_id"][2] = added_bus_dc 
            data["busdc"]["$added_bus_dc"]["index"] = added_bus_dc 
            data["busdc"]["$added_bus_dc"]["split"] = false
            data["busdc"]["$added_bus_dc"]["ZIL"] = true
        end
    end 

    
    # Creating a dictionary with the split buses
    extremes_ZIL_dc = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if b["split"] == true && !haskey(extremes_ZIL_dc,b["busdc_split"]) # isolating only the original buses being split
            extremes_ZIL_dc["$(b["busdc_split"])"] = []
        end
    end

    for b in eachindex(data["busdc"])#1:length(data["bus"])
        if haskey(data["busdc"]["$b"],"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["busdc_split"] == parse(Int64,i) && data["busdc"]["$b"]["index"] <= n_buses_original_dc  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL_dc[i],data["busdc"]["$b"]["index"])
                end
            end
        end
    end
    for b in eachindex(data["busdc"])#1:length(data["bus"])
        if haskey(data["busdc"]["$b"],"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["busdc_split"] == parse(Int64,i) && data["busdc"]["$b"]["index"] > n_buses_original_dc  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL_dc[i],data["busdc"]["$b"]["index"])
                end
            end
        end
    end
    

    data["dcswitch"] = Dict{String,Any}()
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses
    switch_id = 0
    for i in eachindex(extremes_ZIL_dc)
        switch_id += 1
        data["dcswitch"]["$switch_id"] = Dict{String,Any}()
        data["dcswitch"]["$switch_id"]["busdc_split"] = deepcopy(extremes_ZIL_dc[i][1])
        data["dcswitch"]["$switch_id"]["f_busdc"] = deepcopy(extremes_ZIL_dc[i][1]) 
        data["dcswitch"]["$switch_id"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][2])
        data["dcswitch"]["$switch_id"]["index"] = switch_id
        data["dcswitch"]["$switch_id"]["psw"] = 100.0
        data["dcswitch"]["$switch_id"]["thermal_rating"] = 100.0
        data["dcswitch"]["$switch_id"]["state"] = 1
        data["dcswitch"]["$switch_id"]["status"] = 1
        data["dcswitch"]["$switch_id"]["source_id"] = []
        data["dcswitch"]["$switch_id"]["cost"] = 1.0
        push!(data["dcswitch"]["$switch_id"]["source_id"],"dcswitch")
        push!(data["dcswitch"]["$switch_id"]["source_id"],switch_id)
        data["dcswitch"]["$switch_id"]["ZIL"] = true
    end

    # Branch dc
    for (br_id,br) in data["branchdc"] 
        for i in eachindex(extremes_ZIL_dc)
            n_buses = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
            if br["fbusdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id))
                data["busdc"]["$added_branch_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                if haskey(data["busdc"]["$added_branch_bus"],"split")
                    delete!(data["busdc"]["$added_branch_bus"],"split")
                end
                br["fbusdc"] = added_branch_bus
            elseif br["tbusdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id)) 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                data["busdc"]["$added_branch_bus"]["split"] = false
                br["tbusdc"] = added_branch_bus
            end
        end
    end

    # Converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
            for i in eachindex(extremes_ZIL_dc)
                n_buses = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
                if cv["busdc_i"] == parse(Int64,i)
                    added_conv_bus = n_buses + 1
                    data["busdc"]["$added_conv_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    #data["busdc"]["$added_conv_bus"]["bus_type"] = 1
                    data["busdc"]["$added_conv_bus"]["busdc_i"] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["index"] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_conv_bus"]["original"] = parse(Int64,cv_id) 
                    data["busdc"]["$added_conv_bus"]["auxiliary"] = "convdc"
                    data["busdc"]["$added_conv_bus"]["busdc_split"] = parse(Int64,i) 
                    data["busdc"]["$added_conv_bus"]["split"] = false
                    cv["busdc_i"] = added_conv_bus
                end
            end
        end
    end
    
    # Linking the auxiliary buses to both split buses with switches
    for (b_id,b) in data["busdc"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == true
            for i in eachindex(extremes_ZIL_dc)
                if b["busdc_split"] == parse(Int64,i) 
                    number_switches = length(data["dcswitch"])
                    added_switch_1 = number_switches + 1
                    data["dcswitch"]["$added_switch_1"] = deepcopy(data["dcswitch"]["1"])
                    data["dcswitch"]["$added_switch_1"]["cost"] = 0.0
                    data["dcswitch"]["$added_switch_1"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                    data["dcswitch"]["$added_switch_1"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][1])
                    data["dcswitch"]["$added_switch_1"]["index"] = added_switch_1
                    data["dcswitch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                    data["dcswitch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["dcswitch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                    data["dcswitch"]["$added_switch_1"]["busdc_split"] = parse(Int64,i) 
                    #data["dcswitch"]["$added_switch_1"]["ZIL"] = false

                    added_switch_2 = added_switch_1 + 1
                    data["dcswitch"]["$added_switch_2"] = deepcopy(data["dcswitch"]["1"])
                    data["dcswitch"]["$added_switch_2"]["cost"] = 0.0
                    data["dcswitch"]["$added_switch_2"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                    data["dcswitch"]["$added_switch_2"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][2])
                    data["dcswitch"]["$added_switch_2"]["index"] = added_switch_2
                    data["dcswitch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["dcswitch"]["$added_switch_2"]["original"] = deepcopy(b["original"])  
                    data["dcswitch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                    data["dcswitch"]["$added_switch_2"]["busdc_split"] = parse(Int64,i) 
                    #data["dcswitch"]["$added_switch_2"]["ZIL"] = false  
                end
            end
        end
    end

    dcswitch_couples = compute_couples_of_dcswitches(data)
    if haskey(data,"switch_couples")
    else
        data["switch_couples"] = Dict{String,Any}()
    end
    data["dcswitch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = deepcopy(dcswitch_couples)
    
    return data, dcswitch_couples, extremes_ZIL_dc
end

function compute_couples_of_switches(data)
    switch_couples = Dict{String,Any}() # creating a dictionary to check the couples of switches linking each grid element to both parts of the split busbar
    for (sw_id,sw) in data["switch"]
        for l in eachindex(data["switch"])
            if (haskey(sw, "auxiliary") && haskey(data["switch"][l], "auxiliary")) && (sw["auxiliary"] == data["switch"][l]["auxiliary"]) && (sw["original"] == data["switch"][l]["original"]) && (sw["index"] != data["switch"][l]["index"]) &&  (sw["bus_split"] == data["switch"][l]["bus_split"])
                switch_couples["$sw_id"] = Dict{String,Any}()
                switch_couples["$sw_id"]["f_sw"] = deepcopy(sw["index"])
                switch_couples["$sw_id"]["t_sw"] = deepcopy(data["switch"][l]["index"])
                switch_couples["$sw_id"]["bus_split"] = deepcopy(data["switch"][l]["bus_split"])
                for (s_id,s) in data["switch"] 
                    if !(haskey(s, "auxiliary")) && haskey(s,"ZIL") && s["bus_split"] == switch_couples["$sw_id"]["bus_split"]
                        switch_couples["$sw_id"]["switch_split"] = deepcopy(s["index"])
                    end
                end
            end
        end
    end 
    eliminate_duplicates_couple_of_switches(switch_couples)
    return switch_couples
end

function eliminate_duplicates_couple_of_switches(switch_couples)
    for (sw_id,sw) in switch_couples
        for l in eachindex(switch_couples)
            if sw["f_sw"] == switch_couples[l]["t_sw"] && sw["t_sw"] == switch_couples[l]["f_sw"]
                delete!(switch_couples,l)
            end
        end
    end
end

function compute_couples_of_switches_feas_check(data)
    switch_couples = Dict{String,Any}() # creating a dictionary to check the couples of switches linking each grid element to both parts of the split busbar
    for (sw_id,sw) in data["switch"]
        for l in eachindex(data["switch"])
            if (haskey(sw, "auxiliary") && haskey(data["switch"][l], "auxiliary")) && (sw["auxiliary"] == data["switch"][l]["auxiliary"]) && (sw["original"] == data["switch"][l]["original"]) && (sw["index"] != data["switch"][l]["index"]) &&  (sw["bus_split"] == data["switch"][l]["bus_split"])
                switch_couples["$sw_id"] = Dict{String,Any}()
                switch_couples["$sw_id"]["f_sw"] = deepcopy(sw["index"])
                switch_couples["$sw_id"]["t_sw"] = deepcopy(data["switch"][l]["index"])
                switch_couples["$sw_id"]["bus_split"] = deepcopy(data["switch"][l]["bus_split"])
                for (s_id,s) in data["switch"] 
                    if !(haskey(s, "auxiliary")) && haskey(s,"ZIL") && s["bus_split"] == switch_couples["$sw_id"]["bus_split"]
                        switch_couples["$sw_id"]["switch_split"] = deepcopy(s["index"])
                    end
                end
            end
        end
    end 
    return switch_couples
end

function compute_couples_of_dcswitches(data)
    switch_couples = Dict{String,Any}()
    for (sw_id,sw) in data["dcswitch"]
        for l in eachindex(data["dcswitch"])
            if (haskey(sw, "auxiliary") && haskey(data["dcswitch"][l], "auxiliary")) && (sw["auxiliary"] == data["dcswitch"][l]["auxiliary"]) && (sw["original"] == data["dcswitch"][l]["original"]) && (sw["index"] != data["dcswitch"][l]["index"]) && (sw["busdc_split"] == data["dcswitch"][l]["busdc_split"])
                switch_couples["$sw_id"] = Dict{String,Any}()
                switch_couples["$sw_id"]["f_sw"] = deepcopy(sw["index"])
                switch_couples["$sw_id"]["t_sw"] = deepcopy(data["dcswitch"][l]["index"])
                switch_couples["$sw_id"]["busdc_split"] = deepcopy(data["dcswitch"][l]["busdc_split"])
                for (s_id,s) in data["dcswitch"] 
                    if !(haskey(s, "auxiliary")) && haskey(s,"ZIL") && s["busdc_split"] == switch_couples["$sw_id"]["busdc_split"]
                        switch_couples["$sw_id"]["dcswitch_split"] = deepcopy(s["index"])
                    end
                end
            end
        end
    end
    eliminate_duplicates_couple_of_switches(switch_couples)
    return switch_couples
end

function compute_couples_of_dcswitches_mc(data)
    switch_couples = Dict{String,Any}()
    for (sw_id,sw) in data["dcswitch"]
        for l in eachindex(data["dcswitch"])
            if (haskey(sw, "auxiliary") && haskey(data["dcswitch"][l], "auxiliary")) && (sw["auxiliary"] == data["dcswitch"][l]["auxiliary"]) && (sw["original"] == data["dcswitch"][l]["original"]) && (sw["index"] != data["dcswitch"][l]["index"]) && (sw["busdc_split"] == data["dcswitch"][l]["busdc_split"]) && (sw["terminal"] == data["dcswitch"][l]["terminal"])
                switch_couples["$sw_id"] = Dict{String,Any}()
                switch_couples["$sw_id"]["f_sw"] = deepcopy(sw["index"])
                switch_couples["$sw_id"]["t_sw"] = deepcopy(data["dcswitch"][l]["index"])
                switch_couples["$sw_id"]["busdc_split"] = deepcopy(data["dcswitch"][l]["busdc_split"])
                switch_couples["$sw_id"]["terminal"] = deepcopy(data["dcswitch"][l]["terminal"])
                for (s_id,s) in data["dcswitch"] 
                    if !(haskey(s, "auxiliary")) && haskey(s,"ZIL") && s["busdc_split"] == switch_couples["$sw_id"]["busdc_split"] && s["terminal"] == switch_couples["$sw_id"]["terminal"]
                        switch_couples["$sw_id"]["dcswitch_split"] = deepcopy(s["index"])
                    end
                end
            end
        end
    end
    eliminate_duplicates_couple_of_switches(switch_couples)
    return switch_couples
end


function AC_busbar_split_AC_grid(data,bus_to_be_split) 
    min_bus_original = minimum([bus["index"] for (b, bus) in data["bus"]]) 
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in keys(data["bus"])
            if parse(Int64,i) != bus_to_be_split
                data["bus"]["$i"]["split"] = false
            end
        end
        data["bus"]["$bus_to_be_split"]["split"] = true
        data["bus"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["bus"])
            #if parse(Int64,i) != n #&& !haskey(data["bus"][i],"ZIL")
                data["bus"]["$i"]["split"] = false
                data["bus"]["$i"]["ZIL"] = false
            #end
        end
        for n in bus_to_be_split # selecting only split buses
            data["bus"]["$n"]["split"] = true
            data["bus"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original = maximum([bus["index"] for (b, bus) in data["bus"]]) 
    max_bus_original = minimum([bus["index"] for (b, bus) in data["bus"]]) 
    count_ = 0
    for (b_id,b) in data["bus"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original # make sure we include only the original buses
            count_ += 1
            b["bus_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            added_bus = n_buses_original + count_
            data["bus"]["$added_bus"] = deepcopy(b)
            data["bus"]["$added_bus"]["bus_type"] = 1
            data["bus"]["$added_bus"]["bus_i"] = added_bus 
            data["bus"]["$added_bus"]["source_id"][2] = added_bus 
            data["bus"]["$added_bus"]["index"] = added_bus
            data["bus"]["$added_bus"]["split"] = false
            data["bus"]["$added_bus"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if b["split"] == true && !haskey(extremes_ZIL,b["bus_split"]) # isolating only the original buses being split
            extremes_ZIL["$(b["bus_split"])"] = []
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] <= n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] > n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses, assuming there are no switches already existing in the test case. Using PowerModels format
    switch_id = 0
    for i in eachindex(extremes_ZIL)
        switch_id += 1
        data["switch"]["$switch_id"] = Dict{String,Any}()
        data["switch"]["$switch_id"]["bus_split"] = deepcopy(extremes_ZIL[i][1]) # the original bus being split is always the first elements in the vector of each key of extremes_ZIL
        data["switch"]["$switch_id"]["f_bus"] = deepcopy(extremes_ZIL[i][1]) # assigning from and to bus to each switch. One switch for each key in the extremes_ZIL
        data["switch"]["$switch_id"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
        data["switch"]["$switch_id"]["index"] = switch_id
        data["switch"]["$switch_id"]["psw"] = 100.0 # assuming a maximum active power for the switch
        data["switch"]["$switch_id"]["qsw"] = 100.0 # assuming a maximum reactive power for the switch
        data["switch"]["$switch_id"]["thermal_rating"] = 100.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        data["switch"]["$switch_id"]["cost"] = 1.0
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
        data["switch"]["$switch_id"]["ZIL"] = true
    end
     
    # Add a bus for each grid element connected to the bus being split
    # Gen
    for (g_id,g) in data["gen"] 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]])
            if g["gen_bus"] == parse(Int64,i) # isolating the generators connected to the split busbar. Adding new buses in PowerModels format
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_gen_bus"]["original"] = deepcopy(parse(Int64,g_id)) # element to indicate the original number of the grid elements before the split
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen" # type of grid element linked to the busbar being split
                data["bus"]["$added_gen_bus"]["split"] = false # indicating that the bus is not created because of the busbar split. It is an auxiliary bus for the grid element connected to the busbar being split
                data["bus"]["$added_gen_bus"]["bus_split"] = deepcopy(parse(Int64,i)) # number of the bus to which the grid element was originally attached to
                g["gen_bus"] = added_gen_bus # updating the number of the generator. The original number is indicated by the "original" element
            end
        end
    end 
    
    # Load -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    for (l_id,l) in data["load"] 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_load_bus"]["original"] = deepcopy(parse(Int64,l_id))  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    for (br_id,br) in data["branch"] 
        for i in eachindex(extremes_ZIL)
            #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, f_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id))
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                if haskey(data["bus"]["$added_branch_bus"],"split")
                    delete!(data["bus"]["$added_branch_bus"],"split")
                end
                br["f_bus"] = added_branch_bus
            elseif br["t_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, t_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id)) 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_branch_bus"]["split"] = false
                br["t_bus"] = added_branch_bus
            end
        end
    end
    
    
    # Linking the auxiliary buses to both split buses with switches. Two switches for each new auxiliary bus that was created before (close to reality representation)
    for (b_id,b) in data["bus"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    number_switches = length(data["switch"])
                    #first switch 
                    added_switch_1 = number_switches + 1
                    data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                    data["switch"]["$added_switch_1"]["cost"] = 0.0
                    data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                    data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                    data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                    data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                    #data["switch"]["$added_switch_1"]["ZIL"] = false

                    #second switch
                    added_switch_2 = added_switch_1 + 1
                    data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                    data["switch"]["$added_switch_2"]["cost"] = 0.0
                    data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                    data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                    data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                    data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                    #data["switch"]["$switch_id"]["ZIL"] = false
                end
            end
        end
    end
    # The total number of switches is sum forall b in B of (2∗nb +1) where B is the number of substations being split and nb is the number of grid elements connected to each substation
    switch_couples = compute_couples_of_switches(data) # using the function to check the couples of switches linking each grid element to both parts of the split busbar  
    data["switch_couples"] = Dict{String,Any}()
    data["switch_couples"] = deepcopy(switch_couples)
    return data, switch_couples, extremes_ZIL
end

function prepare_starting_value_dict(result,grid)
    for (b_id,b) in grid["bus"]
        if haskey(result["solution"]["bus"],b_id)
            if abs(result["solution"]["bus"]["$b_id"]["va"]) < 10^(-4)
                b["va_starting_value"] = 0.0
            else
                b["va_starting_value"] = result["solution"]["bus"]["$b_id"]["va"]
            end
            if abs(result["solution"]["bus"]["$b_id"]["vm"]) < 10^(-4)
                b["vm_starting_value"] = 0.0
            else
                b["vm_starting_value"] = result["solution"]["bus"]["$b_id"]["vm"]
            end
        else
            b["va_starting_value"] = 0.0
            b["vm_starting_value"] = 1.0
        end
    end
    for (b_id,b) in grid["gen"]
        if abs(result["solution"]["gen"]["$b_id"]["pg"]) < 10^(-5)
            b["pg_starting_value"] = 0.0
        else
            b["pg_starting_value"] = result["solution"]["gen"]["$b_id"]["pg"]
        end
        if abs(result["solution"]["gen"]["$b_id"]["qg"]) < 10^(-5)
            b["qg_starting_value"] = 0.0
        else
            b["qg_starting_value"] = result["solution"]["gen"]["$b_id"]["qg"]
        end
    end
    for (sw_id,sw) in grid["switch"]
        if !haskey(sw,"auxiliary") # calling ZILs
            sw["starting_value"] = 1.0
        else
            if haskey(grid["switch_couples"],sw_id)
                grid["switch"]["$(grid["switch_couples"][sw_id]["f_sw"])"]["starting_value"] = 0.0
                grid["switch"]["$(grid["switch_couples"][sw_id]["t_sw"])"]["starting_value"] = 1.0
            end
            #sw["starting_value"] = 0.0
        end
    end
end

function prepare_starting_value_dict_lpac(result,grid)
    for (b_id,b) in grid["bus"]
        if haskey(result["solution"]["bus"],b_id)
            if abs(result["solution"]["bus"]["$b_id"]["va"]) < 10^(-4)
                b["va_starting_value"] = 0.0
            else
                b["va_starting_value"] = result["solution"]["bus"]["$b_id"]["va"]
            end
            if abs(result["solution"]["bus"]["$b_id"]["phi"]) < 10^(-4)
                b["phi_starting_value"] = 0.0
            else
                b["phi_starting_value"] = result["solution"]["bus"]["$b_id"]["phi"]
            end
        else
            b["va_starting_value"] = 0.0
            b["phi_starting_value"] = 0.1
        end
    end
    for (b_id,b) in grid["gen"]
        if abs(result["solution"]["gen"]["$b_id"]["pg"]) < 10^(-5)
            b["pg_starting_value"] = 0.0
        else
            b["pg_starting_value"] = result["solution"]["gen"]["$b_id"]["pg"]
        end
        if abs(result["solution"]["gen"]["$b_id"]["qg"]) < 10^(-5)
            b["qg_starting_value"] = 0.0
        else
            b["qg_starting_value"] = result["solution"]["gen"]["$b_id"]["qg"]
        end
    end
    for (sw_id,sw) in grid["switch"]
        if !haskey(sw,"auxiliary") # calling ZILs
            sw["starting_value"] = 1.0
        else
            if haskey(grid["switch_couples"],sw_id)
                grid["switch"]["$(grid["switch_couples"][sw_id]["f_sw"])"]["starting_value"] = 0.0
                grid["switch"]["$(grid["switch_couples"][sw_id]["t_sw"])"]["starting_value"] = 1.0
            end
        end
    end
end

function AC_busbars_split_multiconductor(data,bus_to_be_split) 
    min_bus_original = minimum([bus["index"] for (b, bus) in data["bus"]]) 
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in keys(data["bus"])
            if parse(Int64,i) != bus_to_be_split
                data["bus"]["$i"]["split"] = false
            end
        end
        data["bus"]["$bus_to_be_split"]["split"] = true
        data["bus"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["bus"])
            #if parse(Int64,i) != n #&& !haskey(data["bus"][i],"ZIL")
                data["bus"]["$i"]["split"] = false
                data["bus"]["$i"]["ZIL"] = false
            #end
        end
        for n in bus_to_be_split # selecting only split buses
            data["bus"]["$n"]["split"] = true
            data["bus"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original = maximum([bus["index"] for (b, bus) in data["bus"]]) 
    count_ = 0
    for (b_id,b) in data["bus"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original # make sure we include only the original buses
            count_ += 1
            b["bus_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            added_bus = n_buses_original + count_
            data["bus"]["$added_bus"] = deepcopy(b)
            data["bus"]["$added_bus"]["bus_type"] = 1
            data["bus"]["$added_bus"]["bus_i"] = added_bus 
            data["bus"]["$added_bus"]["source_id"][2] = added_bus 
            data["bus"]["$added_bus"]["index"] = added_bus
            data["bus"]["$added_bus"]["split"] = false
            data["bus"]["$added_bus"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if b["split"] == true && !haskey(extremes_ZIL,b["bus_split"]) # isolating only the original buses being split
            extremes_ZIL["$(b["bus_split"])"] = []
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] <= n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    for b in eachindex(data["bus"])#1:length(data["bus"])
        if haskey(data["bus"]["$b"],"bus_split")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) && data["bus"]["$b"]["index"] > n_buses_original  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    push!(extremes_ZIL[i],data["bus"]["$b"]["index"])
                end
            end
        end
    end
    
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses, assuming there are no switches already existing in the test case. Using PowerModels format
    switch_id = 0
    for i in eachindex(extremes_ZIL)
        switch_id += 1
        data["switch"]["$switch_id"] = Dict{String,Any}()
        data["switch"]["$switch_id"]["bus_split"] = deepcopy(extremes_ZIL[i][1]) # the original bus being split is always the first elements in the vector of each key of extremes_ZIL
        data["switch"]["$switch_id"]["f_bus"] = deepcopy(extremes_ZIL[i][1]) # assigning from and to bus to each switch. One switch for each key in the extremes_ZIL
        data["switch"]["$switch_id"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
        data["switch"]["$switch_id"]["index"] = switch_id
        data["switch"]["$switch_id"]["psw"] = 100.0 # assuming a maximum active power for the switch
        data["switch"]["$switch_id"]["qsw"] = 100.0 # assuming a maximum reactive power for the switch
        data["switch"]["$switch_id"]["thermal_rating"] = 100.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        data["switch"]["$switch_id"]["cost"] = 1.0
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
        data["switch"]["$switch_id"]["ZIL"] = true
    end
     
    # Add a bus for each grid element connected to the bus being split
    # Gen
    for (g_id,g) in data["gen"] 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if g["gen_bus"] == parse(Int64,i) # isolating the generators connected to the split busbar. Adding new buses in PowerModels format
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_gen_bus"]["original"] = deepcopy(parse(Int64,g_id)) # element to indicate the original number of the grid elements before the split
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen" # type of grid element linked to the busbar being split
                data["bus"]["$added_gen_bus"]["split"] = false # indicating that the bus is not created because of the busbar split. It is an auxiliary bus for the grid element connected to the busbar being split
                data["bus"]["$added_gen_bus"]["bus_split"] = deepcopy(parse(Int64,i)) # number of the bus to which the grid element was originally attached to
                g["gen_bus"] = added_gen_bus # updating the number of the generator. The original number is indicated by the "original" element
            end
        end
    end 
    
    # Load -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    for (l_id,l) in data["load"] 
        #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true # element to indicate that this bus was generated as an auxiliary bus for a grid element that was connected to a busbar being split
                data["bus"]["$added_load_bus"]["original"] = deepcopy(parse(Int64,l_id))  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch -> repeating what was done for the generators. Refer to the generator part above for detailed comments.
    for (br_id,br) in data["branch"] 
        for i in eachindex(extremes_ZIL)
            #n_buses = length(data["bus"]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) 
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, f_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id))
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                if haskey(data["bus"]["$added_branch_bus"],"split")
                    delete!(data["bus"]["$added_branch_bus"],"split")
                end
                br["f_bus"] = added_branch_bus
            elseif br["t_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, t_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["original"] = deepcopy(parse(Int64,br_id)) 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                data["bus"]["$added_branch_bus"]["split"] = false
                br["t_bus"] = added_branch_bus
            end
        end
    end
    
    # Multiconductor model for converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"]
            n_poles = length(cv["Pacrated"]) 
            poles = keys(cv["Pacrated"])
            bus_ac_i_helper = deepcopy(cv["busac_i"])
            cv["busac_i"] = Dict{String,Any}()
            for p in poles
                cv["busac_i"]["$p"] = deepcopy(bus_ac_i_helper)
            end
            for i in eachindex(extremes_ZIL)
                for p in poles 
                    n_buses = maximum([bus["index"] for (b, bus) in data["bus"]])
                    if cv["busac_i"][p] == parse(Int64,i)
                        added_conv_bus = n_buses + 1
                        data["bus"]["$added_conv_bus"] = deepcopy(data["bus"]["$min_bus_original"])
                        data["bus"]["$added_conv_bus"]["bus_type"] = 1
                        data["bus"]["$added_conv_bus"]["bus_i"] = added_conv_bus 
                        data["bus"]["$added_conv_bus"]["index"] = added_conv_bus 
                        data["bus"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                        data["bus"]["$added_conv_bus"]["auxiliary_bus"] = true 
                        data["bus"]["$added_conv_bus"]["original"] = deepcopy(parse(Int64,cv_id)) 
                        data["bus"]["$added_conv_bus"]["auxiliary"] = "convdc"
                        data["bus"]["$added_conv_bus"]["pole"] = "$(p)"
                        data["bus"]["$added_conv_bus"]["bus_split"] = deepcopy(parse(Int64,i)) 
                        data["bus"]["$added_conv_bus"]["split"] = false
                        cv["busac_i"][p] = added_conv_bus
                    end
                end
            end
        end
    end
    
    
    # Linking the auxiliary buses to both split buses with switches. Two switches for each new auxiliary bus that was created before (close to reality representation)
    for (b_id,b) in data["bus"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    number_switches = length(data["switch"])
                    #first switch 
                    added_switch_1 = number_switches + 1
                    data["switch"]["$added_switch_1"] = deepcopy(data["switch"]["1"])
                    data["switch"]["$added_switch_1"]["cost"] = 0.0
                    data["switch"]["$added_switch_1"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch_1"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                    data["switch"]["$added_switch_1"]["index"] = added_switch_1 
                    data["switch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                    data["switch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch_1"]["bus_split"] = parse(Int64,i) 
                    if haskey(b,"pole")
                        data["switch"]["$added_switch_1"]["pole"] = deepcopy(b["pole"])
                    end
                    #data["switch"]["$added_switch_1"]["ZIL"] = false

                    #second switch
                    added_switch_2 = added_switch_1 + 1
                    data["switch"]["$added_switch_2"] = deepcopy(data["switch"]["1"])
                    data["switch"]["$added_switch_2"]["cost"] = 0.0
                    data["switch"]["$added_switch_2"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch_2"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                    data["switch"]["$added_switch_2"]["index"] = added_switch_2 
                    data["switch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                    data["switch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch_2"]["bus_split"] = parse(Int64,i) 
                    if haskey(b,"pole")
                        data["switch"]["$added_switch_2"]["pole"] = deepcopy(b["pole"])
                    end
                    #data["switch"]["$switch_id"]["ZIL"] = false
                end
            end
        end
    end
    # The total number of switches is sum forall b in B of (2∗nb +1) where B is the number of substations being split and nb is the number of grid elements connected to each substation
    switch_couples = compute_couples_of_switches(data) # using the function to check the couples of switches linking each grid element to both parts of the split busbar  
    data["switch_couples"] = Dict{String,Any}()
    data["switch_couples"] = deepcopy(switch_couples)
    data["dcswitch_couples"] = Dict{String,Any}()

    return data, switch_couples, extremes_ZIL
end

function DC_busbars_split_multiconductor(data,bus_to_be_split) 
    min_bus_original = minimum([bus["index"] for (b, bus) in data["busdc"]]) 
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in keys(data["busdc"])
            if parse(Int64,i) != bus_to_be_split
                data["busdc"]["$i"]["split"] = false
            end
        end
        data["busdc"]["$bus_to_be_split"]["split"] = true
        data["busdc"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["busdc"])
            #if parse(Int64,i) != n #&& !haskey(data["busdc"][i],"ZIL")
                data["busdc"]["$i"]["split"] = false
                data["busdc"]["$i"]["ZIL"] = false
            #end
        end
        for n in bus_to_be_split # selecting only split buses
            data["busdc"]["$n"]["split"] = true
            data["busdc"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original_dc = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
    count_dc = 0
    for (b_id,b) in data["busdc"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original_dc # make sure we include only the original buses
            count_dc += 1
            added_bus_dc = n_buses_original_dc + count_dc
            data["busdc"]["$added_bus_dc"] = deepcopy(b)
            data["busdc"]["$added_bus_dc"]["busdc_i"] = added_bus_dc
            data["busdc"]["$added_bus_dc"]["busdc_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            data["busdc"]["$added_bus_dc"]["source_id"][2] = added_bus_dc 
            data["busdc"]["$added_bus_dc"]["index"] = added_bus_dc
            data["busdc"]["$added_bus_dc"]["split"] = false
            data["busdc"]["$added_bus_dc"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL_dc = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if b["split"] == true #&& !haskey(extremes_ZIL_dc,b["busdc_split"]) # isolating only the original buses being split
            extremes_ZIL_dc["$(b["index"])"] = Dict{String,Any}()
            for terminal in keys(b["Vdcmax"])
                extremes_ZIL_dc["$(b["index"])"]["$terminal"] = []
            end
        end
    end
    
    for b in eachindex(data["busdc"])#1:length(data["bus"])
        #if haskey(data["busdc"]["$b"],"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["index"] == parse(Int64,i) && data["busdc"]["$b"]["index"] <= n_buses_original_dc  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    terminals = keys(data["busdc"]["$b"]["Vdcmax"])
                    for terminal in terminals
                        push!(extremes_ZIL_dc["$(data["busdc"]["$b"]["index"])"]["$terminal"],data["busdc"]["$b"]["index"])
                    end
                end
            end
        #end
    end
    for b in eachindex(data["busdc"])#1:length(data["busdc"])
        if haskey(data["busdc"]["$b"],"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["busdc_split"] == parse(Int64,i) && data["busdc"]["$b"]["index"] > n_buses_original_dc  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    terminals = keys(data["busdc"]["$b"]["Vdcmax"])
                    for terminal in terminals
                        push!(extremes_ZIL_dc["$(data["busdc"]["$b"]["busdc_split"])"]["$terminal"],data["busdc"]["$b"]["index"])
                    end
                end
            end
        end
    end
    
    data["dcswitch"] = Dict{String,Any}()
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses, assuming there are no switches already existing in the test case. Using PowerModels format
    switch_id = 0
    for i in eachindex(extremes_ZIL_dc)
        for terminal in eachindex(extremes_ZIL_dc[i])
            switch_id += 1
            data["dcswitch"]["$switch_id"] = Dict{String,Any}()
            data["dcswitch"]["$switch_id"]["busdc_split"] = deepcopy(extremes_ZIL_dc[i][terminal][1]) # the original bus being split is always the first elements in the vector of each key of extremes_ZIL
            data["dcswitch"]["$switch_id"]["f_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][1]) # assigning from and to bus to each switch. One switch for each key in the extremes_ZIL
            data["dcswitch"]["$switch_id"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][2])
            data["dcswitch"]["$switch_id"]["index"] = switch_id
            data["dcswitch"]["$switch_id"]["terminal"] = terminal
            data["dcswitch"]["$switch_id"]["psw"] = 100.0 # assuming a maximum active power for the switch
            data["dcswitch"]["$switch_id"]["thermal_rating"] = 100.0
            data["dcswitch"]["$switch_id"]["state"] = 1
            data["dcswitch"]["$switch_id"]["status"] = 1
            data["dcswitch"]["$switch_id"]["source_id"] = []
            data["dcswitch"]["$switch_id"]["cost"] = 1.0
            push!(data["dcswitch"]["$switch_id"]["source_id"],"dcswitch")
            push!(data["dcswitch"]["$switch_id"]["source_id"],switch_id)
            data["dcswitch"]["$switch_id"]["ZIL"] = true
        end
    end
    
    # Add a bus for each grid element connected to the bus being split
    
    # Branch dc 
    for (br_id,br) in data["branchdc"] 
        n_conductors = length(br["status"]) 
        conductors = keys(br["status"])
        fbusdc_helper = deepcopy(br["fbusdc"])
        tbusdc_helper = deepcopy(br["tbusdc"])
        br["fbusdc"] = Dict{String,Any}()
        br["tbusdc"] = Dict{String,Any}()
        for c in conductors
            br["fbusdc"]["$c"] = deepcopy(fbusdc_helper)
            br["tbusdc"]["$c"] = deepcopy(tbusdc_helper)
        end

        for i in eachindex(extremes_ZIL_dc)
                n_buses_dc = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
                c = first(conductors)
                if br["fbusdc"][c] == parse(Int64,i) && !haskey(br,"ZIL")
                    added_branchdc_bus = n_buses_dc + 1
                    data["busdc"]["$added_branchdc_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    data["busdc"]["$added_branchdc_bus"]["busdc_i"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["index"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["source_id"][2] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_branchdc_bus"]["original"] = deepcopy(parse(Int64,br_id))
                    data["busdc"]["$added_branchdc_bus"]["auxiliary"] = "branchdc"
                    data["busdc"]["$added_branchdc_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                    data["busdc"]["$added_branchdc_bus"]["split"] = false
                    for cond in conductors
                        br["fbusdc"][cond] = added_branchdc_bus
                    end
                    terminals = eachindex(data["busdc"]["$added_branchdc_bus"]["Vdcmax"])
                    terminals_to_be_deleted = setdiff(terminals, conductors)
                    println("Deleting terminals $terminals_to_be_deleted from auxiliary DC bus $added_branchdc_bus")
                    for del in terminals_to_be_deleted
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmax"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Cdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Pdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmin"],del)
                    end
                elseif br["tbusdc"][c] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, t_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                    added_branchdc_bus = n_buses_dc + 1
                    data["busdc"]["$added_branchdc_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    data["busdc"]["$added_branchdc_bus"]["busdc_i"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["index"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["source_id"][2] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_branchdc_bus"]["original"] = deepcopy(parse(Int64,br_id)) 
                    data["busdc"]["$added_branchdc_bus"]["auxiliary"] = "branchdc"
                    #data["busdc"]["$added_branchdc_bus"]["pole"] = "$(p)"
                    data["busdc"]["$added_branchdc_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                    data["busdc"]["$added_branchdc_bus"]["split"] = false
                    for cond in conductors
                        br["tbusdc"][cond] = added_branchdc_bus
                    end
                    terminals = eachindex(data["busdc"]["$added_branchdc_bus"]["Vdcmax"])
                    terminals_to_be_deleted = setdiff(terminals, conductors)
                    println("Deleting terminals $terminals_to_be_deleted from auxiliary DC bus $added_branchdc_bus")
                    for del in terminals_to_be_deleted
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmax"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Cdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Pdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmin"],del)
                    end
                end
        end
    end
    
    # Multiconductor model for converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"]
            n_poles = length(cv["Pacrated"]) 
            poles = keys(cv["Pacrated"])
            bus_dc_i_helper = deepcopy(cv["busdc_i"])
            cv["busdc_i"] = Dict{String,Any}()
            for p in poles
                cv["busdc_i"]["$p"] = deepcopy(bus_dc_i_helper)
            end

            for i in eachindex(extremes_ZIL_dc)
                n_buses_dc = maximum([bus["index"] for (b, bus) in data["busdc"]])
                p = first(poles)
                if cv["busdc_i"][p] == parse(Int64,i)
                    added_conv_bus = n_buses_dc + 1
                    data["busdc"]["$added_conv_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    data["busdc"]["$added_conv_bus"]["busdc_i"] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["index"] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_conv_bus"]["original"] = deepcopy(parse(Int64,cv_id)) 
                    data["busdc"]["$added_conv_bus"]["auxiliary"] = "convdc"
                    data["busdc"]["$added_conv_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                    data["busdc"]["$added_conv_bus"]["split"] = false
                    for p in poles
                        cv["busdc_i"][p] = added_conv_bus
                        data["busdc"]["$added_conv_bus"]["pole"] = "$(p)"
                    end
                end
            end
        end
    end
    
    
    # Linking the auxiliary buses to both split buses with switches. Two switches for each new auxiliary bus that was created before (close to reality representation)
    for (b_id,b) in data["busdc"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            for i in eachindex(extremes_ZIL_dc)
                if b["busdc_split"] == parse(Int64,i)
                    for terminal in eachindex(b["Vdcmax"])
                        number_switches = length(data["dcswitch"])
                        #first switch 
                        added_switch_1 = number_switches + 1
                        data["dcswitch"]["$added_switch_1"] = deepcopy(data["dcswitch"]["1"])
                        data["dcswitch"]["$added_switch_1"]["cost"] = 0.0
                        data["dcswitch"]["$added_switch_1"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                        data["dcswitch"]["$added_switch_1"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][1])
                        data["dcswitch"]["$added_switch_1"]["index"] = added_switch_1 
                        data["dcswitch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                        data["dcswitch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["dcswitch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                        data["dcswitch"]["$added_switch_1"]["busdc_split"] = parse(Int64,i) 
                        data["dcswitch"]["$added_switch_1"]["terminal"] = deepcopy(terminal)

                        #second switch
                        added_switch_2 = added_switch_1 + 1
                        data["dcswitch"]["$added_switch_2"] = deepcopy(data["dcswitch"]["1"])
                        data["dcswitch"]["$added_switch_2"]["cost"] = 0.0
                        data["dcswitch"]["$added_switch_2"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                        data["dcswitch"]["$added_switch_2"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][2])
                        data["dcswitch"]["$added_switch_2"]["index"] = added_switch_2 
                        data["dcswitch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                        data["dcswitch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["dcswitch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                        data["dcswitch"]["$added_switch_2"]["busdc_split"] = parse(Int64,i) 
                        data["dcswitch"]["$added_switch_2"]["terminal"] = deepcopy(terminal)
                    end
                end
            end
        end
    end
    # The total number of switches is sum forall b in B of (2∗nb +1) where B is the number of substations being split and nb is the number of grid elements connected to each substation
    dcswitch_couples = compute_couples_of_dcswitches_mc(data) # using the function to check the couples of switches linking each grid element to both parts of the split busbar  
    data["switch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = deepcopy(dcswitch_couples)
    return data, dcswitch_couples, extremes_ZIL_dc
end

function DC_busbars_split_multiconductor_updated(data,bus_to_be_split) 
    min_bus_original = minimum([bus["index"] for (b, bus) in data["busdc"]]) 
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in keys(data["busdc"])
            if parse(Int64,i) != bus_to_be_split
                data["busdc"]["$i"]["split"] = false
            end
        end
        data["busdc"]["$bus_to_be_split"]["split"] = true
        data["busdc"]["$bus_to_be_split"]["ZIL"] = true
    else
        for i in keys(data["busdc"])
            #if parse(Int64,i) != n #&& !haskey(data["busdc"][i],"ZIL")
                data["busdc"]["$i"]["split"] = false
                data["busdc"]["$i"]["ZIL"] = false
            #end
        end
        for n in bus_to_be_split # selecting only split buses
            data["busdc"]["$n"]["split"] = true
            data["busdc"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original_dc = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
    count_dc = 0
    for (b_id,b) in data["busdc"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original_dc # make sure we include only the original buses
            count_dc += 1
            added_bus_dc = n_buses_original_dc + count_dc
            data["busdc"]["$added_bus_dc"] = deepcopy(b)
            data["busdc"]["$added_bus_dc"]["busdc_i"] = added_bus_dc
            data["busdc"]["$added_bus_dc"]["busdc_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            data["busdc"]["$added_bus_dc"]["source_id"][2] = added_bus_dc 
            data["busdc"]["$added_bus_dc"]["index"] = added_bus_dc
            data["busdc"]["$added_bus_dc"]["split"] = false
            data["busdc"]["$added_bus_dc"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL_dc = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if b["split"] == true #&& !haskey(extremes_ZIL_dc,b["busdc_split"]) # isolating only the original buses being split
            extremes_ZIL_dc["$(b["index"])"] = Dict{String,Any}()
            for terminal in keys(b["Vdcmax"])
                extremes_ZIL_dc["$(b["index"])"]["$terminal"] = []
            end
        end
    end
    
    for b in eachindex(data["busdc"])#1:length(data["bus"])
        #if haskey(data["busdc"]["$b"],"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["index"] == parse(Int64,i) && data["busdc"]["$b"]["index"] <= n_buses_original_dc  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    terminals = keys(data["busdc"]["$b"]["Vdcmax"])
                    for terminal in terminals
                        push!(extremes_ZIL_dc["$(data["busdc"]["$b"]["index"])"]["$terminal"],data["busdc"]["$b"]["index"])
                    end
                end
            end
        #end
    end
    for b in eachindex(data["busdc"])#1:length(data["busdc"])
        if haskey(data["busdc"]["$b"],"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["busdc_split"] == parse(Int64,i) && data["busdc"]["$b"]["index"] > n_buses_original_dc  # counting only the buses related to the bus split, pushing them to the keys in the dictionary
                    terminals = keys(data["busdc"]["$b"]["Vdcmax"])
                    for terminal in terminals
                        push!(extremes_ZIL_dc["$(data["busdc"]["$b"]["busdc_split"])"]["$terminal"],data["busdc"]["$b"]["index"])
                    end
                end
            end
        end
    end
    
    data["dcswitch"] = Dict{String,Any}()
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses, assuming there are no switches already existing in the test case. Using PowerModels format
    switch_id = 0
    for i in eachindex(extremes_ZIL_dc)
        for terminal in eachindex(extremes_ZIL_dc[i])
            switch_id += 1
            data["dcswitch"]["$switch_id"] = Dict{String,Any}()
            data["dcswitch"]["$switch_id"]["busdc_split"] = deepcopy(extremes_ZIL_dc[i][terminal][1]) # the original bus being split is always the first elements in the vector of each key of extremes_ZIL
            data["dcswitch"]["$switch_id"]["f_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][1]) # assigning from and to bus to each switch. One switch for each key in the extremes_ZIL
            data["dcswitch"]["$switch_id"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][2])
            data["dcswitch"]["$switch_id"]["index"] = switch_id
            data["dcswitch"]["$switch_id"]["terminal"] = terminal
            data["dcswitch"]["$switch_id"]["psw"] = 100.0 # assuming a maximum active power for the switch
            data["dcswitch"]["$switch_id"]["thermal_rating"] = 100.0
            data["dcswitch"]["$switch_id"]["state"] = 1
            data["dcswitch"]["$switch_id"]["status"] = 1
            data["dcswitch"]["$switch_id"]["source_id"] = []
            data["dcswitch"]["$switch_id"]["cost"] = 1.0
            push!(data["dcswitch"]["$switch_id"]["source_id"],"dcswitch")
            push!(data["dcswitch"]["$switch_id"]["source_id"],switch_id)
            data["dcswitch"]["$switch_id"]["ZIL"] = true
        end
    end
    
    # Add a bus for each grid element connected to the bus being split
    
    # Branch dc 
    for (br_id,br) in data["branchdc"] 
        n_conductors = length(br["status"]) 
        conductors = keys(br["status"])
        #fbusdc_helper = deepcopy(br["fbusdc"])
        #tbusdc_helper = deepcopy(br["tbusdc"])
        #br["fbusdc"] = Dict{String,Any}()
        #br["tbusdc"] = Dict{String,Any}()
        #for c in conductors
        #    br["fbusdc"]["$c"] = deepcopy(fbusdc_helper)
        #    br["tbusdc"]["$c"] = deepcopy(tbusdc_helper)
        #end

        for i in eachindex(extremes_ZIL_dc)
                n_buses_dc = maximum([bus["index"] for (b, bus) in data["busdc"]]) 
                c = first(conductors)
                if br["fbusdc"] == parse(Int64,i) && !haskey(br,"ZIL")
                    added_branchdc_bus = n_buses_dc + 1
                    data["busdc"]["$added_branchdc_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    data["busdc"]["$added_branchdc_bus"]["busdc_i"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["index"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["source_id"][2] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_branchdc_bus"]["original"] = deepcopy(parse(Int64,br_id))
                    data["busdc"]["$added_branchdc_bus"]["auxiliary"] = "branchdc"
                    data["busdc"]["$added_branchdc_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                    data["busdc"]["$added_branchdc_bus"]["split"] = false
                    #for cond in conductors
                        br["fbusdc"] = added_branchdc_bus
                    #end
                    terminals = eachindex(data["busdc"]["$added_branchdc_bus"]["Vdcmax"])
                    terminals_to_be_deleted = setdiff(terminals, conductors)
                    println("Deleting terminals $terminals_to_be_deleted from auxiliary DC bus $added_branchdc_bus")
                    for del in terminals_to_be_deleted
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmax"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Cdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Pdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmin"],del)
                    end
                elseif br["tbusdc"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, t_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                    added_branchdc_bus = n_buses_dc + 1
                    data["busdc"]["$added_branchdc_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    data["busdc"]["$added_branchdc_bus"]["busdc_i"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["index"] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["source_id"][2] = added_branchdc_bus 
                    data["busdc"]["$added_branchdc_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_branchdc_bus"]["original"] = deepcopy(parse(Int64,br_id)) 
                    data["busdc"]["$added_branchdc_bus"]["auxiliary"] = "branchdc"
                    #data["busdc"]["$added_branchdc_bus"]["pole"] = "$(p)"
                    data["busdc"]["$added_branchdc_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                    data["busdc"]["$added_branchdc_bus"]["split"] = false
                    #for cond in conductors
                        br["tbusdc"] = added_branchdc_bus
                    #end
                    terminals = eachindex(data["busdc"]["$added_branchdc_bus"]["Vdcmax"])
                    terminals_to_be_deleted = setdiff(terminals, conductors)
                    println("Deleting terminals $terminals_to_be_deleted from auxiliary DC bus $added_branchdc_bus")
                    for del in terminals_to_be_deleted
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmax"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Cdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Pdc"],del)
                        delete!(data["busdc"]["$added_branchdc_bus"]["Vdcmin"],del)
                    end
                end
        end
    end
    
    # Multiconductor model for converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"]
            n_poles = length(cv["Pacrated"]) 
            poles = keys(cv["Pacrated"])
            bus_dc_i_helper = deepcopy(cv["busdc_i"])
            cv["busdc_i"] = Dict{String,Any}()
            for p in poles
                cv["busdc_i"]["$p"] = deepcopy(bus_dc_i_helper)
            end

            for i in eachindex(extremes_ZIL_dc)
                n_buses_dc = maximum([bus["index"] for (b, bus) in data["busdc"]])
                p = first(poles)
                if cv["busdc_i"][p] == parse(Int64,i)
                    added_conv_bus = n_buses_dc + 1
                    data["busdc"]["$added_conv_bus"] = deepcopy(data["busdc"]["$min_bus_original"])
                    data["busdc"]["$added_conv_bus"]["busdc_i"] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["index"] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                    data["busdc"]["$added_conv_bus"]["auxiliary_bus"] = true 
                    data["busdc"]["$added_conv_bus"]["original"] = deepcopy(parse(Int64,cv_id)) 
                    data["busdc"]["$added_conv_bus"]["auxiliary"] = "convdc"
                    data["busdc"]["$added_conv_bus"]["busdc_split"] = deepcopy(parse(Int64,i)) 
                    data["busdc"]["$added_conv_bus"]["split"] = false
                    for p in poles
                        cv["busdc_i"][p] = added_conv_bus
                        data["busdc"]["$added_conv_bus"]["pole"] = "$(p)"
                    end
                end
            end
        end
    end
    
    
    # Linking the auxiliary buses to both split buses with switches. Two switches for each new auxiliary bus that was created before (close to reality representation)
    for (b_id,b) in data["busdc"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["auxiliary_bus"] == true
            for i in eachindex(extremes_ZIL_dc)
                if b["busdc_split"] == parse(Int64,i)
                    for terminal in eachindex(b["Vdcmax"])
                        number_switches = length(data["dcswitch"])
                        #first switch 
                        added_switch_1 = number_switches + 1
                        data["dcswitch"]["$added_switch_1"] = deepcopy(data["dcswitch"]["1"])
                        data["dcswitch"]["$added_switch_1"]["cost"] = 0.0
                        data["dcswitch"]["$added_switch_1"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                        data["dcswitch"]["$added_switch_1"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][1])
                        data["dcswitch"]["$added_switch_1"]["index"] = added_switch_1 
                        data["dcswitch"]["$added_switch_1"]["source_id"][2] = deepcopy(added_switch_1)
                        data["dcswitch"]["$added_switch_1"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["dcswitch"]["$added_switch_1"]["original"] = deepcopy(b["original"]) 
                        data["dcswitch"]["$added_switch_1"]["busdc_split"] = parse(Int64,i) 
                        data["dcswitch"]["$added_switch_1"]["terminal"] = deepcopy(terminal)

                        #second switch
                        added_switch_2 = added_switch_1 + 1
                        data["dcswitch"]["$added_switch_2"] = deepcopy(data["dcswitch"]["1"])
                        data["dcswitch"]["$added_switch_2"]["cost"] = 0.0
                        data["dcswitch"]["$added_switch_2"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                        data["dcswitch"]["$added_switch_2"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][terminal][2])
                        data["dcswitch"]["$added_switch_2"]["index"] = added_switch_2 
                        data["dcswitch"]["$added_switch_2"]["source_id"][2] = deepcopy(added_switch_2)
                        data["dcswitch"]["$added_switch_2"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                        data["dcswitch"]["$added_switch_2"]["original"] = deepcopy(b["original"]) 
                        data["dcswitch"]["$added_switch_2"]["busdc_split"] = parse(Int64,i) 
                        data["dcswitch"]["$added_switch_2"]["terminal"] = deepcopy(terminal)
                    end
                end
            end
        end
    end
    # The total number of switches is sum forall b in B of (2∗nb +1) where B is the number of substations being split and nb is the number of grid elements connected to each substation
    dcswitch_couples = compute_couples_of_dcswitches_mc(data) # using the function to check the couples of switches linking each grid element to both parts of the split busbar  
    data["switch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = deepcopy(dcswitch_couples)
    return data, dcswitch_couples, extremes_ZIL_dc
end
