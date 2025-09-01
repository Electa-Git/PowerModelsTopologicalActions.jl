## TO BE FIXED, DO NOT USE ###

# Function to split a set of selected AC busbars, with fixed connections
function AC_busbar_split_more_buses_fixed(data,bus_to_be_split) # one can split whatever number of ac busbars
    
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
        for n in bus_to_be_split
            for i in keys(data["bus"])
                if parse(Int64,i) != n && !haskey(data["bus"][i],"ZIL")
                    data["bus"]["$i"]["split"] = false
                end
            end
            data["bus"]["$n"]["split"] = true
            data["bus"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original = length(data["bus"])
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
            split_bus = b["bus_split"]
            extremes_ZIL["$split_bus"] = []
        end
    end
    for b in 1:length(data["bus"])
        if haskey(data["bus"]["$b"],"ZIL")
            for i in eachindex(extremes_ZIL)
                if data["bus"]["$b"]["bus_split"] == parse(Int64,i) # counting only the buses related to the bus split, pushing them to the keys in the dictionary
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
        data["switch"]["$switch_id"]["psw"] = 10.0 # assuming a maximum active power for the switch
        data["switch"]["$switch_id"]["qsw"] = 10.0 # assuming a maximum reactive power for the switch
        data["switch"]["$switch_id"]["thermal_rating"] = 10.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
        data["switch"]["$switch_id"]["ZIL"] = true
        data["switch"]["$switch_id"]["maximum_angle"] = pi/6 # 30 degrees
    end
    return data, extremes_ZIL
end    

# Elements linked to split bus
function elements_AC_busbar_split(data)
    split_elements = Dict{String,Any}()
    split_elements["bus"] = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if haskey(b,"ZIL") && b["ZIL"] == true && b["split"] == true
            print("AC bus $(b_id) was split","\n")
            split_elements["bus"][b_id] = Dict{String,Any}()
            split_elements["bus"][b_id]["gen"] = []
            split_elements["bus"][b_id]["load"] = []
            split_elements["bus"][b_id]["branch"] = Dict{String,Any}()
            split_elements["bus"][b_id]["branch"]["f_bus"] = []
            split_elements["bus"][b_id]["branch"]["t_bus"] = []
            split_elements["bus"][b_id]["convdc"] = []
            for (g_id,g) in data["gen"]
                if g["gen_bus"] == b["index"]
                    print("Generator $(g_id) is connected to the split bus $(b_id)","\n")
                    push!(split_elements["bus"][b_id]["gen"],g_id)
                end
            end
            for (l_id,l) in data["load"]
                if l["load_bus"] == b["index"]
                    print("Load $(l_id) is connected to the split bus $(b_id)","\n")
                    push!(split_elements["bus"][b_id]["load"],l_id)
                end
            end
            for (br_id,br) in data["branch"]
                if br["f_bus"] == b["index"]
                    print("Branch $(br_id) is connected to the split f_bus $(b_id)","\n")
                    push!(split_elements["bus"][b_id]["branch"]["f_bus"],br_id)
                elseif br["t_bus"] == b["index"]
                    print("Branch $(br_id) is connected to the split t_bus $(b_id)","\n")
                    push!(split_elements["bus"][b_id]["branch"]["t_bus"],br_id)
                end
            end
            for (c_id,c) in data["convdc"]
                if c["busac_i"] == b["index"]
                    print("Convdc $(c_id) is connected to the split bus $(b_id)","\n")
                    push!(split_elements["bus"][b_id]["convdc"],c_id)
                end
            end
        end
    end
    return split_elements
end

# Function to split a set of selected DC busbars, with fixed connections
function DC_busbar_split_more_buses_fixed(data,bus_to_be_split) # one can split whatever number of ac busbars

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
        for n in bus_to_be_split
            for i in keys(data["busdc"])
                if parse(Int64,i) != n && !haskey(data["busdc"][i],"ZIL")
                    data["busdc"]["$i"]["split"] = false
                end
            end
            data["busdc"]["$n"]["split"] = true
            data["busdc"]["$n"]["ZIL"] = true
        end
    end

    # Adding a new bus to represent the split, basing on "split" == true indicated before
    n_buses_original = length(data["busdc"])
    count_ = 0
    for (b_id,b) in data["busdc"] 
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original # make sure we include only the original buses
            count_ += 1
            b["busdc_split"] = deepcopy(b["index"]) # indicating which is the original bus being split and that the bus is created from a split
            added_bus = n_buses_original + count_
            data["busdc"]["$added_bus"] = deepcopy(b)
            data["busdc"]["$added_bus"]["busdc_i"] = added_bus 
            data["busdc"]["$added_bus"]["source_id"][2] = added_bus 
            data["busdc"]["$added_bus"]["index"] = added_bus
            data["busdc"]["$added_bus"]["split"] = false
            data["busdc"]["$added_bus"]["ZIL"] = true # indicating that there is a ZIL connected to the bus
        end
    end 
    
    # Creating a dictionary with the split buses -> keys are the original bus being split, the elements of the vector are the two buses linked by the ZIL
    extremes_ZIL_dc = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if b["split"] == true && !haskey(extremes_ZIL_dc,b["busdc_split"])
            split_bus_dc = b["busdc_split"]
            extremes_ZIL_dc["$split_bus_dc"] = []
        end
    end
    for b in 1:length(data["busdc"])
        if haskey(data["busdc"]["$b"],"ZIL")
            for i in eachindex(extremes_ZIL_dc)
                if data["busdc"]["$b"]["busdc_split"] == parse(Int64,i)
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
        data["dcswitch"]["$switch_id"]["psw"] = 10.0
        data["dcswitch"]["$switch_id"]["thermal_rating"] = 10.0
        data["dcswitch"]["$switch_id"]["state"] = 1
        data["dcswitch"]["$switch_id"]["status"] = 1
        data["dcswitch"]["$switch_id"]["source_id"] = []
        push!(data["dcswitch"]["$switch_id"]["source_id"],"dcswitch")
        push!(data["dcswitch"]["$switch_id"]["source_id"],switch_id)
        data["dcswitch"]["$switch_id"]["ZIL"] = true
        data["dcswitch"]["$switch_id"]["maximum_angle"] = pi/6 # 30 degrees
    end

    return data, extremes_ZIL_dc
end    

# Elements linked to split bus
function elements_DC_busbar_split(data)
    split_elements_dc = Dict{String,Any}()
    split_elements_dc["busdc"] = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if haskey(b,"ZIL") && b["ZIL"] == true && b["split"] == true
            print("DC bus $(b_id) was split","\n")
            split_elements_dc["busdc"][b_id] = Dict{String,Any}()
            split_elements_dc["busdc"][b_id]["branchdc"] = Dict{String,Any}()
            split_elements_dc["busdc"][b_id]["branchdc"]["fbusdc"] = []
            split_elements_dc["busdc"][b_id]["branchdc"]["tbusdc"] = []
            split_elements_dc["busdc"][b_id]["convdc"] = []
            for (br_id,br) in data["branchdc"]
                if br["fbusdc"] == b["index"]
                    print("Branch dc $(br_id) is connected to the split fbusdc $(b_id)","\n")
                    push!(split_elements_dc["busdc"][b_id]["branchdc"]["fbusdc"],br_id)
                elseif br["tbusdc"] == b["index"]
                    print("Branch dc $(br_id) is connected to the split tbusdc $(b_id)","\n")
                    push!(split_elements_dc["busdc"][b_id]["branchdc"]["tbusdc"],br_id)
                end
            end
            for (c_id,c) in data["convdc"]
                if c["busdc_i"] == b["index"]
                    print("Convdc $(c_id) is connected to the split DC bus $(b_id)","\n")
                    push!(split_elements_dc["busdc"][b_id]["convdc"],c_id)
                end
            end
        end
    end
    return split_elements_dc
end

