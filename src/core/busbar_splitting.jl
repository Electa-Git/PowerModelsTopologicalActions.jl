
# Function to split the selected AC busbar and create an augmented network representation of the grid with switches linking every grid element to the two parts of the splitted busbar 
function AC_busbar_split(data,bus_to_be_split) # only possible to split one busbar

    # Adding a new key to indicate which bus can be split, choosing it for now -> to be updated
    for i in keys(data["bus"])
        if parse(Int64,i) != bus_to_be_split
            data["bus"]["$i"]["split"] = false
        end
    end
    data["bus"]["$bus_to_be_split"]["split"] = true
    data["bus"]["$bus_to_be_split"]["ZIL"] = true
    
    # Adding a bus next to the split
    n_buses_original = length(data["bus"])
    count_ = 0
    for (b_id,b) in data["bus"] 
        if haskey(b,"ZIL") && parse(Int64,b_id) <= n_buses_original
            count_ += 1
            b["bus_split"] = deepcopy(b["index"])
            added_bus = n_buses_original + count_
            data["bus"]["$added_bus"] = deepcopy(b)
            data["bus"]["$added_bus"]["bus_i"] = added_bus 
            data["bus"]["$added_bus"]["source_id"][2] = added_bus 
            data["bus"]["$added_bus"]["index"] = added_bus
            data["bus"]["$added_bus"]["split"] = false
            data["bus"]["$added_bus"]["ZIL"] = true
        end
    end 
    
    # Creating a dictionary with the split buses
    extremes_ZIL = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if b["split"] == true && !haskey(extremes_ZIL,b["bus_split"])
            split_bus = b["bus_split"]
            extremes_ZIL["$split_bus"] = []
        end
        if haskey(b,"ZIL")
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    push!(extremes_ZIL[i],b["index"])
                end
            end
        end
    end
    
    
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses
    switch_id = 0
    for i in eachindex(extremes_ZIL)
        switch_id += 1
        data["switch"]["$switch_id"] = Dict{String,Any}()
        #data["switch"]["$switch_id"] = deepcopy(data_sw["switch"]["1"])
        data["switch"]["$switch_id"]["f_bus"] = deepcopy(extremes_ZIL[i][1]) 
        data["switch"]["$switch_id"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
        data["switch"]["$switch_id"]["index"] = switch_id
        data["switch"]["$switch_id"]["psw"] = 10.0
        data["switch"]["$switch_id"]["qsw"] = 10.0
        data["switch"]["$switch_id"]["thermal_rating"] = 10.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
    end
    
    # Add a bus for each grid element
    # Gen
    for (g_id,g) in data["gen"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if g["gen_bus"] == parse(Int64,i)
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true
                data["bus"]["$added_gen_bus"]["original"] = parse(Int64,g_id)  
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen"
                data["bus"]["$added_gen_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_gen_bus"]["split"] = false
                g["gen_bus"] = added_gen_bus
            end
        end
    end 
    
    # Load
    for (l_id,l) in data["load"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_load_bus"]["original"] = parse(Int64,l_id)  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch
    for (br_id,br) in data["branch"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["original"] = parse(Int64,br_id)
                data["bus"]["$added_branch_bus"]["bus_split"] = parse(Int64,i) 
                if haskey(data["bus"]["$added_branch_bus"],"split")
                    delete!(data["bus"]["$added_branch_bus"],"split")
                end
                br["f_bus"] = added_branch_bus
            elseif br["t_bus"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["original"] = parse(Int64,br_id) 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_branch_bus"]["split"] = false
                br["t_bus"] = added_branch_bus
            end
        end
    end

    # Converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if cv["busac_i"] == parse(Int64,i)
                added_conv_bus = n_buses + 1
                data["bus"]["$added_conv_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_conv_bus"]["bus_type"] = 1
                data["bus"]["$added_conv_bus"]["bus_i"] = added_conv_bus 
                data["bus"]["$added_conv_bus"]["index"] = added_conv_bus 
                data["bus"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                data["bus"]["$added_conv_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_conv_bus"]["original"] = parse(Int64,cv_id) 
                data["bus"]["$added_conv_bus"]["auxiliary"] = "convdc"
                data["bus"]["$added_conv_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_conv_bus"]["split"] = false
                cv["busac_i"] = added_conv_bus
            end
        end
        end
    end
    
    # Linking the auxiliary buses to both split buses with switches
    for (b_id,b) in data["bus"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == true
            for i in eachindex(extremes_ZIL)
                number_switches = length(data["switch"])
                added_switch = number_switches + 1
                data["switch"]["$added_switch"] = deepcopy(data["switch"]["1"])
                data["switch"]["$added_switch"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                data["switch"]["$added_switch"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                data["switch"]["$added_switch"]["index"] = added_switch 
                data["switch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                data["switch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["switch"]["$added_switch"]["original"] = deepcopy(b["original"]) 
                data["switch"]["$added_switch"]["bus_split"] = parse(Int64,i) 
                data["switch"]["$added_switch"]["index"] = added_switch 
            end
        end
        # Connecting to the second bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == false && b["auxiliary"] == "gen"
            for i in eachindex(extremes_ZIL)
                number_switches = length(data["switch"])
                added_switch = number_switches + 1
                data["switch"]["$added_switch"] = deepcopy(data["switch"]["1"])
                data["switch"]["$added_switch"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                data["switch"]["$added_switch"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                data["switch"]["$added_switch"]["index"] = added_switch
                data["switch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["switch"]["$added_switch"]["bus_split"] = parse(Int64,i) 
                data["switch"]["$added_switch"]["original"] = deepcopy(b["original"])  
                data["switch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
            end
        end
    end

    switch_couples = compute_couples_of_switches(data)
    data["switch_couples"] = Dict{String,Any}()
    data["switch_couples"] = deepcopy(switch_couples)

    return data, switch_couples, extremes_ZIL
end

# Stating the from and to bus of every AC switch
function compute_couples_of_switches(data)
    switch_couples = Dict{String,Any}() # creating a dictionary to check the couples of switches linking each grid element to both parts of the split busbar
    t_sws = []
    for (sw_id,sw) in data["switch"]
        for l in keys(data["switch"])
            if (haskey(sw, "auxiliary") && haskey(data["switch"][l], "auxiliary")) && (sw["auxiliary"] == data["switch"][l]["auxiliary"]) && (sw["original"] == data["switch"][l]["original"]) && (sw["index"] != data["switch"][l]["index"]) &&  (sw["bus_split"] == data["switch"][l]["bus_split"])
                if !issubset(sw["index"],t_sws) 
                    switch_couples["$sw_id"] = Dict{String,Any}()
                    switch_couples["$sw_id"]["f_sw"] = sw["index"]
                    switch_couples["$sw_id"]["t_sw"] = data["switch"][l]["index"]
                    switch_couples["$sw_id"]["bus_split"] = data["switch"][l]["bus_split"]
                    for (s_id,s) in data["switch"] 
                        if !(haskey(s, "auxiliary")) && haskey(s,"ZIL") && s["bus_split"] == switch_couples["$sw_id"]["bus_split"]
                            switch_couples["$sw_id"]["switch_split"] = deepcopy(s["index"])
                        end
                    end
                    push!(t_sws,switch_couples["$sw_id"]["t_sw"])
                end
            end
        end
    end 
    return switch_couples
end

#  Stating the from and to bus of every DC switch
function compute_couples_of_dcswitches(data)
    switch_couples = Dict{String,Any}()
    t_sws = []
    for (sw_id,sw) in data["dcswitch"]
        for l in keys(data["dcswitch"])
            if (haskey(sw, "auxiliary") && haskey(data["dcswitch"][l], "auxiliary")) && (sw["auxiliary"] == data["dcswitch"][l]["auxiliary"]) && (sw["original"] == data["dcswitch"][l]["original"]) && (sw["index"] != data["dcswitch"][l]["index"]) && (sw["busdc_split"] == data["dcswitch"][l]["busdc_split"])
                if !issubset(sw["index"],t_sws) 
                    switch_couples["$sw_id"] = Dict{String,Any}()
                    switch_couples["$sw_id"]["f_sw"] = sw["index"]
                    switch_couples["$sw_id"]["t_sw"] = data["dcswitch"][l]["index"]
                    switch_couples["$sw_id"]["busdc_split"] = data["dcswitch"][l]["busdc_split"]
                    for (s_id,s) in data["dcswitch"] 
                        if !(haskey(s, "auxiliary")) && haskey(s,"ZIL") && s["busdc_split"] == switch_couples["$sw_id"]["busdc_split"]
                            switch_couples["$sw_id"]["dcswitch_split"] = deepcopy(s["index"])
                        end
                    end
                    push!(t_sws,switch_couples["$sw_id"]["t_sw"])
                end
            end
        end
    end
    return switch_couples
end

# Function to split the selected DC busbar and create an augmented network representation of the grid with switches linking every grid element to the two parts of the splitted busbar 
function DC_busbar_split(data,bus_to_be_split)

    # Adding a new key to indicate which bus can be split, choosing it for now -> to be updated
    for i in keys(data["busdc"])
        if parse(Int64,i) != bus_to_be_split
            data["busdc"]["$i"]["split"] = false
        end
    end
    data["busdc"]["$bus_to_be_split"]["split"] = true
    data["busdc"]["$bus_to_be_split"]["ZIL"] = true
    
    # Adding a busdc next to the split
    n_buses_original_dc = length(data["busdc"])
    count_dc = 0
    for (b_id,b) in data["busdc"] 
        if haskey(b,"ZIL") && parse(Int64,b_id) <= n_buses_original_dc
            count_dc += 1
            b["busdc_split"] = deepcopy(b["index"])
            added_bus_dc = n_buses_original_dc + count_dc
            data["busdc"]["$added_bus_dc"] = deepcopy(b)
            data["busdc"]["$added_bus_dc"]["index"] = added_bus_dc 
            data["busdc"]["$added_bus_dc"]["source_id"][2] = added_bus_dc 
            data["busdc"]["$added_bus_dc"]["busdc_i"] = added_bus_dc
            data["busdc"]["$added_bus_dc"]["split"] = false
            data["busdc"]["$added_bus_dc"]["ZIL"] = true
        end
    end 

    
    # Creating a dictionary with the split buses
    extremes_ZIL_dc = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if b["split"] == true && !haskey(extremes_ZIL_dc,b["busdc_split"])
            split_bus_dc = b["busdc_split"]
            extremes_ZIL_dc["$split_bus_dc"] = []
        end
        if haskey(b,"ZIL")
            for i in eachindex(extremes_ZIL_dc)
                if b["busdc_split"] == parse(Int64,i)
                    push!(extremes_ZIL_dc[i],b["index"])
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
        #data["switch"]["$switch_id"] = deepcopy(data_sw["switch"]["1"])
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
    end

    #=
    # Add a busdc for each grid element
    # Gen_dc
    for (g_id,g) in data["gen"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if g["gen_bus"] == parse(Int64,i)
                added_gen_bus = n_buses + 1
                data["busdc"]["$added_gen_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_gen_bus"]["busdc_i"] = added_gen_bus 
                data["busdc"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["busdc"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["busdc"]["$added_gen_bus"]["auxiliary_bus"] = true
                data["busdc"]["$added_gen_bus"]["auxiliary"] = "gen"
                data["busdc"]["$added_gen_bus"]["original"] = parse(Int64,g_id)  
                delete!(data["busdc"]["$added_gen_bus"],"ZIL")
                if haskey(data["busdc"]["$added_gen_bus"],"split")
                    delete!(data["busdc"]["$added_gen_bus"],"split")
                end

                g["gen_bus"] = added_gen_bus
            end
        end
    end 
    
    # Load_dc
    for (l_id,l) in data["load"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL_dc)
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["busdc"]["$added_load_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_load_bus"]["busdc_i"] = added_load_bus 
                data["busdc"]["$added_load_bus"]["index"] = added_load_bus 
                data["busdc"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["busdc"]["$added_load_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_load_bus"]["original"] = parse(Int64,l_id)  
                data["busdc"]["$added_load_bus"]["auxiliary"] = "load"
                delete!(data["busdc"]["$added_load_bus"],"ZIL")
                if haskey(data["busdc"]["$added_load_bus"],"split")
                    delete!(data["busdc"]["$added_load_bus"],"split")
                end
                l["load_bus"] = added_load_bus
            end
        end
    end 
    =#
    # Branch dc
    for (br_id,br) in data["branchdc"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if br["f_busdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["original"] = parse(Int64,br_id)
                data["busdc"]["$added_branch_bus"]["busdc_split"] = parse(Int64,i) 
                if haskey(data["busdc"]["$added_branch_bus"],"split")
                    delete!(data["busdc"]["$added_branch_bus"],"split")
                end
                br["f_busdc"] = added_branch_bus
            elseif br["t_busdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["original"] = parse(Int64,br_id) 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["busdc_split"] = parse(Int64,i) 
                data["busdc"]["$added_branch_bus"]["split"] = false
                br["t_busdc"] = added_branch_bus
            end
        end
    end

    # Converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if cv["busdc_i"] == parse(Int64,i)
                added_conv_bus = n_buses + 1
                data["busdc"]["$added_conv_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_conv_bus"]["bus_type"] = 1
                data["busdc"]["$added_conv_bus"]["busdc_i"] = added_conv_bus 
                data["busdc"]["$added_conv_bus"]["index"] = added_conv_bus 
                data["busdc"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                data["busdc"]["$added_conv_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_conv_bus"]["original"] = parse(Int64,cv_id) 
                data["busdc"]["$added_conv_bus"]["auxiliary"] = "convdc"
                data["busdc"]["$added_conv_bus"]["busdc_split"] = parse(Int64,i) 
                data["busdc"]["$added_conv_bus"]["split"] = false
                delete!(data["busdc"]["$added_conv_bus"],"ZIL")
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
                number_switches = length(data["dcswitch"])
                added_switch = number_switches + 1
                data["dcswitch"]["$added_switch"] = deepcopy(data["dcswitch"]["1"])
                data["dcswitch"]["$added_switch"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                data["dcswitch"]["$added_switch"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][1])
                data["dcswitch"]["$added_switch"]["index"] = added_switch 
                data["dcswitch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                data["dcswitch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["dcswitch"]["$added_switch"]["original"] = deepcopy(b["original"]) 
                data["dcswitch"]["$added_switch"]["index"] = added_switch 
            end
        end
        # Connecting to the second bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == false && b["auxiliary"] == "gen"
            for i in eachindex(extremes_ZIL_dc)
                number_switches = length(data["dcswitch"])
                added_switch = number_switches + 1
                data["dcswitch"]["$added_switch"] = deepcopy(data["dcswitch"]["1"])
                data["dcswitch"]["$added_switch"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                data["dcswitch"]["$added_switch"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][2])
                data["dcswitch"]["$added_switch"]["index"] = added_switch
                data["dcswitch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["dcswitch"]["$added_switch"]["original"] = deepcopy(b["original"])  
                data["dcswitch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
            end
        end
    end

    dcswitch_couples = compute_couples_of_dcswitches(data)
    data["dcswitch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = deepcopy(dcswitch_couples)
    return data, dcswitch_couples, extremes_ZIL_dc
end

# Function to split a set of selected AC busbars and create an augmented network representation of the grid with switches linking every grid element to the two parts of the splitted busbars 
function AC_busbar_split_more_buses(data,bus_to_be_split) # one can split whatever number of ac busbars
    # Adding a new key to indicate which bus can be split + ZIL
    if length(bus_to_be_split) == 1
        for i in eachindex(data["bus"])
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
            split_bus = b["bus_split"]
            extremes_ZIL["$split_bus"] = []
        end
    end
    buses_ = []
    for (b_id,b) in data["bus"]
        push!(buses_,b["index"])
    end
    buses_ordered = sort(buses_)
    for b in buses_ordered#1:length(data["bus"])
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
        data["switch"]["$switch_id"]["psw"] = 40.0 # assuming a maximum active power for the switch
        data["switch"]["$switch_id"]["qsw"] = 40.0 # assuming a maximum reactive power for the switch
        data["switch"]["$switch_id"]["thermal_rating"] = 0.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
        data["switch"]["$switch_id"]["ZIL"] = true
    end
     
    # Add a bus for each grid element connected to the bus being split
    # Gen
    for (g_id,g) in data["gen"] 
        n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            if g["gen_bus"] == parse(Int64,i) # isolating the generators connected to the split busbar. Adding new buses in PowerModels format
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["1"])
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
        n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["1"])
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
        n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
        for i in eachindex(extremes_ZIL)
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") # making sure this is not a ZIL, f_bus part. Redundant if, but useful if one decides to model the ZIL as a branch and not as a switch. 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
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
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
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
            n_buses = maximum([bus["index"] for (b, bus) in data["bus"]]) # number of buses before starting to create new buses to link all the generators attached to the bus being split
            for i in eachindex(extremes_ZIL)
                if cv["busac_i"] == parse(Int64,i)
                    added_conv_bus = n_buses + 1
                    data["bus"]["$added_conv_bus"] = deepcopy(data["bus"]["1"])
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
        # Connecting to the first bus in the extremes_ZIL keys
        if haskey(b,"auxiliary_bus") # filtering out the non-auxiliary buses
            for i in eachindex(extremes_ZIL)
                # Connecting to the first part of the substation being split
                if b["bus_split"] == extremes_ZIL[i][1] # linking the auxiliary bus to the original split substation to which the grid element was attached before splitting. Redundant if there is only one substation being split
                    number_switches = length(data["switch"])
                    added_switch = number_switches + 1
                    data["switch"]["$added_switch"] = deepcopy(data["switch"]["1"]) # defining the same elements as before
                    data["switch"]["$added_switch"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                    data["switch"]["$added_switch"]["index"] = added_switch 
                    data["switch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                    data["switch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch"]["original"] = deepcopy(b["original"]) 
                    data["switch"]["$added_switch"]["bus_split"] = deepcopy(extremes_ZIL[i][1])
                    data["switch"]["$added_switch"]["ZIL"] = false
                end
                # Connecting to the second part of the substation being split
                if b["bus_split"] == extremes_ZIL[i][1] 
                    number_switches = length(data["switch"])
                    added_switch = number_switches + 1
                    data["switch"]["$added_switch"] = deepcopy(data["switch"]["1"]) # defining the same elements as before
                    data["switch"]["$added_switch"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                    data["switch"]["$added_switch"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                    data["switch"]["$added_switch"]["index"] = added_switch
                    data["switch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                    data["switch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["switch"]["$added_switch"]["original"] = deepcopy(b["original"])  
                    data["switch"]["$added_switch"]["bus_split"] = deepcopy(extremes_ZIL[i][1])
                    data["switch"]["$added_switch"]["ZIL"] = false
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

# Function to split a set of selected DC busbars and create an augmented network representation of the grid with switches linking every grid element to the two parts of the splitted busbars 
function DC_busbar_split_more_buses(data,bus_to_be_split) # one can split whatever number of dc busbars. Same reasoning as the ac busbar version
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

    # Adding a busdc next to the split
    n_buses_original_dc = length(data["busdc"])
    count_dc = 0
    for (b_id,b) in data["busdc"] 
        if haskey(b,"ZIL") && parse(Int64,b_id) <= n_buses_original_dc
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
    end

    # Add a busdc for each grid element, same keys as for the ac busbars

    # Gen and loads on the DC part can be added, but we do not have any for now 
    #=
    # Gen_dc
    for (g_id,g) in data["gen"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if g["gen_bus"] == parse(Int64,i)
                added_gen_bus = n_buses + 1
                data["busdc"]["$added_gen_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_gen_bus"]["busdc_i"] = added_gen_bus 
                data["busdc"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["busdc"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["busdc"]["$added_gen_bus"]["auxiliary_bus"] = true
                data["busdc"]["$added_gen_bus"]["auxiliary"] = "gen"
                data["busdc"]["$added_gen_bus"]["original"] = parse(Int64,g_id)  
                delete!(data["busdc"]["$added_gen_bus"],"ZIL")
                if haskey(data["busdc"]["$added_gen_bus"],"split")
                    delete!(data["busdc"]["$added_gen_bus"],"split")
                end

                g["gen_bus"] = added_gen_bus
            end
        end
    end 
    
    # Load_dc
    for (l_id,l) in data["load"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL_dc)
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["busdc"]["$added_load_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_load_bus"]["busdc_i"] = added_load_bus 
                data["busdc"]["$added_load_bus"]["index"] = added_load_bus 
                data["busdc"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["busdc"]["$added_load_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_load_bus"]["original"] = parse(Int64,l_id)  
                data["busdc"]["$added_load_bus"]["auxiliary"] = "load"
                delete!(data["busdc"]["$added_load_bus"],"ZIL")
                if haskey(data["busdc"]["$added_load_bus"],"split")
                    delete!(data["busdc"]["$added_load_bus"],"split")
                end
                l["load_bus"] = added_load_bus
            end
        end
    end 
    =#
    # Branch dc
    for (br_id,br) in data["branchdc"] 
        for i in eachindex(extremes_ZIL_dc)
            n_buses = length(data["busdc"])
            if br["fbusdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["original"] = parse(Int64,br_id)
                data["busdc"]["$added_branch_bus"]["busdc_split"] = parse(Int64,i) 
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
                data["busdc"]["$added_branch_bus"]["original"] = parse(Int64,br_id) 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["busdc_split"] = parse(Int64,i) 
                data["busdc"]["$added_branch_bus"]["split"] = false
                br["tbusdc"] = added_branch_bus
            end
        end
    end

    # Converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
            n_buses = length(data["busdc"])
            for i in eachindex(extremes_ZIL_dc)
                if cv["busdc_i"] == parse(Int64,i)
                    added_conv_bus = n_buses + 1
                    data["busdc"]["$added_conv_bus"] = deepcopy(data["busdc"]["1"])
                    data["busdc"]["$added_conv_bus"]["bus_type"] = 1
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
                if b["busdc_split"] == extremes_ZIL_dc[i][1] 
                    number_switches = length(data["dcswitch"])
                    added_switch = number_switches + 1
                    data["dcswitch"]["$added_switch"] = deepcopy(data["dcswitch"]["1"])
                    data["dcswitch"]["$added_switch"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                    data["dcswitch"]["$added_switch"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][1])
                    data["dcswitch"]["$added_switch"]["index"] = added_switch 
                    data["dcswitch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                    data["dcswitch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["dcswitch"]["$added_switch"]["original"] = deepcopy(b["original"]) 
                    data["dcswitch"]["$added_switch"]["busdc_split"] = deepcopy(extremes_ZIL_dc[i][1])
                    data["dcswitch"]["$added_switch"]["ZIL"] = false
                end
                if b["busdc_split"] == extremes_ZIL_dc[i][1] 
                    number_switches = length(data["dcswitch"])
                    added_switch = number_switches + 1
                    data["dcswitch"]["$added_switch"] = deepcopy(data["dcswitch"]["1"])
                    data["dcswitch"]["$added_switch"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                    data["dcswitch"]["$added_switch"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][2])
                    data["dcswitch"]["$added_switch"]["index"] = added_switch
                    data["dcswitch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                    data["dcswitch"]["$added_switch"]["original"] = deepcopy(b["original"])  
                    data["dcswitch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                    data["dcswitch"]["$added_switch"]["busdc_split"] = deepcopy(extremes_ZIL_dc[i][1])
                    data["dcswitch"]["$added_switch"]["ZIL"] = false  
                end
            end
        end
    end

    dcswitch_couples = compute_couples_of_dcswitches(data)
    data["dcswitch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = deepcopy(dcswitch_couples)
    return data, dcswitch_couples, extremes_ZIL_dc
end
