### Switch Constraints ###
"enforces static switch constraints"
function constraint_dc_switch_state(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)

    if switch["state"] == 0
        f_idx = (i, switch["f_busdc"], switch["t_busdc"])
        constraint_dc_switch_state_open(pm, nw, f_idx)
    else
        @assert switch["state"] == 1
        constraint_dc_switch_state_closed(pm, nw, switch["f_busdc"], switch["t_busdc"])
    end
end

"enforces controlable switch constraints"
#function constraint_dc_switch_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
#    switch = _PM.ref(pm, nw, :dcswitch, i)
#
#    f_idx = (i, switch["f_busdc"], switch["t_busdc"])
#    vad_min = _PM.ref(pm, nw, :off_angmin)
#    vad_max = _PM.ref(pm, nw, :off_angmax)
#
#    constraint_dc_switch_power_on_off(pm, nw, i, f_idx)
#    constraint_dc_switch_voltage_on_off(pm, nw, i, switch["f_busdc"], switch["t_busdc"], vad_min, vad_max)
#end

"enforces an mva limit on the power flow over a switch"

function constraint_power_balance_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    bus_arcs_dcgrid = _PM.ref(pm, nw, :bus_arcs_dcgrid, i)
    bus_convs_dc = _PM.ref(pm, nw, :bus_convs_dc, i)
    pd = _PM.ref(pm, nw, :busdc, i)["Pdc"]
    constraint_power_balance_dc_ots(pm, nw, i, bus_arcs_dcgrid, bus_convs_dc, pd)
end

function constraint_power_balance_dc_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    bus_arcs_dcgrid = _PM.ref(pm, nw, :bus_arcs_dcgrid, i)
    bus_convs_dc = _PM.ref(pm, nw, :bus_convs_dc, i)
    bus_arcs_sw_dc = _PM.ref(pm, nw, :bus_arcs_sw_dc, i)
    pd = _PM.ref(pm, nw, :busdc, i)["Pdc"]
    constraint_power_balance_dc_switch(pm, nw, i, bus_arcs_dcgrid, bus_convs_dc, bus_arcs_sw_dc, pd)
end

function constraint_converter_current_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    Vmax = conv["Vmmax"]
    Imax = conv["Imax"]
    constraint_converter_current_ots(pm, nw, i, Vmax, Imax)
end

function constraint_voltage_angle_difference_ots(pm::_PM.AbstractACPModel, i::Int; nw::Int=_PM.nw_id_default)
    branch = _PM.ref(pm,nw,:branch,i)
    f_bus = branch["f_bus"]
    t_bus = branch["t_bus"]
    f_idx = (i, f_bus, t_bus)
    pair = (f_bus, t_bus)
    buspair = _PM.ref(pm, nw, :buspairs, pair)

    #if buspair["branch"] == i
        constraint_voltage_angle_difference_ots(pm, nw, i, f_idx, buspair["angmin"], buspair["angmax"])
    #end
end

function constraint_voltage_angle_difference_ots(pm::_PM.AbstractDCPModel, i::Int; nw::Int=_PM.nw_id_default)
    branch = _PM.ref(pm,nw,:branch,i)
    f_bus = branch["f_bus"]
    t_bus = branch["t_bus"]
    f_idx = (i, f_bus, t_bus)
    pair = (f_bus, t_bus)
    buspair = _PM.ref(pm, nw, :buspairs, pair)

    #if buspair["branch"] == i
        constraint_voltage_angle_difference_ots(pm, nw, i, f_idx, buspair["angmin"], buspair["angmax"])
    #end
end

function constraint_linearised_binary_variable(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    csi_binaries = 10^(-5)
    constraint_linearised_binary_variable(pm, nw, i, csi_binaries)
end

function constraint_linearised_binary_variable_DC_branch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    csi_binaries = 10^(-5)
    constraint_linearised_binary_variable_DC_branch(pm, nw, i, csi_binaries)
end

function constraint_linearised_binary_variable_DC_conv(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    csi_binaries = 10^(-5)
    constraint_linearised_binary_variable_DC_conv(pm, nw, i, csi_binaries)
end

# These need to be updated

function thermal_constraint_ots_fr(pm::_PM.AbstractACPModel, i::Int, nw::Int=_PM.nw_id_default)

    branch = _PM.ref(pm,nw,:branch,i)
    f_bus = branch["f_bus"]
    t_bus = branch["t_bus"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    g, b = _PM.calc_branch_y(branch)
    g_fr = branch["g_fr"]
    b_fr = branch["b_fr"]

    thermal_constraint_ots_fr(pm, nw, i, f_bus, t_bus, f_idx, t_idx, g, b, g_fr, b_fr)

end

function thermal_constraint_ots_to(pm::_PM.AbstractACPModel, i::Int, nw::Int=_PM.nw_id_default)

    branch = _PM.ref(pm,nw,:branch,i)
    f_bus = branch["f_bus"]
    t_bus = branch["t_bus"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    g, b = _PM.calc_branch_y(branch)
    g_to = branch["g_to"]
    b_to = branch["b_to"]

    thermal_constraint_ots_to(pm, nw, i, f_bus, t_bus, f_idx, t_idx, g, b, g_to, b_to)
end

function constraint_ohms_ots_dc_branch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    branch = _PM.ref(pm,nw,:branchdc,i)
    f_bus = branch["fbusdc"]
    t_bus = branch["tbusdc"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    p = _PM.ref(pm, nw, :dcpol)

    constraint_ohms_ots_dc_branch(pm, nw, f_bus, t_bus, f_idx, t_idx, branch["r"], p)
end

#function constraint_ohms_ots_dc_branch(pm::_PM.AbstractDCPModel, i::Int; nw::Int=_PM.nw_id_default)
#    branch = _PM.ref(pm,nw,:branchdc,i)
#    f_bus = branch["fbusdc"]
#    t_bus = branch["tbusdc"]
#    f_idx = (i, f_bus, t_bus)
#    t_idx = (i, t_bus, f_bus)
#
#    p = _PM.ref(pm, nw, :dcpol)
#
#    constraint_ohms_ots_dc_branch(pm, nw, f_bus, t_bus, f_idx, t_idx, branch["r"], p)
#end

## dc OTS
function constraint_converter_losses_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    a = conv["LossA"]
    b = conv["LossB"]
    c = conv["LossCinv"]
    plmax = conv["LossA"] + conv["LossB"] * conv["Pacrated"] + conv["LossCinv"] * (conv["Pacrated"])^2
    constraint_converter_losses_dc_ots(pm, nw, i, a, b, c, plmax)
end

function constraint_converter_losses(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    a = conv["LossA"]
    b = conv["LossB"]
    c = conv["LossCinv"]
    plmax = conv["LossA"] + conv["LossB"] * conv["Pacrated"] + conv["LossCinv"] * (conv["Pacrated"])^2
    constraint_converter_losses(pm, nw, i, a, b, c, plmax)
end

function constraint_converter_losses_dc_ots_fully_constrained(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    a = conv["LossA"]
    b = conv["LossB"]
    c = conv["LossCinv"]
    plmax = conv["LossA"] + conv["LossB"] * conv["Pacrated"] + conv["LossCinv"] * (conv["Pacrated"])^2
    constraint_converter_losses_dc_ots_fully_constrained(pm, nw, i, a, b, c, plmax)
end

function constraint_conv_filter_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    constraint_conv_filter_dc_ots(pm, nw, i, conv["bf"], Bool(conv["filter"]) )
end

function constraint_conv_transformer_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    constraint_conv_transformer_dc_ots(pm, nw, i, conv["rtf"], conv["xtf"], conv["busac_i"], conv["tm"], Bool(conv["transformer"]))
end

function constraint_branch_limit_on_off_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    branch = _PM.ref(pm, nw, :branchdc, i)
    f_bus = branch["fbusdc"]
    t_bus = branch["tbusdc"]
    f_idx = (i, f_bus, t_bus)
    t_idx = (i, t_bus, f_bus)

    pmax = branch["rateA"]
    pmin = -branch["rateA"]
    vpu = 0.8; #as taken in the variable creation
    imax = (branch["rateA"]/0.8)^2
    imin = 0
    constraint_branch_limit_on_off_dc_ots(pm, nw, i, f_idx, t_idx, pmax, pmin, imax, imin)
end

function constraint_converter_limit_on_off_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    bigM = 1.2
    conv = _PM.ref(pm, nw, :convdc, i)
    pmax = conv["Pacrated"]
    pmin = -conv["Pacrated"]
    qmax = conv["Qacrated"]
    qmin = -conv["Qacrated"]
    pmaxdc = conv["Pacrated"] * bigM
    pmindc = -conv["Pacrated"] * bigM
    imax = conv["Imax"]

    constraint_converter_limit_on_off_dc_ots(pm, nw, i, pmax, pmin, qmax, qmin, pmaxdc, pmindc, imax)
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    conv = _PM.ref(pm, nw, :convdc, i)
    constraint_conv_reactor_dc_ots(pm, nw, i, conv["rc"], conv["xc"], Bool(conv["reactor"]))
end

# Busbar splitting
function constraint_linearised_binary_variable_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    csi_binaries = 10^(-5)
    constraint_linearised_binary_variable_switch(pm, nw, i, csi_binaries)
end

function constraint_linearised_binary_variable_switch_no_ZIL(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    csi_binaries = 10^(-5)
    constraint_linearised_binary_variable_switch_no_ZIL(pm, nw, i, csi_binaries)
end

function constraint_exclusivity_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :switch_couples, i)
    #constraint_exclusivity_switch(pm, nw, switch_couple["f_sw"], switch_couple["t_sw"],switch_couple["switch_split"])
    constraint_exclusivity_switch(pm, nw, switch_couple["f_sw"], switch_couple["t_sw"])
end

function constraint_ZIL_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :switch_couples, i)
    constraint_ZIL_switch(pm, nw, switch_couple["t_sw"], switch_couple["switch_split"])
end

function constraint_ZIL_no_OTS(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :switch_couples, i)
    constraint_ZIL_no_OTS(pm, nw, switch_couple["t_sw"], switch_couple["f_sw"], switch_couple["switch_split"])
end

function constraint_ZIL_dc_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :dcswitch_couples, i)
    constraint_ZIL_dc_switch(pm, nw, switch_couple["f_sw"], switch_couple["dcswitch_split"])
end

function constraint_BS_OTS_branch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :switch_couples, i)
    switch_ = _PM.ref(pm, nw, :switch)
    branch_ = _PM.ref(pm, nw, :branch)
    single_switch = switch_[switch_couple["f_sw"]]
    branch_original = single_switch["original"]
    
    constraint_BS_OTS_branch(pm, nw, switch_couple["f_sw"],switch_couple["t_sw"], 
    (branch_[branch_original]["index"],branch_[branch_original]["f_bus"],branch_[branch_original]["t_bus"]),
    (branch_[branch_original]["index"],branch_[branch_original]["t_bus"],branch_[branch_original]["f_bus"]),
    (branch_[branch_original]["index"],branch_[branch_original]["f_bus"],branch_[branch_original]["t_bus"]),
    (branch_[branch_original]["index"],branch_[branch_original]["t_bus"],branch_[branch_original]["f_bus"]),
    single_switch,"auxiliary")
end

function constraint_BS_OTS_dcbranch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :dcswitch_couples, i)
    switch_ = _PM.ref(pm, nw, :dcswitch)
    branch_ = _PM.ref(pm, nw, :branchdc)
    single_switch = switch_[switch_couple["f_sw"]]
    #print(single_switch)
    branch_original = single_switch["original"]

    #if single_switch["auxiliary"] == "branchdc"
        constraint_BS_OTS_dcbranch(pm, nw, switch_couple["f_sw"],switch_couple["t_sw"], 
        (branch_[branch_original]["index"],branch_[branch_original]["fbusdc"],branch_[branch_original]["tbusdc"]),
        (branch_[branch_original]["index"],branch_[branch_original]["tbusdc"],branch_[branch_original]["fbusdc"]),
        (branch_[branch_original]["index"],branch_[branch_original]["fbusdc"],branch_[branch_original]["tbusdc"]),
        (branch_[branch_original]["index"],branch_[branch_original]["tbusdc"],branch_[branch_original]["fbusdc"]),
        single_switch,"auxiliary")
    #end
end

"enforces an mva limit on the power flow over a switch"
function constraint_switch_thermal_limit(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)

    if haskey(switch, "thermal_rating")
        f_idx = (i, switch["f_bus"], switch["t_bus"])
        constraint_switch_thermal_limit(pm, nw, f_idx, switch["thermal_rating"])
    end
end

function constraint_dc_switch_thermal_limit(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)

    if haskey(switch, "thermal_rating")
        f_idx = (i, switch["f_busdc"], switch["t_busdc"])
        constraint_dc_switch_thermal_limit(pm, nw, f_idx, switch["thermal_rating"])
    end
end

function constraint_switch_power_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    f_idx = (i, switch["f_bus"], switch["t_bus"])

    constraint_switch_power_on_off(pm, nw, i, f_idx)
end

function constraint_switch_power(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    f_idx = (i, switch["f_bus"], switch["t_bus"])

    constraint_switch_power(pm, nw, i, f_idx)
end

function constraint_dc_switch_power_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)
    f_idx = (i, switch["f_busdc"], switch["t_busdc"])

    constraint_dc_switch_power_on_off(pm, nw, i, f_idx)
end

function constraint_switch_voltage_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    #vad_min = _PM.ref(pm, nw, :off_angmin)
    #vad_max = _PM.ref(pm, nw, :off_angmax)

    constraint_switch_voltage_on_off(pm, nw, i, switch["f_bus"], switch["t_bus"])
end

function constraint_switch_voltage_on_off_big_M(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    
    constraint_switch_voltage_on_off_big_M(pm, nw, i, switch["f_bus"], switch["t_bus"])
end
    

function constraint_switch_voltage(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    #vad_min = _PM.ref(pm, nw, :off_angmin)
    #vad_max = _PM.ref(pm, nw, :off_angmax)

    constraint_switch_voltage(pm, nw, i, switch["f_bus"], switch["t_bus"])
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)
    #vad_min = _PM.ref(pm, nw, :Vdcmin)
    #vad_max = _PM.ref(pm, nw, :Vdcmax)

    constraint_dc_switch_voltage_on_off(pm, nw, i, switch["f_busdc"], switch["t_busdc"])
end

function constraint_dc_switch_voltage_on_off_big_M(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)
    
    constraint_dc_switch_voltage_on_off_big_M(pm, nw, i, switch["f_busdc"], switch["t_busdc"])
end
    

function constraint_exclusivity_switch_no_OTS(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :switch_couples, i)
    constraint_exclusivity_switch_no_OTS(pm, nw, switch_couple["f_sw"], switch_couple["t_sw"])
end

function constraint_voltage_angles_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    #switch_from = _PM.ref(pm, nw, :switch, switch_couple["f_sw"])
    #switch_to = _PM.ref(pm, nw, :switch, switch_couple["f_sw"])
    #switch = _PM.ref(pm, nw, :switch)
    #switch_
    bus_1_ = switch_["f_bus"]
    bus_2_ = switch_["t_bus"]

    constraint_voltage_angles_switch(pm, nw, switch_["index"], bus_1_ ,bus_2_)
end

function constraint_voltage_angles_dc_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :dcswitch_couples, i)
    switch_from = _PM.ref(pm, nw, :dcswitch, switch_couple["f_sw"])
    switch_to = _PM.ref(pm, nw, :dcswitch, switch_couple["f_sw"])
    #switch = _PM.ref(pm, nw, :switch)

    bus_1_ = switch_from["f_busdc"]
    bus_2_ = switch_to["t_busdc"]

    constraint_voltage_angles_dc_switch(pm, nw, switch_couple["f_sw"], switch_couple["t_sw"],switch_couple["dcswitch_split"],bus_1_,bus_2_)
end

function constraint_exclusivity_dc_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :dcswitch_couples, i)
    constraint_exclusivity_dc_switch(pm, nw, switch_couple["f_sw"], switch_couple["t_sw"])
end

function constraint_exclusivity_dc_switch_no_OTS(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_couple = _PM.ref(pm, nw, :dcswitch_couples, i)
    constraint_exclusivity_dc_switch_no_OTS(pm, nw, switch_couple["f_sw"], switch_couple["t_sw"], switch_couple["dcswitch_split"])
end

function constraint_power_balance_ac_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    bus = _PM.ref(pm, nw, :bus, i)
    bus_arcs = _PM.ref(pm, nw, :bus_arcs, i)
    bus_arcs_sw = _PM.ref(pm, nw, :bus_arcs_sw, i)
    bus_gens = _PM.ref(pm, nw, :bus_gens, i)
    bus_convs_ac = _PM.ref(pm, nw, :bus_convs_ac, i)
    bus_loads = _PM.ref(pm, nw, :bus_loads, i)
    bus_shunts = _PM.ref(pm, nw, :bus_shunts, i)

    pd = Dict(k => _PM.ref(pm, nw, :load, k, "pd") for k in bus_loads)
    qd = Dict(k => _PM.ref(pm, nw, :load, k, "qd") for k in bus_loads)

    gs = Dict(k => _PM.ref(pm, nw, :shunt, k, "gs") for k in bus_shunts)
    bs = Dict(k => _PM.ref(pm, nw, :shunt, k, "bs") for k in bus_shunts)

    constraint_power_balance_ac_switch(pm, nw, i, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
end

function constraint_power_balance_ac_grid_ac_switch(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    bus = _PM.ref(pm, nw, :bus, i)
    bus_arcs = _PM.ref(pm, nw, :bus_arcs, i)
    bus_arcs_sw = _PM.ref(pm, nw, :bus_arcs_sw, i)
    bus_gens = _PM.ref(pm, nw, :bus_gens, i)
    bus_loads = _PM.ref(pm, nw, :bus_loads, i)
    bus_shunts = _PM.ref(pm, nw, :bus_shunts, i)

    pd = Dict(k => _PM.ref(pm, nw, :load, k, "pd") for k in bus_loads)
    qd = Dict(k => _PM.ref(pm, nw, :load, k, "qd") for k in bus_loads)

    gs = Dict(k => _PM.ref(pm, nw, :shunt, k, "gs") for k in bus_shunts)
    bs = Dict(k => _PM.ref(pm, nw, :shunt, k, "bs") for k in bus_shunts)

    constraint_power_balance_ac_grid_ac_switch(pm, nw, i, bus_arcs, bus_arcs_sw, bus_gens, bus_loads, bus_shunts, pd, qd, gs, bs)
end


function constraint_switch_difference_voltage_angles(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)

    #f_idx = (i, switch["f_bus"], switch["t_bus"])
    constraint_switch_difference_voltage_angles(pm, nw, switch, switch["maximum_angle"])
end

function constraint_ac_switch_power(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    f_idx = (i, switch["f_bus"], switch["t_bus"])
    
    constraint_ac_switch_power(pm, nw, i, switch["f_bus"], switch["t_bus"])
end

function constraint_current_switch_thermal_limits(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :switch, i)
    #f_idx = (i, switch["f_bus"], switch["t_bus"])
    rate_sw = switch["rate_sw"]
    
    constraint_current_switch_thermal_limits(pm, nw, i, switch["f_bus"], switch["t_bus"], rate_sw, rate_sw)
end

function constraint_dc_switch_power(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)
    f_idx = (i, switch["f_busdc"], switch["t_busdc"])
    
    constraint_dc_switch_power(pm, nw, i, switch["f_busdc"], switch["t_busdc"])
end

function constraint_current_dc_switch_thermal_limits(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch = _PM.ref(pm, nw, :dcswitch, i)
    #f_idx = (i, switch["f_bus"], switch["t_bus"])
    rate_sw = switch["rate_sw"]
    
    constraint_current_dc_switch_thermal_limits(pm, nw, i, switch["f_busdc"], switch["t_busdc"], rate_sw)
end

###################### Bilinear terms reformulation ############################

function constraint_aux_switches(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    bus_1 = switch_["f_bus"]
    bus_2 = switch_["t_bus"]

    constraint_aux_switches(pm, nw, switch_["index"])
end

function constraint_aux_differences(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    bus_1 = switch_["f_bus"]
    bus_2 = switch_["t_bus"]

    constraint_aux_differences(pm, nw, switch_["index"], bus_1, bus_2)
end

function constraint_aux_dcswitches(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :dcswitch, i)
    bus_1 = switch_["f_busdc"]
    bus_2 = switch_["t_busdc"]

    constraint_aux_dcswitches(pm, nw, switch_["index"])
end

function constraint_aux_dcdifferences(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :dcswitch, i)
    bus_1 = switch_["f_busdc"]
    bus_2 = switch_["t_busdc"]

    constraint_aux_dcdifferences(pm, nw, switch_["index"], bus_1, bus_2)
end


function constraint_1_aux_voltage_angles(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    bus_1 = switch_["f_bus"]
    bus_2 = switch_["t_bus"]

    constraint_1_aux_voltage_angles(pm, nw, switch_["index"], -2*pi, 2*pi)
end

function constraint_2_aux_voltage_angles(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    bus_1 = switch_["f_bus"]
    bus_2 = switch_["t_bus"]

    constraint_2_aux_voltage_angles(pm, nw, switch_["index"], -2*pi, 2*pi)
end

function constraint_1_aux_voltage_magnitudes(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    bus_1 = switch_["f_bus"]
    bus_2 = switch_["t_bus"]

    constraint_1_aux_voltage_magnitudes(pm, nw, switch_["index"], -10.0, 10.0)
end

function constraint_2_aux_voltage_magnitudes(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :switch, i)
    bus_1 = switch_["f_bus"]
    bus_2 = switch_["t_bus"]

    constraint_2_aux_voltage_magnitudes(pm, nw, switch_["index"], -10.0, 10.0)
end

function constraint_1_aux_voltage_dc_magnitudes(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :dcswitch, i)
    bus_1 = switch_["f_busdc"]
    bus_2 = switch_["t_busdc"]

    constraint_1_aux_voltage_dc_magnitudes(pm, nw, switch_["index"], -10.0, 10.0)
end

function constraint_2_aux_voltage_dc_magnitudes(pm::_PM.AbstractPowerModel, i::Int; nw::Int=_PM.nw_id_default)
    switch_ = _PM.ref(pm, nw, :dcswitch, i)
    bus_1 = switch_["f_busdc"]
    bus_2 = switch_["t_busdc"]

    constraint_2_aux_voltage_dc_magnitudes(pm, nw, switch_["index"], -10.0, 10.0)
end