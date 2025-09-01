## TO BE FIXED, DO NOT USE ###


export run_acdcsw_AC_current

# AC Busbar splitting for AC/DC grid
"ACDC opf with controllable switches in AC busbar splitting configuration for AC/DC grids"
function run_acdcsw_AC_current(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_current; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_AC_current(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator(pm) # binary variable to indicate the status of an ac switch
    variable_switch_power(pm) # variable to indicate the power flowing through an ac switch (if closed)
    variable_switch_current(pm) # variable to represent the current through the AC switch


    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)


    # Objective function
    _PM.objective_min_fuel_cost(pm)

    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_switch(pm, i) # including the ac switches in the power balance of the AC part of an AC/DC grid
    end

    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i) # limiting the apparent power flowing through an ac switch
        constraint_switch_voltage_on_off(pm,i) # making sure that the voltage magnitude and angles are equal at the two extremes of a closed switch
        constraint_switch_power_on_off(pm,i) # limiting the maximum active and reactive power through an ac switch
        constraint_ac_switch_power(pm,i)
        constraint_current_switch_thermal_limits(pm,i)
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_branch(pm,i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

    for i in _PM.ids(pm, :dcline)
        _PM.constraint_dcline_power_losses(pm, i)
    end

    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        _PMACDC.constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end

""