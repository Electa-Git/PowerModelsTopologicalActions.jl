## TO BE FIXED, DO NOT USE ###

export run_acdcsw_AC_DC_fixed
export run_acdcsw_AC_DC_fixed_diff

# DC Busbar splitting for AC/DC grid
"ACDC opf with controllable switches in DC busbar splitting configuration for AC/DC grids"
function run_acdcsw_AC_DC_fixed(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_DC_fixed; ref_extensions=[add_ref_dcgrid_dcswitch!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end


""
function build_acdcsw_AC_DC_fixed(pm::_PM.AbstractPowerModel) # this model combines what defined in the build_acdcsw_AC and build_acdcsw_DC models, please refer to those functions to have more detailed explanation of those formulations
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator(pm)
    variable_switch_power(pm)

    # DC grid
    variable_dc_switch_indicator(pm)
    variable_dc_switch_power(pm)

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
        constraint_power_balance_ac_switch(pm, i)
    end


    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i)
        constraint_switch_voltage_on_off(pm,i)
        constraint_switch_power_on_off(pm,i)
    end

    for i in _PM.ids(pm, :dcswitch)
        constraint_dc_switch_thermal_limit(pm, i)
        constraint_dc_switch_voltage_on_off(pm,i)
        constraint_dc_switch_power_on_off(pm,i)
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
        constraint_power_balance_dc_switch(pm, i)
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


function run_acdcsw_AC_DC_fixed_diff(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_DC_fixed_diff; ref_extensions=[add_ref_dcgrid_dcswitch!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end


""
function build_acdcsw_AC_DC_fixed_diff(pm::_PM.AbstractPowerModel) # this model combines what defined in the build_acdcsw_AC and build_acdcsw_DC models, please refer to those functions to have more detailed explanation of those formulations
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator(pm)
    variable_switch_power(pm)

    # DC grid
    variable_dc_switch_indicator(pm)
    variable_dc_switch_power(pm)

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
        constraint_power_balance_ac_switch(pm, i)
    end


    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i)
        constraint_switch_voltage_on_off(pm,i)
        constraint_switch_power_on_off(pm,i)
        constraint_switch_difference_voltage_angles(pm,i)
    end

    for i in _PM.ids(pm, :dcswitch)
        constraint_dc_switch_thermal_limit(pm, i)
        constraint_dc_switch_voltage_on_off(pm,i)
        constraint_dc_switch_power_on_off(pm,i)
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
        constraint_power_balance_dc_switch(pm, i)
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

