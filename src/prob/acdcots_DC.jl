export run_acdcots_DC

## AC formulation of the AC/DC OTS with only DC branches that can be switched ##

""
function run_acdcots_DC(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_DC(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

""
function run_acdcots_DC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_DC; ref_extensions = [_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcots_DC(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # OTS variables for DC grid
    variable_dc_branch_indicator(pm)
    variable_dc_conv_indicator(pm)
    variable_voltage_slack_ots(pm)

    _PM.objective_min_fuel_cost(pm)

    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    #constraint_voltage_dc_ots(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i) #angle difference across transformer and reactor - useful for LPAC if available?
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end
    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        constraint_ohms_ots_dc_branch(pm, i)
        constraint_branch_limit_on_off_dc_ots(pm,i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses_dc_ots(pm, i)
        constraint_converter_current_ots(pm,i)
        constraint_conv_transformer_dc_ots(pm, i)
        constraint_conv_reactor_dc_ots(pm, i)
        constraint_conv_filter_dc_ots(pm, i)
        constraint_converter_limit_on_off_dc_ots(pm,i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end

