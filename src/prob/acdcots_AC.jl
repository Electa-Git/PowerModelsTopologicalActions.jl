export run_acdcots_AC

## AC formulation of the AC/DC OTS with only AC branches that can be switched ##
""
function run_acdcots_AC(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

""
function run_acdcots_AC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcots_AC(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage_on_off(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    _PM.variable_branch_indicator(pm)

    _PM.objective_min_fuel_cost(pm)

    _PM.constraint_model_voltage_on_off(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from_on_off(pm, i)
        _PM.constraint_ohms_yt_to_on_off(pm, i)

        _PM.constraint_voltage_angle_difference_on_off(pm,i)

        _PM.constraint_thermal_limit_from_on_off(pm,i)
        _PM.constraint_thermal_limit_to_on_off(pm,i)
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
