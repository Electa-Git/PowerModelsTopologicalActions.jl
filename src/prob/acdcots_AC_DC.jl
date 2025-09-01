export run_acdcots_AC_DC

"""
    run_acdcots_AC_DC(file::String, model_type::Type, solver; kwargs...)

Solve an AC/DC Optimal Transmission Switching (OTS) problem with both AC and DC switching.

This function formulates and solves a comprehensive OTS problem for hybrid AC/DC power 
systems where both AC transmission lines and DC branches/converters can be switched 
on/off simultaneously. This provides the maximum flexibility for topological optimization.

# Arguments
- `file::String`: Path to the power system data file (typically MATPOWER .m format)
- `model_type::Type`: PowerModels formulation type (e.g., ACPPowerModel, SOCWRPowerModel)
- `solver`: JuMP-compatible optimization solver (typically mixed-integer solver like Juniper)
- `kwargs...`: Additional keyword arguments passed to the solver

# Returns
- `result::Dict`: Solution dictionary containing optimal AC and DC switching decisions

# Example
```julia
using PowerModelsTopologicalActionsII, PowerModels, PowerModelsACDC
using Ipopt, Juniper

# Setup solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt)

# Solve combined AC/DC OTS problem
result = run_acdcots_AC_DC("case5_acdc.m", ACPPowerModel, juniper)
```
"""
function run_acdcots_AC_DC(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC_DC(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

"""
    run_acdcots_AC_DC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)

Solve a combined AC/DC OTS problem using pre-parsed network data.

# Arguments
- `data::Dict{String,Any}`: Parsed power system data dictionary
- `model_type::Type`: PowerModels formulation type
- `solver`: JuMP-compatible optimization solver
- `kwargs...`: Additional keyword arguments

# Returns
- `result::Dict`: Solution dictionary with optimal AC and DC switching decisions
"""
function run_acdcots_AC_DC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC_DC; ref_extensions = [_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

"""
    build_acdcots_AC_DC(pm::AbstractPowerModel)

Build the optimization model for combined AC/DC OTS with full switching capability.

This function constructs the most comprehensive mathematical formulation including:
- AC bus voltage variables with on/off capability
- AC branch power flow variables with switching logic
- DC branch power flow variables with switching capability
- DC converter variables with on/off states
- Binary switching variables for both AC and DC elements
- Power balance constraints for both AC and DC systems
- Modified Ohm's law constraints for all switchable elements
- Converter operation constraints with switching logic
- Thermal limits and voltage constraints for all components

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure containing network data and formulation
"""
function build_acdcots_AC_DC(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage_on_off(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)
    _PM.variable_branch_indicator(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    variable_dc_branch_indicator(pm)
    variable_dc_conv_indicator(pm)
    variable_voltage_slack_ots(pm)

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
        constraint_ohms_ots_dc_branch(pm, i)
        constraint_branch_limit_on_off_dc_ots(pm,i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses_dc_ots(pm, i)
        constraint_converter_current_ots(pm,i)
        constraint_conv_transformer_dc_ots(pm, i)
        constraint_conv_reactor_dc_ots(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        constraint_converter_limit_on_off_dc_ots(pm,i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end
