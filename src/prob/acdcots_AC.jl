export run_acdcots_AC

"""
    run_acdcots_AC(file::String, model_type::Type, solver; kwargs...)

Solve an AC/DC Optimal Transmission Switching (OTS) problem with AC branch switching enabled.

This function formulates and solves an OTS problem for hybrid AC/DC power systems where
only AC transmission lines can be switched on/off. The DC grid topology remains fixed.

# Arguments
- `file::String`: Path to the power system data file (typically MATPOWER .m format)
- `model_type::Type`: PowerModels formulation type (e.g., ACPPowerModel, SOCWRPowerModel)
- `solver`: JuMP-compatible optimization solver (typically mixed-integer solver like Juniper)
- `kwargs...`: Additional keyword arguments passed to the solver

# Returns
- `result::Dict`: Solution dictionary containing optimal switching decisions and power flows

# Example
```julia
using PowerModelsTopologicalActionsII, PowerModels, PowerModelsACDC
using Ipopt, Juniper

# Setup solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt)

# Solve OTS problem
result = run_acdcots_AC("case5_acdc.m", ACPPowerModel, juniper)
```
"""
function run_acdcots_AC(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

"""
    run_acdcots_AC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)

Solve an AC/DC OTS problem with AC switching using pre-parsed network data.

# Arguments
- `data::Dict{String,Any}`: Parsed power system data dictionary
- `model_type::Type`: PowerModels formulation type
- `solver`: JuMP-compatible optimization solver
- `kwargs...`: Additional keyword arguments

# Returns
- `result::Dict`: Solution dictionary with optimal switching decisions
"""
function run_acdcots_AC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

"""
    build_acdcots_AC(pm::AbstractPowerModel)

Build the optimization model for AC/DC OTS with AC branch switching.

This function constructs the complete mathematical formulation including:
- AC bus voltage variables (with on/off capability)
- Generator power variables
- AC and DC branch power flow variables
- Binary switching variables for AC branches
- Power balance constraints for AC and DC systems
- Ohm's law constraints with switching logic
- Thermal limits and voltage constraints
- Converter operation constraints

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure containing network data and formulation
"""
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
