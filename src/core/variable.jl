"""
    variable_bus_voltage_angle_sp(pm::AbstractPowerModel; nw::Int, bounded::Bool=true, report::Bool=true)

Create bus voltage angle variables for busbar splitting (sp) formulations.

Creates voltage angle variables `va[i]` for each bus `i` in the network, specifically
for busbar splitting problems where bus sections may have different voltage angles.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `nw::Int`: Network identifier (default: nw_id_default)
- `bounded::Bool`: Whether to apply angle bounds (default: true)
- `report::Bool`: Whether to include in solution reporting (default: true)

# Variables Created
- `va[i]`: Voltage angle at bus `i` (radians)

# Notes
The `_sp` suffix indicates this is for busbar splitting formulations where
individual bus sections may need separate voltage angle tracking.
"""
function variable_bus_voltage_angle_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    va = _PM.var(pm, nw)[:va] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :bus)], base_name="$(nw)_va",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :bus, i), "va_starting_value")
    )
    
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :bus, :va, _PM.ids(pm, nw, :bus), va)
end

"""
    variable_bus_voltage_magnitude_sp(pm::AbstractPowerModel; nw::Int, bounded::Bool=true, report::Bool=true)

Create bus voltage magnitude variables for busbar splitting formulations.

Creates voltage magnitude variables `vm[i]` for each bus `i`, with bounds applied
based on the bus voltage limits from the network data.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `nw::Int`: Network identifier (default: nw_id_default)
- `bounded::Bool`: Whether to apply voltage magnitude bounds (default: true)
- `report::Bool`: Whether to include in solution reporting (default: true)

# Variables Created
- `vm[i]`: Voltage magnitude at bus `i` (per unit)

# Bounds Applied
- Lower bound: `bus["vmin"]` from network data
- Upper bound: `bus["vmax"]` from network data
"""
function variable_bus_voltage_magnitude_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    vm = _PM.var(pm, nw)[:vm] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :bus)], base_name="$(nw)_vm",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :bus, i), "vm_starting_value")
    )

    if bounded
        for (i, bus) in _PM.ref(pm, nw, :bus)
            JuMP.set_lower_bound(vm[i], bus["vmin"])
            JuMP.set_upper_bound(vm[i], bus["vmax"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :bus, :vm, _PM.ids(pm, nw, :bus), vm)
end

"""
    variable_gen_power_sp(pm::AbstractPowerModel; kwargs...)

Create both active and reactive power generation variables for busbar splitting.

This is a convenience function that creates both active (`pg`) and reactive (`qg`) 
power generation variables by calling the individual variable creation functions.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `kwargs...`: Keyword arguments passed to individual variable functions

# Variables Created
- `pg[i]`: Active power generation at generator `i` (MW)
- `qg[i]`: Reactive power generation at generator `i` (MVAr)
"""
function variable_gen_power_sp(pm::_PM.AbstractPowerModel; kwargs...)
    variable_gen_power_real_sp(pm; kwargs...)
    variable_gen_power_imaginary_sp(pm; kwargs...)
end

"""
    variable_gen_power_real_sp(pm::AbstractPowerModel; nw::Int, bounded::Bool=true, report::Bool=true)

Create active power generation variables for busbar splitting formulations.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `nw::Int`: Network identifier (default: nw_id_default)
- `bounded::Bool`: Whether to apply generation limits (default: true)
- `report::Bool`: Whether to include in solution reporting (default: true)

# Variables Created
- `pg[i]`: Active power generation at generator `i` (MW)

# Bounds Applied
- Lower bound: `gen["pmin"]` from generator data
- Upper bound: `gen["pmax"]` from generator data
"""
function variable_gen_power_real_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    pg = _PM.var(pm, nw)[:pg] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_pg",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :gen, i), "pg_starting_value")
    )

    if bounded
        for (i, gen) in _PM.ref(pm, nw, :gen)
            JuMP.set_lower_bound(pg[i], gen["pmin"])
            JuMP.set_upper_bound(pg[i], gen["pmax"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :gen, :pg, _PM.ids(pm, nw, :gen), pg)
end

"""
    variable_gen_power_imaginary_sp(pm::AbstractPowerModel; nw::Int, bounded::Bool=true, report::Bool=true)

Create reactive power generation variables for busbar splitting formulations.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `nw::Int`: Network identifier (default: nw_id_default)
- `bounded::Bool`: Whether to apply generation limits (default: true)
- `report::Bool`: Whether to include in solution reporting (default: true)

# Variables Created
- `qg[i]`: Reactive power generation at generator `i` (MVAr)

# Bounds Applied
- Lower bound: `gen["qmin"]` from generator data
- Upper bound: `gen["qmax"]` from generator data
"""
function variable_gen_power_imaginary_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    qg = _PM.var(pm, nw)[:qg] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :gen)], base_name="$(nw)_qg",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :gen, i), "qg_starting_value")
    )

    if bounded
        for (i, gen) in _PM.ref(pm, nw, :gen)
            JuMP.set_lower_bound(qg[i], gen["qmin"])
            JuMP.set_upper_bound(qg[i], gen["qmax"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :gen, :qg, _PM.ids(pm, nw, :gen), qg)
end