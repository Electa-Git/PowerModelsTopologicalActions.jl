"variable: `t[i]` for `i` in `bus`es"
function variable_bus_voltage_angle_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    va = _PM.var(pm, nw)[:va] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :bus)], base_name="$(nw)_va",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :bus, i), "va_starting_value")
    )
    
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :bus, :va, _PM.ids(pm, nw, :bus), va)
end

"variable: `v[i]` for `i` in `bus`es"
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

"generates variables for both `active` and `reactive` generation"
function variable_gen_power_sp(pm::_PM.AbstractPowerModel; kwargs...)
    variable_gen_power_real_sp(pm; kwargs...)
    variable_gen_power_imaginary_sp(pm; kwargs...)
end


"variable: `pg[j]` for `j` in `gen`"
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

"variable: `qq[j]` for `j` in `gen`"
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