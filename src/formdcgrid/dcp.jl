## ACDC switch
function constraint_power_balance_ac_switch(pm::_PM.AbstractDCPModel, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    p = _PM.var(pm, n,  :p)
    pg = _PM.var(pm, n,  :pg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    psw  = _PM.var(pm, n, :psw)
    v = 1

    cstr_p = JuMP.@constraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*v^2)

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
    end
end

function constraint_switch_voltage_on_off(pm::_PM.AbstractDCPModel, n::Int, i, f_bus, t_bus)
    #vm_fr = _PM.var(pm, n, :vm, f_bus)
    #vm_to = _PM.var(pm, n, :vm, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*va_fr == z*va_to)
end

function constraint_switch_power_on_off(pm::_PM.AbstractDCPModel, n::Int, i, f_idx)
    psw = _PM.var(pm, n, :psw, f_idx)
    z = _PM.var(pm, n, :z_switch, i)

    psw_lb, psw_ub = _IM.variable_domain(psw)

    JuMP.@constraint(pm.model, psw <= psw_ub*z)
    JuMP.@constraint(pm.model, psw >= psw_lb*z)
end

function variable_switch_power(pm::_PM.AbstractDCPModel; kwargs...)
    variable_switch_power_real(pm; kwargs...)
end

function constraint_switch_thermal_limit(pm::_PM.AbstractDCPModel, n::Int, f_idx, rating)
    psw = _PM.var(pm, n, :psw, f_idx)

    JuMP.@constraint(pm.model, psw <= rating)
    JuMP.@constraint(pm.model, - rating <= psw)
end

function constraint_BS_OTS_branch(pm::_PM.AbstractDCPModel, n::Int,i_1, i_2, pf, pt, qf, qt ,sw,aux)
    z_1 = _PM.var(pm, n, :z_switch, i_1)
    z_2 = _PM.var(pm, n, :z_switch, i_2)
    pf_ = _PM.var(pm, n, :p, pf)
    pt_ = _PM.var(pm, n, :p, pt)

    JuMP.@constraint(pm.model, pf_ <= (z_1+z_2)*100)
    JuMP.@constraint(pm.model, pt_ <= (z_1+z_2)*100)
    JuMP.@constraint(pm.model, - (z_1+z_2)*100 <= pf_)
    JuMP.@constraint(pm.model, - (z_1+z_2)*100 <= pt_)
end


function constraint_switch_voltage_on_off_big_M(pm::_PM.AbstractDCPModel, n::Int, i, f_bus, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    z = _PM.var(pm, n, :z_switch, i)
    M_va = 2*pi

    JuMP.@constraint(pm.model, va_fr - va_to <= (1-z)*M_va)
    JuMP.@constraint(pm.model,  - (1-z)*M_va <= va_fr - va_to)
    JuMP.@constraint(pm.model, va_to - va_fr <= (1-z)*M_va)
    JuMP.@constraint(pm.model,  - (1-z)*M_va <= va_to - va_fr)
end