function constraint_ohms_ots_dc_branch(pm::_PM.AbstractACPModel, n::Int, f_bus, t_bus, f_idx, t_idx, r, p)
    i, f_bus, t_bus = f_idx
    p_dc_fr = _PM.var(pm, n,  :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n,  :p_dcgrid, t_idx)
    vmdc_fr = _PM.var(pm, n,  :vdcm, f_bus)
    vmdc_to = _PM.var(pm, n,  :vdcm, t_bus)
    z = _PM.var(pm, n, :z_ots_dc, i)

    if r == 0
        JuMP.@constraint(pm.model, z*p_dc_fr + z*p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@NLconstraint(pm.model, p_dc_fr == z*(p * g * vmdc_fr * (vmdc_fr - vmdc_to)))
        JuMP.@NLconstraint(pm.model, p_dc_to == z*(p * g * vmdc_to * (vmdc_to - vmdc_fr)))
    end
end

function constraint_switch_voltage_on_off(pm::_PM.AbstractACPModel, n::Int, i, f_bus, t_bus)
    vm_fr = _PM.var(pm, n, :vm, f_bus)
    vm_to = _PM.var(pm, n, :vm, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*vm_fr == z*vm_to)
    JuMP.@constraint(pm.model, z*va_fr == z*va_to)
end

function constraint_dc_switch_state_closed(pm::_PM.AbstractACPModel, n::Int, f_busdc, t_busdc)
    vm_fr = _PM.var(pm, n, :vdcm, f_busdc)
    vm_to = _PM.var(pm, n, :vdcm, t_busdc)

    JuMP.@constraint(pm.model, vm_fr == vm_to)
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractACPModel, n::Int, i, f_busdc, t_busdc)
    vm_fr = _PM.var(pm, n, :vdcm, f_busdc)
    vm_to = _PM.var(pm, n, :vdcm, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)

    JuMP.@constraint(pm.model, z*vm_fr == z*vm_to)
end

function constraint_ac_switch_power(pm::_PM.AbstractACPModel, n::Int, i, f_bus, t_bus)
    vm_fr = _PM.var(pm, n, :vm, f_bus)
    #vm_to = _PM.var(pm, n, :vm, f_idx)

    i_sw_r = _PM.var(pm, n, :i_sw_r, (i,f_bus,t_bus))
    i_sw_i = _PM.var(pm, n, :i_sw_i, (i,f_bus,t_bus))

    psw = _PM.var(pm, n, :psw, (i,f_bus,t_bus))
    qsw = _PM.var(pm, n, :qsw, (i,f_bus,t_bus))

    JuMP.@NLconstraint(pm.model, psw == vm_fr*i_sw_r)
    JuMP.@NLconstraint(pm.model, qsw == vm_fr*i_sw_i)
end

function constraint_current_switch_thermal_limits(pm::_PM.AbstractACPModel, n::Int, i, f_bus, t_bus, rate_i_r, rate_i_i)
    i_sw_r = _PM.var(pm, n, :i_sw_r, (i,f_bus,t_bus))
    i_sw_i = _PM.var(pm, n, :i_sw_i, (i,f_bus,t_bus))

    JuMP.@constraint(pm.model, - rate_i_r <= i_sw_r)
    JuMP.@constraint(pm.model, i_sw_r <= rate_i_r)
    JuMP.@constraint(pm.model, - rate_i_i <= i_sw_i)
    JuMP.@constraint(pm.model, i_sw_i <= rate_i_i)
end

function constraint_dc_switch_power(pm::_PM.AbstractACPModel, n::Int, i, f_busdc, t_busdc)
    vdcm = _PM.var(pm, n, :vdcm, f_busdc)
    #vm_to = _PM.var(pm, n, :vm, f_idx)

    i_sw_dc = _PM.var(pm, n, :i_sw_dc, (i,f_busdc,t_busdc))
    p_dc_sw = _PM.var(pm, n, :p_dc_sw, (i,f_busdc,t_busdc))

    JuMP.@NLconstraint(pm.model, p_dc_sw == vdcm*i_sw_dc)
end

function constraint_current_dc_switch_thermal_limits(pm::_PM.AbstractACPModel, n::Int, i, f_busdc, t_busdc, rate_i_dc)
    i_sw_dc = _PM.var(pm, n, :i_sw_dc, (i,f_busdc,t_busdc))

    JuMP.@constraint(pm.model, - rate_i_dc <= i_sw_dc)
    JuMP.@constraint(pm.model, i_sw_dc <= rate_i_dc)
end

## ACDC switch
function constraint_power_balance_ac_switch(pm::_PM.AbstractACPModel, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    vm = _PM.var(pm, n,  :vm, i)
    p = _PM.var(pm, n,  :p)
    q = _PM.var(pm, n,  :q)
    pg = _PM.var(pm, n,  :pg)
    qg = _PM.var(pm, n,  :qg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    qconv_grid_ac = _PM.var(pm, n,  :qconv_tf_fr)
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)

    cstr_p = JuMP.@NLconstraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*vm^2)
    cstr_q = JuMP.@NLconstraint(pm.model, sum(q[a] for a in bus_arcs) + sum(qconv_grid_ac[c] for c in bus_convs_ac) + sum(qsw[sw] for sw in bus_arcs_sw) == sum(qg[g] for g in bus_gens)  - sum(qd[d] for d in bus_loads) + sum(bs[s] for s in bus_shunts)*vm^2)

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end

