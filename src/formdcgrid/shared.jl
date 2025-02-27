######################################## OTS Constraints ###########################################
function variable_dcgrid_voltage_magnitude(pm::_PM.AbstractWModels; kwargs...)
    variable_dcgrid_voltage_magnitude_sqr(pm; kwargs...)
end

function constraint_ohms_ots_dc_branch(pm::_PM.AbstractWRModels, n::Int,  f_bus, t_bus, f_idx, t_idx, r, p)
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    wdc_fr = _PM.var(pm, n, :wdc, f_bus)
    wdc_to = _PM.var(pm, n, :wdc, t_bus)
    wdc_frto = _PM.var(pm, n, :wdcr, (f_bus, t_bus))
    l = f_idx[1]
    z = _PM.var(pm, n, :z_ots_dc, l)
    

    if r == 0
        JuMP.@constraint(pm.model, p_dc_fr + p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@constraint(pm.model, p_dc_fr == p * g * z[l] * (wdc_fr - wdc_frto))
        JuMP.@constraint(pm.model, p_dc_to == p * g * z[l] * (wdc_to - wdc_frto))
    end
end

function constraint_converter_losses(pm::_PM.AbstractWModels, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    pconv_dc = _PM.var(pm, n,  :pconv_dc, i)
    iconv = _PM.var(pm, n,  :iconv_ac, i)
    iconv_sq = _PM.var(pm, n,  :iconv_ac_sq, i)

    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == a + b*iconv + c*iconv_sq)
end

function constraint_power_balance_ac_switch(pm::_PM.AbstractWRModels, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    w = _PM.var(pm, n, :w, i)
    p = _PM.var(pm, n,  :p)
    q = _PM.var(pm, n,  :q)
    pg = _PM.var(pm, n,  :pg)
    qg = _PM.var(pm, n,  :qg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    qconv_grid_ac = _PM.var(pm, n,  :qconv_tf_fr)
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)

    cstr_p = JuMP.@constraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*w)
    cstr_q = JuMP.@constraint(pm.model, sum(q[a] for a in bus_arcs) + sum(qconv_grid_ac[c] for c in bus_convs_ac) + sum(qsw[sw] for sw in bus_arcs_sw) == sum(qg[g] for g in bus_gens)  - sum(qd[d] for d in bus_loads) + sum(bs[s] for s in bus_shunts)*w)

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end

######################################## BUSBAR SPLITTING CONSTRAINTS #####################################

function constraint_switch_voltage_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_bus, t_bus)
    w_fr = _PM.var(pm, n, :w, f_bus)
    w_to = _PM.var(pm, n, :w, t_bus)
    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*w_fr == z*w_to)
end

function constraint_switch_voltage_on_off_big_M(pm::_PM.SOCWRPowerModel, n::Int, i, f_bus, t_bus)
    w_fr = _PM.var(pm, n, :w, f_bus)
    w_to = _PM.var(pm, n, :w, t_bus)
    z = _PM.var(pm, n, :z_switch, i)
    M_vm = 1
    M_va = 2*pi

    JuMP.@constraint(pm.model, w_fr - w_to <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= w_fr - w_to)

    JuMP.@constraint(pm.model, w_to - w_fr <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= w_to - w_fr)
end

function constraint_switch_voltage_on_off_big_M(pm::_PM.QCRMPowerModel, n::Int, i, f_bus, t_bus)
    w_fr = _PM.var(pm, n, :w, f_bus)
    w_to = _PM.var(pm, n, :w, t_bus)
    vm_fr = _PM.var(pm, n, :vm, f_bus)
    vm_to = _PM.var(pm, n, :vm, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    z = _PM.var(pm, n, :z_switch, i)
    M_vm = 1
    M_va = 2*pi

    JuMP.@constraint(pm.model, w_fr - w_to <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= w_fr - w_to)

    JuMP.@constraint(pm.model, w_to - w_fr <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= w_to - w_fr)

    JuMP.@constraint(pm.model, vm_fr - vm_to <= (1-z)*M_vm)
    JuMP.@constraint(pm.model, va_fr - va_to <= (1-z)*M_va)

    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= vm_fr - vm_to)
    JuMP.@constraint(pm.model,  - (1-z)*M_va <= va_fr - va_to)

    JuMP.@constraint(pm.model, vm_to - vm_fr <= (1-z)*M_vm)
    JuMP.@constraint(pm.model, va_to - va_fr <= (1-z)*M_va)

    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= vm_to - vm_fr)
    JuMP.@constraint(pm.model,  - (1-z)*M_va <= va_to - va_fr)
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_busdc, t_busdc)
    wdc_fr = _PM.var(pm, n, :wdc, f_busdc)
    wdc_to = _PM.var(pm, n, :wdc, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)

    JuMP.@constraint(pm.model, z*wdc_fr == z*wdc_to)
end

function constraint_dc_switch_voltage_on_off_big_M(pm::_PM.AbstractWRModels, n::Int, i, f_busdc, t_busdc)
    wdc_fr = _PM.var(pm, n, :wdc, f_busdc)
    wdc_to = _PM.var(pm, n, :wdc, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)
    M_vm = 1

    JuMP.@constraint(pm.model, wdc_fr - wdc_to <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= wdc_fr - wdc_to)

    JuMP.@constraint(pm.model, wdc_to - wdc_fr <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= wdc_to - wdc_fr)
end

function constraint_aux_switches(pm::_PM.AbstractWRModels, n::Int, i_1)
    aux_vm = _PM.var(pm, n, :aux_switch_w, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_switch_w, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)

    JuMP.@constraint(pm.model, aux_vm == z_ZIL*aux_diff_vm)
end

function constraint_aux_differences(pm::_PM.AbstractWRModels, n::Int, i_1, bus_1, bus_2)
    aux_diff_vm = _PM.var(pm, n, :delta_switch_w, i_1)
    vm_1 = _PM.var(pm, n, :w, bus_1)
    vm_2 = _PM.var(pm, n, :w, bus_2)

    JuMP.@constraint(pm.model, aux_diff_vm == vm_1 - vm_2)
end

function constraint_1_aux_voltage_angles(pm::_PM.AbstractWRModels, n::Int, i_1, delta_min, delta_max)
end

function constraint_2_aux_voltage_angles(pm::_PM.AbstractWRModels, n::Int, i_1, delta_min, delta_max)
end

function constraint_1_aux_voltage_magnitudes(pm::_PM.AbstractWRModels, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_switch_w, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    
    JuMP.@constraint(pm.model, aux_vm <= z_ZIL*delta_max)
    JuMP.@constraint(pm.model, z_ZIL*delta_min <= aux_vm)
end

function constraint_2_aux_voltage_magnitudes(pm::_PM.AbstractWRModels, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_switch_w, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_switch_w, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    
    JuMP.@constraint(pm.model, - delta_max*(1-z_ZIL) <= aux_vm - aux_diff_vm)
    JuMP.@constraint(pm.model, aux_vm - aux_diff_vm <= - delta_min*(1-z_ZIL))
end

function constraint_aux_dcswitches(pm::_PM.AbstractWRModels, n::Int, i_1)
    aux_vm = _PM.var(pm, n, :aux_dcswitch_w, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_dcswitch_w, i_1)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_1)

    JuMP.@constraint(pm.model, aux_vm == z_ZIL*aux_diff_vm)
end

function constraint_aux_dcdifferences(pm::_PM.AbstractWRModels, n::Int, i_1, bus_1, bus_2)
    aux_diff_vm = _PM.var(pm, n, :delta_dcswitch_w, i_1)
    vm_1 = _PM.var(pm, n, :wdc, bus_1)
    vm_2 = _PM.var(pm, n, :wdc, bus_2)

    JuMP.@constraint(pm.model, aux_diff_vm == vm_1 - vm_2)
end

function constraint_1_aux_voltage_dc_magnitudes(pm::_PM.AbstractWRModels, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_dcswitch_w, i_1)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_1)
    
    JuMP.@constraint(pm.model, aux_vm <= z_ZIL*delta_max)
    JuMP.@constraint(pm.model, z_ZIL*delta_min <= aux_vm)
end

function constraint_2_aux_voltage_dc_magnitudes(pm::_PM.AbstractWRModels, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_dcswitch_w, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_dcswitch_w, i_1)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_1)
    
    JuMP.@constraint(pm.model, - delta_max*(1-z_ZIL) <= aux_vm - aux_diff_vm)
    JuMP.@constraint(pm.model, aux_vm - aux_diff_vm <= - delta_min*(1-z_ZIL))
end