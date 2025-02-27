function variable_bus_voltage_sp(pm::_PM.AbstractLPACModel; kwargs...)
    variable_bus_voltage_angle_sp(pm; kwargs...)
    _PM.variable_bus_voltage_magnitude(pm; kwargs...)
    _PM.variable_buspair_cosine(pm; kwargs...)
end

function variable_bus_voltage_lpac_sp(pm::_PM.AbstractLPACModel; kwargs...)
    variable_bus_voltage_angle_sp(pm; kwargs...)
    variable_bus_voltage_magnitude_sp(pm; kwargs...)
    _PM.variable_buspair_cosine(pm; kwargs...)
end

function variable_bus_voltage_magnitude_sp(pm::_PM.AbstractLPACModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    phi = _PM.var(pm, nw)[:phi] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :bus)], base_name="$(nw)_phi",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :bus, i), "phi_starting_value")
    )

    if bounded
        for (i, bus) in ref(pm, nw, :bus)
            JuMP.set_lower_bound(phi[i], bus["vmin"] - 1.0)
            JuMP.set_upper_bound(phi[i], bus["vmax"] - 1.0)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :bus, :phi, _PM.ids(pm, nw, :bus), phi)
end


function constraint_ohms_ots_dc_branch(pm::_PM.AbstractLPACCModel, n::Int,  f_bus, t_bus, f_idx, t_idx, r, p)
    i, f_bus, t_bus = f_idx
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    phi_fr = _PM.var(pm, n, :phi_vdcm, f_bus)
    phi_to = _PM.var(pm, n, :phi_vdcm, t_bus)
    phi_fr_ub = JuMP.UpperBoundRef(phi_to)
    phi_fr_lb = JuMP.LowerBoundRef(phi_to)
    z = _PM.var(pm, n, :z_ots_dc, i)

    if r == 0
        JuMP.@constraint(pm.model, z*p_dc_fr + z*p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@constraint(pm.model, p_dc_fr == p * g * z*(phi_fr - phi_to))
        JuMP.@constraint(pm.model, p_dc_to == p * g * z*(phi_to - phi_fr))
    end
end

###################### Busbar Splitting Constraints ############################
function constraint_switch_thermal_limit(pm::_PM.AbstractLPACCModel, n::Int, f_idx, rating)
    psw = _PM.var(pm, n, :psw, f_idx)
    qsw = _PM.var(pm, n, :qsw, f_idx)

    #JuMP.@constraint(pm.model, psw^2 + qsw^2 <= rating^2)
    csw_lpac = constraint_switch_capacity_PWL(pm, n, psw, qsw, rating)
end


function constraint_switch_capacity_PWL(pm::_PM.AbstractLPACCModel, n::Int, psw, qsw, rating)
    np = 20 #no. of segments, can be passed as an argument later
    l = 0
    for i = 1:np
        a= rating*sin(l)
        b = rating*cos(l)
        csw_lpac = JuMP.@constraint(pm.model, a*psw + b*qsw <= rating^2) #current and voltage bounds to be proper to use Umax*Imax because Umax*Imax == Smax
        l = l + 2*pi/np
    end
end

function constraint_switch_voltage_on_off(pm::_PM.AbstractLPACCModel, n::Int, i, f_bus, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    phi_fr = _PM.var(pm, n, :phi, f_bus)
    phi_to = _PM.var(pm, n, :phi, t_bus)
    #cs  = _PM.var(pm, n, :cs, (f_bus, t_bus))
    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*phi_fr == z*phi_to)
    JuMP.@constraint(pm.model, z*va_fr == z*va_to)
end

function constraint_switch_voltage_on_off_big_M(pm::_PM.AbstractLPACCModel, n::Int, i, f_bus, t_bus)
    phi_fr = _PM.var(pm, n, :phi, f_bus)
    phi_to = _PM.var(pm, n, :phi, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    z = _PM.var(pm, n, :z_switch, i)
    M_vm = 1 # 0.2 -> tighter implies lower values for lpac case 5 but higher for AC-fc
    M_va = 2*pi # pi

    JuMP.@constraint(pm.model, phi_fr - phi_to <= (1-z)*M_vm)
    JuMP.@constraint(pm.model, va_fr - va_to <= (1-z)*M_va)

    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= phi_fr - phi_to)
    JuMP.@constraint(pm.model,  - (1-z)*M_va <= va_fr - va_to)

    JuMP.@constraint(pm.model, phi_to - phi_fr <= (1-z)*M_vm)
    JuMP.@constraint(pm.model, va_to - va_fr <= (1-z)*M_va)

    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= phi_to - phi_fr)
    JuMP.@constraint(pm.model,  - (1-z)*M_va <= va_to - va_fr)
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractLPACCModel, n::Int, i, f_busdc, t_busdc)
    phi_fr = _PM.var(pm, n, :phi_vdcm, f_busdc)
    phi_to = _PM.var(pm, n, :phi_vdcm, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)

    JuMP.@constraint(pm.model, z*phi_fr == z*phi_to)
end

function constraint_dc_switch_voltage_on_off_big_M(pm::_PM.AbstractLPACCModel, n::Int, i, f_busdc, t_busdc)
    phi_fr = _PM.var(pm, n, :phi_vdcm, f_busdc)
    phi_to = _PM.var(pm, n, :phi_vdcm, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)
    M_vm = 1

    JuMP.@constraint(pm.model, phi_fr - phi_to <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= phi_fr - phi_to)

    JuMP.@constraint(pm.model, phi_to - phi_fr <= (1-z)*M_vm)
    JuMP.@constraint(pm.model,  - (1-z)*M_vm <= phi_to - phi_fr)
end

function constraint_power_balance_ac_switch(pm::_PM.AbstractLPACCModel, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    phi  = _PM.var(pm, n, :phi, i)
    p = _PM.var(pm, n,  :p)
    q = _PM.var(pm, n,  :q)
    pg = _PM.var(pm, n,  :pg)
    qg = _PM.var(pm, n,  :qg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    qconv_grid_ac = _PM.var(pm, n,  :qconv_tf_fr)
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)

    cstr_p = JuMP.@constraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*(1.0 + 2*phi))
    cstr_q = JuMP.@constraint(pm.model, sum(q[a] for a in bus_arcs) + sum(qconv_grid_ac[c] for c in bus_convs_ac) + sum(qsw[sw] for sw in bus_arcs_sw) == sum(qg[g] for g in bus_gens)  - sum(qd[d] for d in bus_loads) + sum(bs[s] for s in bus_shunts)*(1.0 + 2*phi))

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end

function constraint_power_balance_ac_grid_ac_switch(pm::_PM.AbstractLPACCModel, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_loads, bus_shunts, pd, qd, gs, bs)
    phi  = _PM.var(pm, n, :phi, i)
    p = _PM.var(pm, n,  :p)
    q = _PM.var(pm, n,  :q)
    pg = _PM.var(pm, n,  :pg)
    qg = _PM.var(pm, n,  :qg)
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)

    cstr_p = JuMP.@constraint(pm.model, sum(p[a] for a in bus_arcs) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*(1.0 + 2*phi))
    cstr_q = JuMP.@constraint(pm.model, sum(q[a] for a in bus_arcs) + sum(qsw[sw] for sw in bus_arcs_sw) == sum(qg[g] for g in bus_gens)  - sum(qd[d] for d in bus_loads) + sum(bs[s] for s in bus_shunts)*(1.0 + 2*phi))

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end


###################### Bilinear terms reformulation ############################

function constraint_aux_switches(pm::_PM.AbstractLPACCModel, n::Int, i_1)
    aux_va = _PM.var(pm, n, :aux_switch_va, i_1)
    aux_vm = _PM.var(pm, n, :aux_switch_phi, i_1)
    aux_diff_va = _PM.var(pm, n, :delta_switch_va, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_switch_phi, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)

    JuMP.@constraint(pm.model, aux_va == z_ZIL*aux_diff_va)
    JuMP.@constraint(pm.model, aux_vm == z_ZIL*aux_diff_vm)
end

function constraint_aux_differences(pm::_PM.AbstractLPACCModel, n::Int, i_1, bus_1, bus_2)
    aux_diff_va = _PM.var(pm, n, :delta_switch_va, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_switch_phi, i_1)
    va_1 = _PM.var(pm, n, :va, bus_1)
    vm_1 = _PM.var(pm, n, :phi, bus_1)
    va_2 = _PM.var(pm, n, :va, bus_2)
    vm_2 = _PM.var(pm, n, :phi, bus_2)

    JuMP.@constraint(pm.model, aux_diff_va == va_1 - va_2)
    JuMP.@constraint(pm.model, aux_diff_vm == vm_1 - vm_2)
end


function constraint_1_aux_voltage_angles(pm::_PM.AbstractLPACCModel, n::Int, i_1, delta_min, delta_max)
    aux_va = _PM.var(pm, n, :aux_switch_va, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    
    JuMP.@constraint(pm.model, aux_va <= z_ZIL*delta_max)
    JuMP.@constraint(pm.model, z_ZIL*delta_min <= aux_va)
end

function constraint_2_aux_voltage_angles(pm::_PM.AbstractLPACCModel, n::Int, i_1, delta_min, delta_max)
    aux_va = _PM.var(pm, n, :aux_switch_va, i_1)
    aux_diff_va = _PM.var(pm, n, :delta_switch_va, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    
    JuMP.@constraint(pm.model, - delta_max*(1-z_ZIL) <= aux_va - aux_diff_va)
    JuMP.@constraint(pm.model, aux_va - aux_diff_va <= - delta_min*(1-z_ZIL))
end

function constraint_1_aux_voltage_magnitudes(pm::_PM.AbstractLPACCModel, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_switch_phi, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    
    JuMP.@constraint(pm.model, aux_vm <= z_ZIL*delta_max)
    JuMP.@constraint(pm.model, z_ZIL*delta_min <= aux_vm)
end

function constraint_2_aux_voltage_magnitudes(pm::_PM.AbstractLPACCModel, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_switch_phi, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_switch_phi, i_1)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    
    JuMP.@constraint(pm.model, - delta_max*(1-z_ZIL) <= aux_vm - aux_diff_vm)
    JuMP.@constraint(pm.model, aux_vm - aux_diff_vm <= - delta_min*(1-z_ZIL))
end

function constraint_aux_dcswitches(pm::_PM.AbstractLPACCModel, n::Int, i_1)
    aux_vm = _PM.var(pm, n, :aux_dcswitch_phi, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_dcswitch_phi, i_1)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_1)

    JuMP.@constraint(pm.model, aux_vm == z_ZIL*aux_diff_vm)
end

function constraint_aux_dcdifferences(pm::_PM.AbstractLPACCModel, n::Int, i_1, bus_1, bus_2)
    aux_diff_vm = _PM.var(pm, n, :delta_dcswitch_phi, i_1)
    vm_1 = _PM.var(pm, n, :phi_vdcm, bus_1)
    vm_2 = _PM.var(pm, n, :phi_vdcm, bus_2)

    JuMP.@constraint(pm.model, aux_diff_vm == vm_1 - vm_2)
end

function constraint_1_aux_voltage_dc_magnitudes(pm::_PM.AbstractLPACCModel, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_dcswitch_phi, i_1)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_1)
    
    JuMP.@constraint(pm.model, aux_vm <= z_ZIL*delta_max)
    JuMP.@constraint(pm.model, z_ZIL*delta_min <= aux_vm)
end

function constraint_2_aux_voltage_dc_magnitudes(pm::_PM.AbstractLPACCModel, n::Int, i_1, delta_min, delta_max)
    aux_vm = _PM.var(pm, n, :aux_dcswitch_phi, i_1)
    aux_diff_vm = _PM.var(pm, n, :delta_dcswitch_phi, i_1)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_1)
    
    JuMP.@constraint(pm.model, - delta_max*(1-z_ZIL) <= aux_vm - aux_diff_vm)
    JuMP.@constraint(pm.model, aux_vm - aux_diff_vm <= - delta_min*(1-z_ZIL))
end
