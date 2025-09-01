###################### OTS Constraints ############################

"""
    constraint_voltage_dc_ots(pm::AbstractPowerModel, n::Int)

Placeholder function for DC voltage constraints in optimal transmission switching.

This function provides a hook for implementing DC voltage-related constraints in
OTS formulations, though the current implementation is empty.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `n::Int`: Network identifier
"""
function constraint_voltage_dc_ots(pm::_PM.AbstractPowerModel,  n::Int)
end

"""
    constraint_power_balance_dc_ots(pm::AbstractPowerModel, n::Int, i::Int, bus_arcs_dcgrid, bus_convs_dc, pd)

DC power balance constraint for optimal transmission switching.

Enforces Kirchhoff's current law at DC buses in OTS formulations, ensuring that
the sum of power flows into/out of each DC bus equals the net power demand.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `n::Int`: Network identifier
- `i::Int`: DC bus identifier
- `bus_arcs_dcgrid`: Collection of DC branch arcs connected to the bus
- `bus_convs_dc`: Collection of DC converters connected to the bus
- `pd`: Power demand at the DC bus

# Constraint
∑(p_dcgrid[a] for a in bus_arcs_dcgrid) + ∑(pconv_dc[c] for c in bus_convs_dc) = -pd

Where:
- `p_dcgrid[a]`: Power flow on DC branch arc `a`
- `pconv_dc[c]`: DC-side power of converter `c`
- `pd`: Power demand (positive value represents load)
"""
function constraint_power_balance_dc_ots(pm::_PM.AbstractPowerModel, n::Int, i::Int, bus_arcs_dcgrid, bus_convs_dc, pd)
    p_dcgrid = _PM.var(pm, n, :p_dcgrid)
    pconv_dc = _PM.var(pm, n, :pconv_dc)
    z = _PM.var(pm, n, :z_ots_dc, i) 
    JuMP.@constraint(pm.model, sum(p_dcgrid[a] for a in bus_arcs_dcgrid) + sum(pconv_dc[c] for c in bus_convs_dc) == (-pd))
end

"""
    constraint_converter_limit_on_off_dc_ots(pm::AbstractPowerModel, n::Int, i, pmax, pmin, qmax, qmin, pmaxdc, pmindc, imax)

Converter operating limits with switching capability for DC OTS.

Enforces operating limits on DC converters that can be switched on/off in optimal
transmission switching problems. When a converter is off (z_dc = 0), all power
flows and currents are forced to zero. When on (z_dc = 1), normal operating
limits apply.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure
- `n::Int`: Network identifier
- `i`: Converter identifier
- `pmax`, `pmin`: AC-side active power limits (MW)
- `qmax`, `qmin`: AC-side reactive power limits (MVAr)  
- `pmaxdc`, `pmindc`: DC-side active power limits (MW)
- `imax`: Maximum converter current (A)

# Constraints Applied
For each converter power/current variable x and limit [xmin, xmax]:
- x ≤ xmax * z_dc
- x ≥ xmin * z_dc

Where z_dc is the binary converter status variable (1 = on, 0 = off).

# Variables Constrained
- AC-side powers: pconv_ac, pconv_tf_fr, pconv_tf_to, pconv_pr_fr
- AC-side reactive: qconv_ac, qconv_tf_fr, qconv_tf_to, qconv_pr_fr
- DC-side power: pconv_dc
- AC-side current: iconv_ac
"""
function constraint_converter_limit_on_off_dc_ots(pm::_PM.AbstractPowerModel, n::Int, i, pmax, pmin, qmax, qmin, pmaxdc, pmindc, imax)
    pconv_ac = _PM.var(pm, n, :pconv_ac)[i]
    pconv_dc = _PM.var(pm, n, :pconv_dc)[i]
    pconv_tf_fr = _PM.var(pm, n, :pconv_tf_fr)[i]
    pconv_tf_to = _PM.var(pm, n, :pconv_tf_to)[i]
    pconv_pr_fr = _PM.var(pm, n, :pconv_pr_fr)[i]
    
    qconv_ac = _PM.var(pm, n, :qconv_ac)[i]
    qconv_tf_fr = _PM.var(pm, n, :qconv_tf_fr)[i]
    qconv_tf_to = _PM.var(pm, n, :qconv_tf_to)[i]
    qconv_pr_fr = _PM.var(pm, n, :qconv_pr_fr)[i]
    iconv_ac = _PM.var(pm, n, :iconv_ac)[i]
    #vmc = _PM.var(pm, n, :vmc, i)
    #vmf = _PM.var(pm, n, :vmf, i)

    z_dc = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@constraint(pm.model,  pconv_ac <= pmax * z_dc)
    JuMP.@constraint(pm.model,  pconv_ac >= pmin * z_dc)
    JuMP.@constraint(pm.model,  pconv_dc <= pmaxdc * z_dc)
    JuMP.@constraint(pm.model,  pconv_dc >= pmindc * z_dc)
    JuMP.@constraint(pm.model,  pconv_tf_fr <= pmax * z_dc)
    JuMP.@constraint(pm.model,  pconv_tf_fr >= pmin * z_dc)
    JuMP.@constraint(pm.model,  pconv_tf_to <= pmax * z_dc)
    JuMP.@constraint(pm.model,  pconv_tf_to >= pmin * z_dc)
    JuMP.@constraint(pm.model,  pconv_pr_fr <= pmax * z_dc)
    JuMP.@constraint(pm.model,  pconv_pr_fr >= pmin * z_dc)
    
    JuMP.@constraint(pm.model,  qconv_ac <= qmax * z_dc)
    JuMP.@constraint(pm.model,  qconv_ac >= qmin * z_dc)
    JuMP.@constraint(pm.model,  qconv_tf_fr <= qmax * z_dc)
    JuMP.@constraint(pm.model,  qconv_tf_fr >= qmin * z_dc)
    JuMP.@constraint(pm.model,  qconv_tf_to <= qmax * z_dc)
    JuMP.@constraint(pm.model,  qconv_tf_to >= qmin * z_dc)
    JuMP.@constraint(pm.model,  qconv_pr_fr <= qmax * z_dc)
    JuMP.@constraint(pm.model,  qconv_pr_fr >= qmin * z_dc)
    JuMP.@constraint(pm.model,  iconv_ac <= imax * z_dc )

end

function constraint_branch_limit_on_off_dc_ots(pm::_PM.AbstractPowerModel, n::Int, i, f_idx, t_idx, pmax, pmin, imax, imin)
    p_fr = _PM.var(pm, n, :p_dcgrid)[f_idx]
    p_to = _PM.var(pm, n, :p_dcgrid)[t_idx]
    z = _PM.var(pm, n, :z_ots_dc, i)

    JuMP.@constraint(pm.model,  p_fr <= pmax * z)
    JuMP.@constraint(pm.model,  p_fr >= pmin * z)
    JuMP.@constraint(pm.model,  p_to <= pmax * z)
    JuMP.@constraint(pm.model,  p_to >= pmin * z)
end

function constraint_linearised_binary_variable(pm::_PM.AbstractPowerModel, n::Int, i, csi)
    z = _PM.var(pm, n, :z_branch, i)
    JuMP.@constraint(pm.model,  z*(1-z) <= csi)
end

function constraint_linearised_binary_variable_DC_branch(pm::_PM.AbstractPowerModel, n::Int, i, csi)
    z = _PM.var(pm, n, :z_ots_dc, i)
    JuMP.@constraint(pm.model,  z*(1-z) <= csi)
end

function constraint_linearised_binary_variable_DC_conv(pm::_PM.AbstractPowerModel, n::Int, i, csi)
    z = _PM.var(pm, n, :z_conv_dc, i)
    JuMP.@constraint(pm.model,  z*(1-z) <= csi)
end

###################### Busbar Splitting Constraints ############################
function constraint_linearised_binary_variable_switch(pm::_PM.AbstractPowerModel, n::Int, i, csi)
z = _PM.var(pm, n, :z_switch, i)
JuMP.@constraint(pm.model,  z*(1-z) <= csi)
end

function constraint_linearised_binary_variable_switch_no_ZIL(pm::_PM.AbstractPowerModel, n::Int, i, csi)
    sw = _PM.ref(pm, n, :switch, i)
    if sw["ZIL"] == false
        z = _PM.var(pm, n, :z_switch, i)
        JuMP.@constraint(pm.model,  z*(1-z) <= csi)
    end
end

# From PowerModels.jl
function constraint_switch_thermal_limit(pm::_PM.AbstractPowerModel, n::Int, f_idx, rating)
    psw = _PM.var(pm, n, :psw, f_idx)
    qsw = _PM.var(pm, n, :qsw, f_idx)

    JuMP.@constraint(pm.model, psw^2 + qsw^2 <= rating^2)
end

function constraint_switch_thermal_limit_dc(pm::_PM.AbstractPowerModel, n::Int, f_idx, rating)
    psw = _PM.var(pm, n, :psw, f_idx)

    JuMP.@constraint(pm.model, psw <= rating^2)
end

function constraint_dc_switch_thermal_limit(pm::_PM.AbstractPowerModel, n::Int, f_idx, rating)
    psw = _PM.var(pm, n, :p_dc_sw, f_idx)

    JuMP.@constraint(pm.model, psw <= rating)
end


function constraint_dc_switch_state_open(pm::_PM.AbstractPowerModel, n::Int, f_idx)
    psw = _PM.var(pm, n, :psw, f_idx)

    JuMP.@constraint(pm.model, psw == 0.0)
end


""
# TO be included in the DCPPowerModel formulation
function constraint_dc_switch_power_on_off(pm::_PM.AbstractPowerModel, n::Int, i, f_idx)
    psw = _PM.var(pm, n, :p_dc_sw, f_idx)
    z = _PM.var(pm, n, :z_dcswitch, i)

    psw_lb, psw_ub = _IM.variable_domain(psw)

    JuMP.@constraint(pm.model, psw <= psw_ub*z)
    JuMP.@constraint(pm.model, psw_lb*z <= psw)
end

function constraint_switch_power_on_off(pm::_PM.AbstractPowerModel, n::Int, i, f_idx)
    psw = _PM.var(pm, n, :psw, f_idx)
    qsw = _PM.var(pm, n, :qsw, f_idx)
    z = _PM.var(pm, n, :z_switch, i)

    psw_lb, psw_ub = _IM.variable_domain(psw)
    qsw_lb, qsw_ub = _IM.variable_domain(qsw)

    JuMP.@constraint(pm.model, psw <= psw_ub*z)
    JuMP.@constraint(pm.model, psw_lb*z <= psw)
    JuMP.@constraint(pm.model, qsw <= qsw_ub*z)
    JuMP.@constraint(pm.model, qsw_lb*z <= qsw)
end

function constraint_switch_power(pm::_PM.AbstractPowerModel, n::Int, i, f_idx)
    psw = _PM.var(pm, n, :psw, f_idx)
    qsw = _PM.var(pm, n, :qsw, f_idx)

    psw_lb, psw_ub = _IM.variable_domain(psw)
    qsw_lb, qsw_ub = _IM.variable_domain(qsw)

    JuMP.@constraint(pm.model, psw <= psw_ub)
    JuMP.@constraint(pm.model, psw >= psw_lb)
    JuMP.@constraint(pm.model, qsw <= qsw_ub)
    JuMP.@constraint(pm.model, qsw >= qsw_lb)
end

function constraint_exclusivity_switch(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2)
    z_1 = _PM.var(pm, n, :z_switch, i_1)
    z_2 = _PM.var(pm, n, :z_switch, i_2)
    
    JuMP.@constraint(pm.model, z_1 + z_2 <= 1.0)
end

function constraint_ZIL_switch(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2)
    z_1 = _PM.var(pm, n, :z_switch, i_1)
    z_2 = _PM.var(pm, n, :z_switch, i_2)
    
    JuMP.@constraint(pm.model, z_1 <= (1.0 - z_2))
end

function constraint_ZIL_no_OTS(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2, i_3) # Not needed
    z_1 = _PM.var(pm, n, :z_switch, i_1)
    z_2 = _PM.var(pm, n, :z_switch, i_2)
    z_3 = _PM.var(pm, n, :z_switch, i_3)
    
    JuMP.@constraint(pm.model, 1.0 <= z_1 + z_2 + z_3)
end


function constraint_ZIL_dc_switch(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2)
    z_1 = _PM.var(pm, n, :z_dcswitch, i_1)
    z_2 = _PM.var(pm, n, :z_dcswitch, i_2)
    
    JuMP.@constraint(pm.model, z_1 <= (1.0 - z_2))
end

function constraint_exclusivity_switch_no_OTS(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2)
    z_1 = _PM.var(pm, n, :z_switch, i_1)
    z_2 = _PM.var(pm, n, :z_switch, i_2)
    
    JuMP.@constraint(pm.model, z_1 + z_2 == 1.0)
end

function constraint_voltage_angles_switch(pm::_PM.AbstractPowerModel, n::Int, i_1,bus_1, bus_2)
    #z_1 = _PM.var(pm, n, :z_switch, i_1)
    #z_2 = _PM.var(pm, n, :z_switch, i_2)
    z_ZIL = _PM.var(pm, n, :z_switch, i_1)
    va_1 = _PM.var(pm, n, :va, bus_1)
    vm_1 = _PM.var(pm, n, :vm, bus_1)
    va_2 = _PM.var(pm, n, :va, bus_2)
    vm_2 = _PM.var(pm, n, :vm, bus_2)
    

    JuMP.@constraint(pm.model,(va_1 - va_2) <= 100*(1 - z_ZIL))
    JuMP.@constraint(pm.model,-100*(1 - z_ZIL) <= va_1 - va_2)
    JuMP.@constraint(pm.model, vm_1 - vm_2 <= 100*(1 - z_ZIL))
    JuMP.@constraint(pm.model, -100*(1 - z_ZIL) <= vm_1 - vm_2)

    #JuMP.@constraint(pm.model, z_ZIL*va_1 == z_ZIL*va_2)
    #JuMP.@constraint(pm.model, z_ZIL*vm_1 == z_ZIL*vm_2)
end

function constraint_voltage_angles_dc_switch(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2, i_3, bus_1, bus_2)
    z_1 = _PM.var(pm, n, :z_dcswitch, i_1)
    z_2 = _PM.var(pm, n, :z_dcswitch, i_2)
    z_ZIL = _PM.var(pm, n, :z_dcswitch, i_3)
    vm_1 = _PM.var(pm, n, :vdcm, bus_1)
    vm_2 = _PM.var(pm, n, :vdcm, bus_2)
    
    JuMP.@constraint(pm.model, vm_1 == vm_2)
end

function constraint_exclusivity_dc_switch(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2)
    z_1 = _PM.var(pm, n, :z_dcswitch, i_1)
    z_2 = _PM.var(pm, n, :z_dcswitch, i_2)
 
    JuMP.@constraint(pm.model, z_1 + z_2 <= 1.0)
end

function constraint_exclusivity_dc_switch_no_OTS(pm::_PM.AbstractPowerModel, n::Int, i_1, i_2, i_3)
    z_1 = _PM.var(pm, n, :z_dcswitch, i_1)
    z_2 = _PM.var(pm, n, :z_dcswitch, i_2)
    
    JuMP.@constraint(pm.model, z_1 + z_2 == 1.0)
end

function constraint_BS_OTS_branch(pm::_PM.AbstractPowerModel, n::Int,i_1, i_2)
    z_1 = _PM.var(pm, n, :z_switch, i_1)
    z_2 = _PM.var(pm, n, :z_switch, i_2)
    branch_dict = _PM.ref(pm, n, :branch)
    f_sw = _PM.ref(pm, n, :switch, i_1)
    t_sw = _PM.ref(pm, n, :switch, i_2)
    aux = f_sw["auxiliary"]
    orig = f_sw["original"]
    
    if aux == "branch"
        pf = (branch_dict[orig]["index"],branch_dict[orig]["f_bus"],branch_dict[orig]["t_bus"])
        pt = (branch_dict[orig]["index"],branch_dict[orig]["t_bus"],branch_dict[orig]["f_bus"])
        qf = (branch_dict[orig]["index"],branch_dict[orig]["f_bus"],branch_dict[orig]["t_bus"])
        qt = (branch_dict[orig]["index"],branch_dict[orig]["t_bus"],branch_dict[orig]["f_bus"])

        pf_ = _PM.var(pm, n, :p, pf)
        pt_ = _PM.var(pm, n, :p, pt)
        qf_ = _PM.var(pm, n, :q, qf)
        qt_ = _PM.var(pm, n, :q, qt)

        JuMP.@constraint(pm.model, pf_ <= (z_1+z_2)*10)
        JuMP.@constraint(pm.model, pt_ <= (z_1+z_2)*10)
        JuMP.@constraint(pm.model, qf_ <= (z_1+z_2)*10)
        JuMP.@constraint(pm.model, qt_ <= (z_1+z_2)*10)
        JuMP.@constraint(pm.model, - (z_1+z_2)*10 <= pf_)
        JuMP.@constraint(pm.model, - (z_1+z_2)*10 <= pt_)
        JuMP.@constraint(pm.model, - (z_1+z_2)*10 <= qf_)
        JuMP.@constraint(pm.model, - (z_1+z_2)*10 <= qt_)
    end
end

function constraint_BS_OTS_dcbranch(pm::_PM.AbstractPowerModel, n::Int,i_1, i_2, pf, pt, qf, qt ,sw,aux)
    z_1 = _PM.var(pm, n, :z_dcswitch, i_1)
    z_2 = _PM.var(pm, n, :z_dcswitch, i_2)
    pf_ = _PM.var(pm, n, :p_dcgrid, pf)
    pt_ = _PM.var(pm, n, :p_dcgrid, pt)

    JuMP.@constraint(pm.model, pf_ <= (z_1+z_2)*100)
    JuMP.@constraint(pm.model, pt_ <= (z_1+z_2)*100)
    JuMP.@constraint(pm.model, - (z_1+z_2)*100 <= pf_)
    JuMP.@constraint(pm.model, - (z_1+z_2)*100 <= pt_)
end

function constraint_power_balance_dc_switch(pm::_PM.AbstractPowerModel, n::Int, i::Int, bus_arcs_dcgrid, bus_convs_dc, bus_arcs_sw_dc, pd)
    p_dcgrid = _PM.var(pm, n, :p_dcgrid)
    pconv_dc = _PM.var(pm, n, :pconv_dc)
    psw = _PM.var(pm, n, :p_dc_sw)
    JuMP.@constraint(pm.model, sum(p_dcgrid[a] for a in bus_arcs_dcgrid) + sum(pconv_dc[c] for c in bus_convs_dc) + sum(psw[sw] for sw in bus_arcs_sw_dc) == (-pd))
end

function constraint_switch_difference_voltage_angles(pm::_PM.AbstractPowerModel, n::Int, switch, diff_vas)
    va_f = _PM.var(pm, n, :va, switch["f_bus"])
    va_t = _PM.var(pm, n, :va, switch["t_bus"])


    JuMP.@constraint(pm.model, va_f - va_t <= diff_vas)
    JuMP.@constraint(pm.model, - diff_vas <= va_f - va_t)
end


###################### Bilinear terms reformulation ############################
