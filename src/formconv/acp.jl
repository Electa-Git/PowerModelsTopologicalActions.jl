
"""
AC Power (ACP) formulation constraints for converter systems in optimal transmission switching.

This file implements AC power flow constraints for AC/DC converters when used in 
optimal transmission switching problems. The ACP formulation uses polar coordinates
(voltage magnitude and angle) to represent the full nonlinear AC power flow equations.
"""

"""
    constraint_conv_transformer_dc_ots(pm::AbstractACPModel, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)

AC power flow constraints for converter transformers with DC switching capability.

Implements the AC power flow equations for the transformer component of AC/DC converters
when the converter can be switched on/off in optimal transmission switching problems.
When the converter is off (z_DC = 0), all power flows are forced to zero.

# Arguments
- `pm::AbstractACPModel`: ACP power model formulation
- `n::Int`: Network identifier
- `i::Int`: Converter identifier
- `rtf`, `xtf`: Transformer resistance and reactance (per unit)
- `acbus`: AC bus connected to converter
- `tm`: Transformer tap ratio
- `transformer`: Boolean indicating if transformer is present

# Variables Used
- `pconv_tf_fr`, `qconv_tf_fr`: Active/reactive power flow from AC side
- `pconv_tf_to`, `qconv_tf_to`: Active/reactive power flow to filter side
- `vm`, `va`: AC bus voltage magnitude and angle
- `vmf`, `vaf`: Filter bus voltage magnitude and angle
- `z_conv_dc`: Binary converter status (1=on, 0=off)

# Constraints
When transformer is present:
- AC power flow equations with switching logic
- Power flows forced to zero when z_DC = 0

When no transformer:
- Direct connection constraints (zero impedance)
- Equal voltages and zero net power
"""
function constraint_conv_transformer_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)
    ptf_fr = _PM.var(pm, n, :pconv_tf_fr, i)
    qtf_fr = _PM.var(pm, n, :qconv_tf_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vm = _PM.var(pm, n, :vm, acbus)
    va = _PM.var(pm, n, :va, acbus)
    vmf = _PM.var(pm, n, :vmf, i)
    vaf = _PM.var(pm, n, :vaf, i)

    ztf = rtf + im*xtf
    if transformer
        ytf = 1/(rtf + im*xtf)
        gtf = real(ytf)
        btf = imag(ytf)
        gtf_sh = 0
        c1, c2, c3, c4 = ac_power_flow_constraints_dc_ots(pm.model, gtf, btf, gtf_sh, vm, vmf, va, vaf, ptf_fr, ptf_to, qtf_fr, qtf_to, tm, z_DC)
    else
        JuMP.@constraint(pm.model, ptf_fr + ptf_to == 0)
        JuMP.@constraint(pm.model, qtf_fr + qtf_to == 0)
        JuMP.@constraint(pm.model, va == vaf)
        JuMP.@constraint(pm.model, vm == vmf)
    end
end

"""
    constraint_converter_losses(pm::AbstractACPModel, n::Int, i::Int, a, b, c, plmax)

Converter loss model for AC/DC converters.

Implements the converter loss characteristic as a quadratic function of the converter
current. The loss model relates AC-side and DC-side power through converter losses.

# Arguments
- `pm::AbstractACPModel`: ACP power model formulation
- `n::Int`: Network identifier
- `i::Int`: Converter identifier
- `a`, `b`, `c`: Loss coefficients (constant, linear, quadratic)
- `plmax`: Maximum converter losses

# Converter Loss Model
P_ac + P_dc = a + b*I_conv + c*I_conv²

Where:
- P_ac: AC-side active power (positive = consumed from AC)
- P_dc: DC-side active power (positive = injected to DC)
- I_conv: Converter current magnitude
- a: No-load losses (constant term)
- b: Linear loss coefficient
- c: Quadratic loss coefficient

# Notes
- Losses are always positive (power consumed by converter)
- Higher currents result in higher losses (quadratic relationship)
- Model is valid for VSC and LCC converter technologies
"""
function constraint_converter_losses(pm::_PM.AbstractACPModel, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)

    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == a + b*iconv + c*iconv^2)
end

"""
    ac_power_flow_constraints_dc_ots(model, g, b, gsh_fr, vm_fr, vm_to, va_fr, va_to, p_fr, p_to, q_fr, q_to, tm, z_DC)

AC power flow equations with DC switching capability.

Implements the full nonlinear AC power flow equations for a transformer or line
with the ability to be switched on/off via binary variable z_DC. When z_DC = 0,
all power flows are forced to zero.

# Arguments
- `model`: JuMP optimization model
- `g`, `b`: Series conductance and susceptance
- `gsh_fr`: Shunt conductance at from side
- `vm_fr`, `vm_to`: Voltage magnitudes at from and to buses
- `va_fr`, `va_to`: Voltage angles at from and to buses
- `p_fr`, `p_to`: Active power flows from and to
- `q_fr`, `q_to`: Reactive power flows from and to
- `tm`: Transformer tap ratio
- `z_DC`: Binary switching variable

# Power Flow Equations
From side:
- P_fr = z_DC * [g/tm² * V_fr² - g/tm * V_fr * V_to * cos(θ_fr - θ_to) - b/tm * V_fr * V_to * sin(θ_fr - θ_to)]
- Q_fr = z_DC * [-b/tm² * V_fr² + b/tm * V_fr * V_to * cos(θ_fr - θ_to) - g/tm * V_fr * V_to * sin(θ_fr - θ_to)]

To side (similar with appropriate modifications)

# Returns
- Tuple of constraint references (c1, c2, c3, c4)
"""
function ac_power_flow_constraints_dc_ots(model, g, b, gsh_fr, vm_fr, vm_to, va_fr, va_to, p_fr, p_to, q_fr, q_to, tm, z_DC)
    c1 = JuMP.@constraint(model, p_fr == z_DC*( g/(tm^2)*vm_fr^2 + -g/(tm)*vm_fr*vm_to * cos(va_fr-va_to) + -b/(tm)*vm_fr*vm_to*sin(va_fr-va_to)))
    c2 = JuMP.@constraint(model, q_fr == z_DC*(-b/(tm^2)*vm_fr^2 +  b/(tm)*vm_fr*vm_to * cos(va_fr-va_to) + -g/(tm)*vm_fr*vm_to*sin(va_fr-va_to)))
    c3 = JuMP.@constraint(model, p_to == z_DC*( g*vm_to^2 + -g/(tm)*vm_to*vm_fr  *    cos(va_to - va_fr)  + -b/(tm)*vm_to*vm_fr*sin(va_to - va_fr)))
    c4 = JuMP.@constraint(model, q_to == z_DC*(-b*vm_to^2 +  b/(tm)*vm_to*vm_fr  *    cos(va_to - va_fr)  + -g/(tm)*vm_to*vm_fr*sin(va_to - va_fr)))
    return c1, c2, c3, c4
end

function constraint_conv_filter_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, bv, filter) #-> probably not needed
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vmf = _PM.var(pm, n, :vmf, i)

    JuMP.@constraint(pm.model,   ppr_fr + ptf_to == 0 )
    JuMP.@constraint(pm.model, qpr_fr + qtf_to +  (-bv) * filter *vmf^2 == 0)
end

function constraint_converter_losses_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == z_DC*(a + b*iconv + c*iconv^2))
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, rc, xc, reactor)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    ppr_to = - pconv_ac
    qpr_to = - qconv_ac
    ppr_fr = _PM.var(pm, n,  :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n,  :qconv_pr_fr, i)

    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vmf = _PM.var(pm, n, :vmf, i)
    vaf = _PM.var(pm, n, :vaf, i)
    vmc = _PM.var(pm, n, :vmc, i)
    vac = _PM.var(pm, n, :vac, i)

    zc = rc + im*xc
    if reactor
        yc = 1/(zc)
        gc = real(yc)
        bc = imag(yc)
        JuMP.@constraint(pm.model, - pconv_ac == z_DC*( gc*vmc^2 + -gc*vmc*vmf*cos(vac-vaf) + -bc*vmc*vmf*sin(vac-vaf))) # JuMP doesn't allow affine expressions in NL constraints
        JuMP.@constraint(pm.model, - qconv_ac == z_DC*(-bc*vmc^2 +  bc*vmc*vmf*cos(vac-vaf) + -gc*vmc*vmf*sin(vac-vaf))) # JuMP doesn't allow affine expressions in NL constraints
        JuMP.@constraint(pm.model, ppr_fr ==     z_DC*( gc *vmf^2 + -gc *vmf*vmc*cos(vaf - vac) + -bc *vmf*vmc*sin(vaf - vac)))
        JuMP.@constraint(pm.model, qpr_fr ==     z_DC*(-bc *vmf^2 +  bc *vmf*vmc*cos(vaf - vac) + -gc *vmf*vmc*sin(vaf - vac)))
    else
        JuMP.@constraint(pm.model, ppr_fr + ppr_to == 0)
        JuMP.@constraint(pm.model, qpr_fr + qpr_to == 0)
        JuMP.@constraint(pm.model, z_DC*vac == z_DC*vaf)
        JuMP.@constraint(pm.model, z_DC*vmc == z_DC*vmf)
    end
end

function constraint_conv_firing_angle(pm::_PM.AbstractACPModel, n::Int, i::Int, S, P1, Q1, P2, Q2)
    p = _PM.var(pm, n, :pconv_ac, i)
    q = _PM.var(pm, n, :qconv_ac, i)
    phi = _PM.var(pm, n, :phiconv, i)

    JuMP.@constraint(pm.model, p == cos(phi) * S)
    JuMP.@constraint(pm.model, q == sin(phi) * S)
end

function constraint_converter_current_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, Umax, Imax)
    vmc = _PM.var(pm, n, :vmc, i)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac = _PM.var(pm, n, :qconv_ac, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_dc = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@constraint(pm.model, pconv_ac^2 + qconv_ac^2 == z_dc * vmc^2 * iconv^2)
end

function variable_voltage_slack_ots(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=false)
end