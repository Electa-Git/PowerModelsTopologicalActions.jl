########################### OTS CONSTRAINTS #################################
function constraint_converter_losses_dc_ots(pm::_PM.AbstractWModels, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    iconv_sq = _PM.var(pm, n, :iconv_ac_sq, i)
    z = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == a*z + b*iconv + c*iconv_sq)
end

function constraint_conv_transformer_dc_ots(pm::_PM.AbstractWRModels, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)
    ptf_fr = _PM.var(pm, n,  :pconv_tf_fr, i)
    qtf_fr = _PM.var(pm, n,  :qconv_tf_fr, i)
    ptf_to = _PM.var(pm, n,  :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n,  :qconv_tf_to, i)

    w = _PM.var(pm, n,  :w, acbus)  # vm^2
    w_du = _PM.var(pm, n, :w_du_ots, i)
    wf = _PM.var(pm, n,  :wf_ac, i)   # vmf * vmf
    wrf = _PM.var(pm, n,  :wrf_ac, i) # vm*vmf*cos(va-vaf) =  vmf*vm*cos(vaf-va)
    wif = _PM.var(pm, n,  :wif_ac, i) # vm*vmf*sin(va-vaf) = -vmf*vm*sin(vaf-va)

    z = _PM.var(pm, n, :z_conv_dc, i)
    ztf = rtf + im*xtf

    if transformer
        ytf = 1/(rtf + im*xtf)
        gtf = real(ytf)
        btf = imag(ytf)
        c1, c2, c3, c4 = _PMACDC.ac_power_flow_constraints_w(pm, gtf, btf, w_du, wf, wrf, wif, ptf_fr, ptf_to, qtf_fr, qtf_to, tm)

        _IM.relaxation_equality_on_off(pm.model, w, w_du, z)
        JuMP.@constraint(pm.model, w_du >= z*JuMP.lower_bound(w))
        JuMP.@constraint(pm.model, w_du <= z*JuMP.upper_bound(w))
        constraint_voltage_product_converter_ots(pm, wrf, wif, wf, w, z)

    else
        pcon, qcon = constraint_lossless_section_ots(pm, w_du, wf, wrf, wif, ptf_fr, ptf_to, qtf_fr, qtf_to)
        _IM.relaxation_equality_on_off(pm.model, w, w_du, z)
        JuMP.@constraint(pm.model, w_du >= z*JuMP.lower_bound(w))
        JuMP.@constraint(pm.model, w_du <= z*JuMP.upper_bound(w))
    end
end

function constraint_conv_filter_dc_ots(pm::_PM.AbstractWModels, n::Int, i::Int, bv, filter) #-> probably not needed
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    wf = _PM.var(pm, n, :wf_ac, i)

    JuMP.@constraint(pm.model,   ppr_fr + ptf_to == 0 )
    JuMP.@constraint(pm.model, qpr_fr + qtf_to +  (-bv) * filter * wf == 0)
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractWRModels, n::Int, i::Int, rc, xc, reactor)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    ppr_to = - pconv_ac
    qpr_to = - qconv_ac
    ppr_fr = _PM.var(pm, n,  :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n,  :qconv_pr_fr, i)

    z = _PM.var(pm, n, :z_conv_dc, i)

    wf = _PM.var(pm, n,  :wf_ac, i)
    wc = _PM.var(pm, n,  :wc_ac, i)
    wrc = _PM.var(pm, n,  :wrc_ac, i)
    wic = _PM.var(pm, n,  :wic_ac, i)

    zc = rc + im*xc
    
    if reactor
        yc = 1/(zc)
        gc = real(yc)
        bc = imag(yc)
        c1, c2, c3, c4 = _PMACDC.ac_power_flow_constraints_w(pm, gc, bc, wf, wc, wrc, wic, ppr_fr, ppr_to, qpr_fr, qpr_to, 1)
        constraint_voltage_product_converter_ots(pm, wrc, wic, wf, wc, z)
    else
        pcon, qcon = constraint_lossless_section_ots(pm, wf, wc, wrc, wic, ppr_fr, ppr_to, qpr_fr, qpr_to)
    end

end

function constraint_lossless_section_ots(pm::_PM.AbstractWModels, w_fr, w_to, wr, wi, p_fr, p_to, q_fr, q_to)
    JuMP.@constraint(pm.model, w_fr ==  w_to)
    JuMP.@constraint(pm.model, wr   ==  w_fr)
    JuMP.@constraint(pm.model, wi   ==  0)

    pcon = JuMP.@constraint(pm.model, p_fr + p_to == 0)
    qcon = JuMP.@constraint(pm.model, q_fr + q_to == 0)
    return pcon, qcon
end

#function add_converter_voltage_setpoint_ots(sol, pm::_PM.AbstractWModels)
#    _PM.add_setpoint!(sol, pm, "convdc", "vmconv", :wc_ac_ne; scale = (x,item) -> sqrt(x))
#    _PM.add_setpoint!(sol, pm, "convdc", "vmfilt", :wf_ac_ne; scale = (x,item) -> sqrt(x))
#end

"""
LCC firing angle constraints

```
qconv_ac >= Q1 + (pconv_ac-P1) * (Q2-Q1)/(P2-P1)

P1 = cos(0) * Srated
Q1 = sin(0) * Srated
P2 = cos(pi) * Srated
Q2 = sin(pi) * Srated
```
"""
function constraint_conv_firing_angle_ots(pm::_PM.AbstractWModels, n::Int, i::Int, S, P1, Q1, P2, Q2)
    pc = _PM.var(pm, n, :pconv_ac_ne, i)
    qc = _PM.var(pm, n, :qconv_ac_ne, i)
    coeff = (Q2-Q1)/(P2-P1)

    JuMP.@constraint(pm.model, qc >= Q1 + (pc-P1) * coeff )
end

