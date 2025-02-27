########## Variables from PMACDC ##########
function variable_dcgrid_voltage_magnitude(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    vdcm = _PM.var(pm, nw)[:vdcm] = JuMP.JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :busdc)], base_name="$(nw)_vdcm",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, i), "Vdc", 1.0)
    )

    if bounded
        for (i, busdc) in _PM.ref(pm, nw, :busdc)
            JuMP.set_lower_bound(vdcm[i],  busdc["Vdcmin"])
            JuMP.set_upper_bound(vdcm[i],  busdc["Vdcmax"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :busdc, :vm, _PM.ids(pm, nw, :busdc), vdcm)
end

"variable: `vdcm[i]` for `i` in `dcbus`es"
function variable_dcgrid_voltage_magnitude_sqr(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    wdc = _PM.var(pm, nw)[:wdc] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :busdc)], base_name="$(nw)_wdc",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, i), "Vdc", 1.0)^2
    )
    wdcr = _PM.var(pm, nw)[:wdcr] = JuMP.@variable(pm.model,
    [(i,j) in _PM.ids(pm, nw, :buspairsdc)], base_name="$(nw)_wdcr",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, i), "Vdc", 1.0)^2
    )

    if bounded
        for (i, busdc) in _PM.ref(pm, nw, :busdc)
            JuMP.set_lower_bound(wdc[i],  busdc["Vdcmin"]^2)
            JuMP.set_upper_bound(wdc[i],  busdc["Vdcmax"]^2)
        end
        for (bp, buspairdc) in _PM.ref(pm, nw, :buspairsdc)
            JuMP.set_lower_bound(wdcr[bp],  0)
            JuMP.set_upper_bound(wdcr[bp],  buspairdc["vm_fr_max"] * buspairdc["vm_to_max"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :busdc, :wdc, _PM.ids(pm, nw, :busdc), wdc)
end




########## OTS variables ###########
function variable_branch_indicator_linearised_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :branch)
        br[l] = _PM.ref(pm,nw,:branch,l)
    end
 
    z_branch = _PM.var(pm, nw)[:z_branch] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :branch)], base_name="$(nw)_z_branch",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :branch, l), "z_branch_start", br[l]["br_status_initial"])
    )
 
    report && _PM.sol_component_value(pm, nw, :branch, :br_status, _PM.ids(pm, nw, :branch), z_branch)
end

function variable_branch_indicator_linearised(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :branch)
        br[l] = _PM.ref(pm,nw,:branch,l)
    end
 
    z_branch = _PM.var(pm, nw)[:z_branch] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :branch)], base_name="$(nw)_z_branch",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :branch, l), "z_branch_start", 1.0)
    )
 
    report && _PM.sol_component_value(pm, nw, :branch, :br_status, _PM.ids(pm, nw, :branch), z_branch)
end


function variable_dc_branch_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    if !relax
        z_ots_dc_var = _PM.var(pm, nw)[:z_ots_dc] = JuMP.@variable(pm.model,
            [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_z_ots_dc",
            binary = true,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "z_ots_start_dc", 1.0)
        )
    else
        z_ots_dc_var = _PM.var(pm, nw)[:z_ots_dc] = JuMP.@variable(pm.model,
            [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_z_ots_dc",
            lower_bound = 0.0,
            upper_bound = 1.0,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "z_ots_start_dc", 1.0)
        )
    end

    report && _PM.sol_component_value(pm, nw, :branchdc, :br_status, _PM.ids(pm, nw, :branchdc), z_ots_dc_var)
end

function variable_dc_branch_indicator_linearised_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :branchdc)
        br[l] = _PM.ref(pm,nw,:branchdc,l)
    end
 
    z_ots_dc_var = _PM.var(pm, nw)[:z_ots_dc] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_z_ots_dc",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "z_ots_start_dc", br[l]["br_status_initial"])
    )
 
    report && _PM.sol_component_value(pm, nw, :branchdc, :br_status, _PM.ids(pm, nw, :branchdc), z_ots_dc_var)
end

function variable_dc_branch_indicator_linearised(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :branchdc)
        br[l] = _PM.ref(pm,nw,:branchdc,l)
    end
 
    z_ots_dc_var = _PM.var(pm, nw)[:z_ots_dc] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_z_ots_dc",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "z_ots_start_dc", 1.0)
    )
 
    report && _PM.sol_component_value(pm, nw, :branchdc, :br_status, _PM.ids(pm, nw, :branchdc), z_ots_dc_var)
end

function variable_dc_conv_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    if !relax
        z_ots_conv_DC_var = _PM.var(pm, nw)[:z_conv_dc] = JuMP.@variable(pm.model,
            [l in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_z_conv_DC",
            binary = true,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, l), "z_conv_start_DC", 1.0)
        )
    else
        z_ots_conv_DC_var = _PM.var(pm, nw)[:z_conv_dc] = JuMP.@variable(pm.model,
            [l in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_z_conv_DC",
            lower_bound = 0.0,
            upper_bound = 1.0,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, l), "z_conv_start_DC", 1.0)
        )
    end

    report && _PM.sol_component_value(pm, nw, :convdc, :conv_status, _PM.ids(pm, nw, :convdc), z_ots_conv_DC_var)
end

function variable_dc_conv_indicator_linearised_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :convdc)
        br[l] = _PM.ref(pm,nw,:convdc,l)
    end
 
    z_ots_conv_DC_var = _PM.var(pm, nw)[:z_conv_dc] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_z_conv_DC",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, l), "z_conv_start_DC", br[l]["conv_status_initial"])
    )
 
    report && _PM.sol_component_value(pm, nw, :convdc, :conv_status, _PM.ids(pm, nw, :convdc), z_ots_conv_DC_var)
end

function variable_dc_conv_indicator_linearised(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :convdc)
        br[l] = _PM.ref(pm,nw,:convdc,l)
    end
 
    z_ots_conv_DC_var = _PM.var(pm, nw)[:z_conv_dc] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_z_conv_DC",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, l), "z_conv_start_DC", 1.0)
    )
 
    report && _PM.sol_component_value(pm, nw, :convdc, :conv_status, _PM.ids(pm, nw, :convdc), z_ots_conv_DC_var)
end

"variable: `wdcm[i]` for `i` in `busdc"
function variable_dcgrid_voltage_magnitude_sqr_ots(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    bi_bp = Dict([(i, (b["fbusdc"], b["tbusdc"])) for (i,b) in _PM.ref(pm, nw, :branchdc)])
    bus_vdcmax = merge(Dict([(b,bus["Vdcmax"]) for (b,bus) in _PM.ref(pm, nw, :busdc)]),
    Dict([(b,bus["Vdcmax"]) for (b,bus) in _PM.ref(pm, nw, :busdc)]))
    bus_vdcmin = merge(Dict([(b,bus["Vdcmin"]) for (b,bus) in _PM.ref(pm, nw, :busdc)]),
    Dict([(b,bus["Vdcmin"]) for (b,bus) in _PM.ref(pm, nw, :busdc)]))
         # display(_PM.ids(pm, nw, :buspairsdc_ne))
        wdc_ots = _PM.var(pm, nw)[:wdc_ots] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :busdc)], base_name="$(nw)_wdc_ots",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, i), "Vdc",  1.0)^2,
        )
        wdcr_ots = _PM.var(pm, nw)[:wdcr_ots] = JuMP.@variable(pm.model,
        [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdcr_ots",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[l][1]), "Vdc",  1.0)^2,
        )
        if bounded
            for (i, busdc) in _PM.ref(pm, nw, :busdc)
                JuMP.set_lower_bound(wdc_ots[i],  busdc["Vdcmin"]^2)
                JuMP.set_upper_bound(wdc_ots[i],  busdc["Vdcmax"]^2)
            end
            for (br, branchdc) in _PM.ref(pm, nw, :branchdc)
                JuMP.set_lower_bound(wdcr_ots[br],  0)
                JuMP.set_upper_bound(wdcr_ots[br],  bus_vdcmax[bi_bp[br][1]] * bus_vdcmax[bi_bp[br][2]])
            end
        end
        report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :busdc, :wdc_ots, _PM.ids(pm, nw, :busdc), wdc_ots)
        report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdcr_ots, _PM.ids(pm, nw, :branchdc), wdcr_ots)
end


"variable: `wdcm[i]` for `i` in `ne_dcbus`es"
function variable_dcgrid_voltage_magnitude_sqr_ots_du(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true) # this has to to every branch, different than its counterpart(Wdc_fr) since two candidate branches can be connected to same node and two duplicate variables will be needed
    bi_bp = Dict([(i, (b["fbusdc"], b["tbusdc"])) for (i,b) in _PM.ref(pm, nw, :branchdc)])
    wdc_fr_ots = _PM.var(pm, nw)[:wdc_du_fr_ots] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdc_du_fr",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[i][1]), "Vdc",  1.0)^2,
    )
    wdc_to_ots = _PM.var(pm, nw)[:wdc_du_to_ots] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdc_du_to",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[i][1]), "Vdc",  1.0)^2,
    )
    #TODO [OLD FROM PMACDC] replace wdc_du_fr and wdc_du_to with wdc_fr and wdc_to make make it consistent with PM, there multiplication is defined by wr - real and wi- imag
    wdcr_frto_ots = _PM.var(pm, nw)[:wdcr_du] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdcr_du",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[i][1]), "Vdc",  1.0)^2,
    )

    if bounded
        for (i, branchdc) in _PM.ref(pm, nw, :branchdc)
            JuMP.set_lower_bound(wdc_fr_ots[i],  0)
            JuMP.set_upper_bound(wdc_fr_ots[i],  1.21)
            JuMP.set_lower_bound(wdc_to_ots[i],  0)
            JuMP.set_upper_bound(wdc_to_ots[i],  1.21)
            JuMP.set_lower_bound(wdcr_frto_ots[i],  0)
            JuMP.set_upper_bound(wdcr_frto_ots[i],  1.21)
        end
    end
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdc_du_fr_ots, _PM.ids(pm, nw, :branchdc), wdc_fr_ots)
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdc_du_to_ots, _PM.ids(pm, nw, :branchdc), wdc_to_ots)
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdcr_du_ots, _PM.ids(pm, nw, :branchdc), wdcr_frto_ots)
end

"variable: `p_dcgrid[l,i,j]` for `(l,i,j)` in `arcs_dcgrid`"
function variable_active_dcbranch_flow_ots(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
        p = _PM.var(pm, nw)[:p_dcgrid_ots]= JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_dcgrid)], base_name="$(nw)_pdcgrid_ne",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "p_start",  1.0)
        )

    if bounded
        for arc in _PM.ref(pm, nw, :arcs_dcgrid)
            l,i,j = arc
            JuMP.set_lower_bound(p[arc], -_PM.ref(pm, nw, :branchdc, l)["rateA"])
            JuMP.set_upper_bound(p[arc],  _PM.ref(pm, nw, :branchdc, l)["rateA"])
        end
    end

    report && _IM.sol_component_value_edge(pm, _PM.pm_it_sym, nw, :branchdc, :pf, :pt, _PM.ref(pm, nw, :arcs_dcgrid_from), _PM.ref(pm, nw, :arcs_dcgrid_to), p)
end

"variable: `ccm_dcgrid[l]` for `(l)` in `branchdc`"
function variable_dcbranch_current_sqr_ots(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    vpu = 0.8
    cc= _PM.var(pm, nw)[:ccm_dcgrid] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_ccm_dcgrid_ne",
    start = (_PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "p_start",  0.0) / vpu)^2
    )

    if bounded
        for (l, branchdc) in _PM.ref(pm, nw, :branchdc)
            JuMP.set_lower_bound(cc[l], 0)
            JuMP.set_upper_bound(cc[l], (branchdc["rateA"] / vpu)^2)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :ccm, _PM.ids(pm, nw, :branchdc), cc)
end

"variable: `0 <= convdc_ne[c] <= 1` for `c` in `candidate converters"
function variable_branch_ots(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    if !relax
        Z_dc_branch = _PM.var(pm, nw)[:branchdc] = JuMP.@variable(pm.model, #branch_ne is also name in PowerModels, branchdc_ne is candidate branches
        [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_branch_ne",
        binary = true,
        start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "convdc_tnep_start",  0.0)
        )
    else
        Z_dc_branch_ne = _PM.var(pm, nw)[:branchdc] = JuMP.@variable(pm.model, #branch_ne is also name in PowerModels, branchdc_ne is candidate branches
        [l in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_branch_ne",
        lower_bound = 0,
        upper_bound = 1,
        start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "convdc_tnep_start",  0.0)
        )
    end
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :isbuilt, _PM.ids(pm, nw, :branchdc), Z_dc_branch_ne)
end


"variable: `w_du[j]` for `j` in `convdc`"
function variable_voltage_slack_ots(pm::_PM.AbstractWModels; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=false)
    w_du = _PM.var(pm, nw)[:w_du_ots] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_w_du_ots",
    start = 0,  # check
    )
    if bounded
        for (c, convdc) in _PM.ref(pm, nw, :convdc)
            JuMP.set_lower_bound(w_du[c], 0) # check
            JuMP.set_upper_bound(w_du[c], 2) # check
        end
    end
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :w_du_ots, _PM.ids(pm, nw, :convdc), w_du)
end

########## Busbar splitting variables ###########
# AC grid -> no need for new variables, already implemented in PowerModels, copying them here to modify them easily
function variable_switch_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    if !relax
        z_switch = _PM.var(pm, nw)[:z_switch] = JuMP.@variable(pm.model,
            [i in _PM.ids(pm, nw, :switch)], base_name="$(nw)_z_switch",
            binary = true,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, i), "z_switch_start", 1.0)
            #start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, i), "z_switch_start", 1.0)
            )
    else
        z_switch = _PM.var(pm, nw)[:z_switch] = JuMP.@variable(pm.model,
            [i in _PM.ids(pm, nw, :switch)], base_name="$(nw)_z_switch",
            lower_bound = 0.0,
            upper_bound = 1.0,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, i), "z_switch_start", 1.0)
        )
    end
    report && _PM.sol_component_value(pm, nw, :switch, :status, _PM.ids(pm, nw, :switch), z_switch)
end

function variable_switch_indicator_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    z_switch = _PM.var(pm, nw)[:z_switch] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_z_switch",
    binary = true,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "starting_value")
    )
 
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :switch, :status, _PM.ids(pm, nw, :switch), z_switch)
end

function variable_switch_indicator_linearised(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :switch)
        br[l] = _PM.ref(pm,nw,:switch,l)
    end
 
    z_switch = _PM.var(pm, nw)[:z_switch] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_z_switch",
    lower_bound = 0.0,
    upper_bound = 1.0,
    #start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 0.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :status, _PM.ids(pm, nw, :switch), z_switch)
end

function variable_switch_indicator_linearised_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    br = Dict()
    for l in _PM.ids(pm, nw, :switch)
        br[l] = _PM.ref(pm,nw,:switch,l)
    end
 
    z_switch = _PM.var(pm, nw)[:z_switch] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_z_switch",
    lower_bound = 0.0,
    upper_bound = 1.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", br[l]["switch_status_initial"])
    )
 
    report && _PM.sol_component_value(pm, nw, :switch, :status, _PM.ids(pm, nw, :switch), z_switch)
end

#Current through the switches to model the protections based on it
function variable_switch_current_real(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    #i_sw_r = _PM.var(pm, nw)[:i_sw_r] = JuMP.@variable(pm.model,
    #    [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_i_sw_r",
    #    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "psw_start")
    #)

    i_sw_r = JuMP.@variable(pm.model,
    [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_i_sw_r",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "psw_start")
    )

    # this explicit type erasure is necessary
    i_sw_r_expr = Dict{Any,Any}( (l,i,j) => i_sw_r[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw) )
    i_sw_r_expr = merge(i_sw_r_expr, Dict( (l,j,i) => -1.0*i_sw_r[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)))
    _PM.var(pm, nw)[:i_sw_r] = i_sw_r_expr

    report && _PM.sol_component_value_edge(pm, nw, :switch, :i_sw_r_fr, :i_sw_r_to, _PM.ref(pm, nw, :arcs_from_sw), _PM.ref(pm, nw, :arcs_to_sw), i_sw_r_expr)
end

function variable_dc_switch_current(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    i_sw_dc = JuMP.@variable(pm.model,
    [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw_dc)], base_name="$(nw)_i_sw_dc",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "psw_start")
    )

    # this explicit type erasure is necessary
    i_sw_dc_expr = Dict{Any,Any}( (l,i,j) => i_sw_dc[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw_dc) )
    i_sw_dc_expr = merge(i_sw_dc_expr, Dict( (l,j,i) => -1.0*i_sw_dc[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw_dc)))
    _PM.var(pm, nw)[:i_sw_dc] = i_sw_dc_expr

    report && _PM.sol_component_value_edge(pm, nw, :dcswitch, :i_sw_dc_fr, :i_sw_dc_to, _PM.ref(pm, nw, :arcs_from_sw_dc), _PM.ref(pm, nw, :arcs_to_sw_dc), i_sw_dc_expr)
end

function variable_switch_current_imaginary(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    #i_sw_i = _PM.var(pm, nw)[:i_sw_i] = JuMP.@variable(pm.model,
    #    [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_i_sw_i",
    #    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "psw_start")
    #)

    i_sw_i = JuMP.@variable(pm.model,
    [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_i_sw_i",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "psw_start")
    )

    # this explicit type erasure is necessary
    i_sw_i_expr = Dict{Any,Any}( (l,i,j) => i_sw_i[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw) )
    i_sw_i_expr = merge(i_sw_i_expr, Dict( (l,j,i) => -1.0*i_sw_i[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)))
    _PM.var(pm, nw)[:i_sw_i] = i_sw_i_expr

    report && _PM.sol_component_value_edge(pm, nw, :switch, :i_sw_i_fr, :i_sw_i_to, _PM.ref(pm, nw, :arcs_from_sw), _PM.ref(pm, nw, :arcs_to_sw), i_sw_i_expr)
end

function variable_switch_current(pm::_PM.AbstractPowerModel; kwargs...)
    variable_switch_current_real(pm; kwargs...)
    variable_switch_current_imaginary(pm; kwargs...)
end

""
function variable_switch_power(pm::_PM.AbstractPowerModel; kwargs...)
    variable_switch_power_real(pm; kwargs...)
    variable_switch_power_imaginary(pm; kwargs...)
end

function variable_switch_power_sp(pm::_PM.AbstractPowerModel; kwargs...)
    variable_switch_power_real_sp(pm; kwargs...)
    variable_switch_power_imaginary_sp(pm; kwargs...)
end


"variable: `pws[l,i,j]` for `(l,i,j)` in `arcs_sw`"
function variable_switch_power_real(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    psw = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_psw",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "psw_start")
    )

    if bounded
        flow_lb, flow_ub = _PM.ref_calc_switch_flow_bounds(_PM.ref(pm, nw, :switch), _PM.ref(pm, nw, :bus))

        for arc in _PM.ref(pm, nw, :arcs_from_sw)
            l,i,j = arc
            if !isinf(flow_lb[l])
                JuMP.set_lower_bound(psw[arc], flow_lb[l])
            end
            if !isinf(flow_ub[l])
                JuMP.set_upper_bound(psw[arc], flow_ub[l])
            end
        end
    end

    # this explicit type erasure is necessary
    psw_expr = Dict{Any,Any}( (l,i,j) => psw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw) )
    psw_expr = merge(psw_expr, Dict( (l,j,i) => -1.0*psw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)))
    _PM.var(pm, nw)[:psw] = psw_expr

    report && _PM.sol_component_value_edge(pm, nw, :switch, :psw_fr, :psw_to, _PM.ref(pm, nw, :arcs_from_sw), _PM.ref(pm, nw, :arcs_to_sw), psw_expr)
end


"variable: `pws[l,i,j]` for `(l,i,j)` in `arcs_sw`"
function variable_switch_power_imaginary(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    qsw = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_qsw",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "qsw_start")
    )

    if bounded
        flow_lb, flow_ub = _PM.ref_calc_switch_flow_bounds(_PM.ref(pm, nw, :switch), _PM.ref(pm, nw, :bus))

        for arc in _PM.ref(pm, nw, :arcs_from_sw)
            l,i,j = arc
            if !isinf(flow_lb[l])
                JuMP.set_lower_bound(qsw[arc], flow_lb[l])
            end
            if !isinf(flow_ub[l])
                JuMP.set_upper_bound(qsw[arc], flow_ub[l])
            end
        end
    end

    # this explicit type erasure is necessary
    qsw_expr = Dict{Any,Any}( (l,i,j) => qsw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw) )
    qsw_expr = merge(qsw_expr, Dict( (l,j,i) => -1.0*qsw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)))
    _PM.var(pm, nw)[:qsw] = qsw_expr

    report && _PM.sol_component_value_edge(pm, nw, :switch, :qsw_fr, :qsw_to, _PM.ref(pm, nw, :arcs_from_sw), _PM.ref(pm, nw, :arcs_to_sw), qsw_expr)
end

"variable: `pws[l,i,j]` for `(l,i,j)` in `arcs_sw`"
function variable_switch_power_real_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    psw = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_psw",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "psw_start", sp_switch)
    )

    if bounded
        flow_lb, flow_ub = _PM.ref_calc_switch_flow_bounds(_PM.ref(pm, nw, :switch), _PM.ref(pm, nw, :bus))

        for arc in _PM.ref(pm, nw, :arcs_from_sw)
            l,i,j = arc
            if !isinf(flow_lb[l])
                JuMP.set_lower_bound(psw[arc], flow_lb[l])
            end
            if !isinf(flow_ub[l])
                JuMP.set_upper_bound(psw[arc], flow_ub[l])
            end
        end
    end

    # this explicit type erasure is necessary
    psw_expr = Dict{Any,Any}( (l,i,j) => psw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw) )
    psw_expr = merge(psw_expr, Dict( (l,j,i) => -1.0*psw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)))
    _PM.var(pm, nw)[:psw] = psw_expr

    report && _PM.sol_component_value_edge(pm, nw, :switch, :psw_fr, :psw_to, _PM.ref(pm, nw, :arcs_from_sw), _PM.ref(pm, nw, :arcs_to_sw), psw_expr)
end


"variable: `pws[l,i,j]` for `(l,i,j)` in `arcs_sw`"
function variable_switch_power_imaginary_sp(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    qsw = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)], base_name="$(nw)_qsw",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "qsw_start", sp_switch)
    )

    if bounded
        flow_lb, flow_ub = _PM.ref_calc_switch_flow_bounds(_PM.ref(pm, nw, :switch), _PM.ref(pm, nw, :bus))

        for arc in _PM.ref(pm, nw, :arcs_from_sw)
            l,i,j = arc
            if !isinf(flow_lb[l])
                JuMP.set_lower_bound(qsw[arc], flow_lb[l])
            end
            if !isinf(flow_ub[l])
                JuMP.set_upper_bound(qsw[arc], flow_ub[l])
            end
        end
    end

    # this explicit type erasure is necessary
    qsw_expr = Dict{Any,Any}( (l,i,j) => qsw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw) )
    qsw_expr = merge(qsw_expr, Dict( (l,j,i) => -1.0*qsw[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw)))
    _PM.var(pm, nw)[:qsw] = qsw_expr

    report && _PM.sol_component_value_edge(pm, nw, :switch, :qsw_fr, :qsw_to, _PM.ref(pm, nw, :arcs_from_sw), _PM.ref(pm, nw, :arcs_to_sw), qsw_expr)
end

# DC grid
function variable_dc_switch_indicator(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, relax::Bool=false, report::Bool=true)
    if !relax
        z_dcswitch = _PM.var(pm, nw)[:z_dcswitch] = JuMP.@variable(pm.model,
            [i in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_z_dcswitch",
            binary = true,
            start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, i), "z_dcswitch_start", 1.0)
        )
    else
        z_dcswitch = _PM.var(pm, nw)[:z_dcswitch] = JuMP.@variable(pm.model,
            [i in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_z_dcswitch",
            lower_bound = 0,
            upper_bound = 1,
            start = _PM.comp_start_value(ref(pm, nw, :dcswitch, i), "z_dcswitch_start", 1.0)
        )
    end

    report && _PM.sol_component_value(pm, nw, :dcswitch, :status, _PM.ids(pm, nw, :dcswitch), z_dcswitch)
end

function variable_dc_switch_test(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    p_test = _PM.var(pm, nw)[:p_test] = JuMP.@variable(pm.model,
    [(l,i,j) in _PM.ref(pm, nw, :arcs_dcgrid)], base_name="$(nw)_pdcgrid",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "p_start", 1.0)
    )

    if bounded
        for arc in _PM.ref(pm, nw, :arcs_dcgrid)
            l,i,j = arc
            JuMP.set_lower_bound(p_test[arc], -_PM.ref(pm, nw, :branchdc, l)["rateA"])
            JuMP.set_upper_bound(p_test[arc],  _PM.ref(pm, nw, :branchdc, l)["rateA"])
        end
    end

    report && _IM.sol_component_value_edge(pm, _PM.pm_it_sym, nw, :branchdc, :pf, :pt, _PM.ref(pm, nw, :arcs_dcgrid_from), _PM.ref(pm, nw, :arcs_dcgrid_to), p_test)
end

function variable_active_dcbranch_flow(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    p = _PM.var(pm, nw)[:p_dcgrid] = JuMP.@variable(pm.model,
    [(l,i,j) in _PM.ref(pm, nw, :arcs_dcgrid)], base_name="$(nw)_pdcgrid",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :branchdc, l), "p_start", 1.0)
    )

    if bounded
        for arc in _PM.ref(pm, nw, :arcs_dcgrid)
            l,i,j = arc
            JuMP.set_lower_bound(p[arc], -_PM.ref(pm, nw, :branchdc, l)["rateA"])
            JuMP.set_upper_bound(p[arc],  _PM.ref(pm, nw, :branchdc, l)["rateA"])
        end
    end

    report && _IM.sol_component_value_edge(pm, _PM.pm_it_sym, nw, :branchdc, :pf, :pt, _PM.ref(pm, nw, :arcs_dcgrid_from), _PM.ref(pm, nw, :arcs_dcgrid_to), p)
end

"variable: `p_dc_sw[l,i,j]` for `(l,i,j)` in `arcs_dc_sw`"
function variable_dc_switch_power(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, report::Bool=true)
    #p_dc_sw_ = _PM.var(pm, nw)[:p_dc_sw] = JuMP.@variable(pm.model,
    p_dc_sw_ = JuMP.@variable(pm.model,
        [(l,i,j) in _PM.ref(pm, nw, :arcs_from_sw_dc)], base_name="$(nw)_p_dc_sw",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "p_dc_sw_start", 0.5)
    )
    
    if bounded
        flow_lb, flow_ub = _PM.ref_calc_switch_flow_bounds(_PM.ref(pm, nw, :dcswitch), _PM.ref(pm, nw, :busdc))
        for arc in _PM.ref(pm, nw, :arcs_from_sw_dc)
            l,i,j = arc
            if !isinf(flow_lb[l])
                JuMP.set_lower_bound(p_dc_sw_[arc], flow_lb[l])
            end
            if !isinf(flow_ub[l])
                JuMP.set_upper_bound(p_dc_sw_[arc], flow_ub[l])
            end
        end
    end
    
    # this explicit type erasure is necessary
    p_dc_sw_expr = Dict{Any,Any}( (l,i,j) => p_dc_sw_[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw_dc) )
    p_dc_sw_expr = merge(p_dc_sw_expr, Dict( (l,j,i) => -1.0*p_dc_sw_[(l,i,j)] for (l,i,j) in _PM.ref(pm, nw, :arcs_from_sw_dc)))
    _PM.var(pm, nw)[:p_dc_sw] = p_dc_sw_expr
    
    report && _PM.sol_component_value_edge(pm, nw, :dcswitch, :p_dc_sw_fr, :p_dc_sw_to, _PM.ref(pm, nw, :arcs_from_sw_dc), _PM.ref(pm, nw, :arcs_to_sw_dc), p_dc_sw_expr)
end

###################### Bilinear terms reformulation ############################

function auxiliary_variable_switch_voltage_angle(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_switch_va = _PM.var(pm, nw)[:aux_switch_va] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :switch)], base_name="$(nw)aux_switch_va",
    lower_bound = -2.0,
    upper_bound = 2.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, i), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :aux_switch_va, _PM.ids(pm, nw, :switch), aux_switch_va)

end

function auxiliary_variable_switch_voltage_angle(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_switch_va = _PM.var(pm, nw)[:aux_switch_va] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :switch)], base_name="$(nw)aux_switch_va",
    lower_bound = -2.0,
    upper_bound = 2.0,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, i), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :aux_switch_va, _PM.ids(pm, nw, :switch), aux_switch_va)

end

function auxiliary_variable_switch_voltage_angle(pm::_PM.AbstractWRModels; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
end

function auxiliary_variable_switch_voltage_magnitude(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_switch_vm = _PM.var(pm, nw)[:aux_switch_vm] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_aux_switch_vm",
    lower_bound = -0.3,
    upper_bound = 0.3,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :aux_switch_vm, _PM.ids(pm, nw, :switch), aux_switch_vm)
end

function auxiliary_variable_switch_voltage_magnitude(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_switch_phi = _PM.var(pm, nw)[:aux_switch_phi] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_aux_switch_phi",
    lower_bound = - 0.3,
    upper_bound = 0.3,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :aux_switch_phi, _PM.ids(pm, nw, :switch), aux_switch_phi)
end

function auxiliary_variable_switch_voltage_magnitude(pm::_PM.AbstractWRModels; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_switch_w = _PM.var(pm, nw)[:aux_switch_w] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_aux_switch_w",
    lower_bound = - 0.3,
    upper_bound = 0.3,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :aux_switch_w, _PM.ids(pm, nw, :switch), aux_switch_w)
end

function auxiliary_diff_switch_voltage_angle(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_switch_va = _PM.var(pm, nw)[:delta_switch_va] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_delta_switch_va",
    lower_bound = - 1.05,
    upper_bound = 1.05,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :delta_switch_va, _PM.ids(pm, nw, :switch), delta_switch_va)
end

function auxiliary_diff_switch_voltage_angle(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_switch_va = _PM.var(pm, nw)[:delta_switch_va] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_delta_switch_va",
    lower_bound = - 1.05,
    upper_bound = 1.05,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :delta_switch_va, _PM.ids(pm, nw, :switch), delta_switch_va)
end

function auxiliary_diff_switch_voltage_angle(pm::_PM.AbstractWRModels; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
end

function auxiliary_diff_switch_voltage_magnitude(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_switch_vm = _PM.var(pm, nw)[:delta_switch_vm] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)aux_switch_vm",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )
    print(delta_switch_vm)

    report && _PM.sol_component_value(pm, nw, :switch, :delta_switch_vm, _PM.ids(pm, nw, :switch), delta_switch_vm)
end

function auxiliary_diff_switch_voltage_magnitude(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_switch_phi = _PM.var(pm, nw)[:delta_switch_phi] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_aux_switch_phi",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :delta_switch_phi, _PM.ids(pm, nw, :switch), delta_switch_phi)
end

function auxiliary_diff_switch_voltage_magnitude(pm::_PM.AbstractWRModels; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_switch_w = _PM.var(pm, nw)[:delta_switch_w] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :switch)], base_name="$(nw)_aux_switch_w",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :switch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :switch, :delta_switch_w, _PM.ids(pm, nw, :switch), delta_switch_w)
end

function auxiliary_variable_DC_switch_voltage_magnitude(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_dcswitch_vm = _PM.var(pm, nw)[:aux_dcswitch_vm] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_aux_dcswitch_vm",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :dcswitch, :aux_dcswitch_vm, _PM.ids(pm, nw, :dcswitch), aux_dcswitch_vm)
end

function auxiliary_variable_DC_switch_voltage_magnitude(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_dcswitch_phi = _PM.var(pm, nw)[:aux_dcswitch_phi] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_aux_dcswitch_phi",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :dcswitch, :aux_dcswitch_phi, _PM.ids(pm, nw, :dcswitch), aux_dcswitch_phi)
end

function auxiliary_variable_DC_switch_voltage_magnitude(pm::_PM.AbstractWRModels; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    aux_dcswitch_w = _PM.var(pm, nw)[:aux_dcswitch_w] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_aux_dcswitch_w",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :dcswitch, :aux_dcswitch_w, _PM.ids(pm, nw, :dcswitch), aux_dcswitch_w)
end

function auxiliary_diff_DC_switch_voltage_magnitude(pm::_PM.AbstractACPModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_dcswitch_vm = _PM.var(pm, nw)[:delta_dcswitch_vm] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_diff_dcswitch_vm",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :dcswitch, :delta_dcswitch_vm, _PM.ids(pm, nw, :dcswitch), delta_dcswitch_vm)
end

function auxiliary_diff_DC_switch_voltage_magnitude(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_dcswitch_phi = _PM.var(pm, nw)[:delta_dcswitch_phi] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_diff_dcswitch_vm",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :dcswitch, :delta_dcswitch_phi, _PM.ids(pm, nw, :dcswitch), delta_dcswitch_phi)
end

function auxiliary_diff_DC_switch_voltage_magnitude(pm::_PM.AbstractWRModels; nw::Int=_PM.nw_id_default, bounded::Bool=true, relax::Bool=false, report::Bool=true)
    delta_dcswitch_w = _PM.var(pm, nw)[:delta_dcswitch_w] = JuMP.@variable(pm.model,
    [l in _PM.ids(pm, nw, :dcswitch)], base_name="$(nw)_diff_dcswitch_w",
    lower_bound = -0.21,
    upper_bound = 0.21,
    start = _PM.comp_start_value(_PM.ref(pm, nw, :dcswitch, l), "z_switch_start", 1.0)
    )

    report && _PM.sol_component_value(pm, nw, :dcswitch, :delta_dcswitch_w, _PM.ids(pm, nw, :dcswitch), delta_dcswitch_w)
end
