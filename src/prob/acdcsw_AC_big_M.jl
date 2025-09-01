export run_acdcsw_AC_big_M
export run_acdcsw_AC_big_M_sp

export run_acdcsw_AC_grid
export run_acdcsw_AC_grid_sp

"""
    run_acdcsw_AC(file, model_constructor, optimizer; kwargs...)

Solve an AC/DC power system with AC busbar splitting using big-M formulation.

This function implements busbar splitting optimization for the AC portion of hybrid AC/DC 
power systems. Busbar splitting allows a single bus to be divided into multiple bus 
sections, with controllable switches determining the connectivity between sections.

Busbar splitting is particularly useful for:
- Increasing system flexibility and redundancy
- Reducing short-circuit currents
- Enabling maintenance without full bus outage
- Optimizing power flows through bus section management

# Arguments
- `file`: Path to power system data file or parsed data dictionary
- `model_constructor`: PowerModels formulation type (e.g., ACPPowerModel)
- `optimizer`: JuMP-compatible optimization solver (typically mixed-integer)
- `kwargs...`: Additional keyword arguments

# Returns
- `result::Dict`: Solution dictionary containing optimal switch positions and power flows

# Example
```julia
using PowerModelsTopologicalActionsII, PowerModels, PowerModelsACDC
using Ipopt, Juniper

# Setup mixed-integer solver
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0)  
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt)

# Solve AC busbar splitting problem
result = run_acdcsw_AC("case_with_switches.m", ACPPowerModel, juniper)
```
"""
function run_acdcsw_AC(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

"""
    build_acdcsw_AC(pm::AbstractPowerModel)

Build the optimization model for AC busbar splitting in AC/DC systems.

This function constructs the mathematical formulation for busbar splitting including:
- Standard AC bus voltage and generator power variables
- AC branch power flow variables  
- Binary switch indicator variables for controllable bus section switches
- Switch power flow variables (active when switch is closed)
- DC grid variables (fixed topology)
- Modified power balance constraints accounting for switch flows
- Switch operation constraints (big-M formulation)
- Standard AC/DC power flow constraints

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure with busbar splitting data
"""
function build_acdcsw_AC(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator(pm) # binary variable to indicate the status of an ac switch
    variable_switch_power(pm) # variable to indicate the power flowing through an ac switch (if closed)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # Objective function
    objective_min_fuel_cost_ac_switch(pm)

    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_switch(pm, i) # including the ac switches in the power balance of the AC part of an AC/DC grid
    end

    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i) # limiting the apparent power flowing through an ac switch
        constraint_switch_power_on_off(pm,i) # limiting the maximum active and reactive power through an ac switch
        constraint_switch_voltage_on_off_big_M(pm,i)
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_branch(pm,i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end

function run_acdcsw_AC_big_M_hour(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_big_M_hour; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_AC_big_M_hour(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator(pm) # binary variable to indicate the status of an ac switch
    variable_switch_power(pm) # variable to indicate the power flowing through an ac switch (if closed)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # Objective function
    objective_min_fuel_cost_ac_switch(pm)

    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_switch(pm, i) # including the ac switches in the power balance of the AC part of an AC/DC grid
    end

    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i) # limiting the apparent power flowing through an ac switch
        constraint_switch_power_on_off(pm,i) # limiting the maximum active and reactive power through an ac switch
        constraint_switch_voltage_on_off_big_M(pm,i)
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_branch(pm,i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end


""

# AC Busbar splitting for AC/DC grid
"ACDC opf with controllable switches in AC busbar splitting configuration for AC/DC grids"
function run_acdcsw_AC_sp(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_sp; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_AC_sp(pm::_PM.AbstractPowerModel)
    # AC grid
    variable_bus_voltage_sp(pm)
    variable_gen_power_sp(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator_sp(pm) # binary variable to indicate the status of an ac switch
    variable_switch_power(pm) # variable to indicate the power flowing through an ac switch (if closed)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # Objective function
    #_PM.objective_min_fuel_cost(pm)
    objective_min_fuel_cost_ac_switch(pm)

    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_switch(pm, i) # including the ac switches in the power balance of the AC part of an AC/DC grid
    end

    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i) # limiting the apparent power flowing through an ac switch
        constraint_switch_power_on_off(pm,i) # limiting the maximum active and reactive power through an ac switch
        constraint_switch_voltage_on_off_big_M(pm,i)
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_branch(pm,i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
        constraint_ZIL_switch(pm,i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

    # This to be removed probably
    #for i in _PM.ids(pm, :dcline)
    #    _PM.constraint_dcline_power_losses(pm, i)
    #end

    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        _PMACDC.constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end


function run_acdcsw_AC_grid(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_grid; ref_extensions=[_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_AC_grid(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator(pm) # binary variable to indicate the status of an ac switch
    variable_switch_power(pm) # variable to indicate the power flowing through an ac switch (if closed)

    # Objective function
    objective_min_fuel_cost_ac_switch(pm)
    #_PM.objective_min_fuel_cost(pm)
    # Constraints
    _PM.constraint_model_voltage(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_grid_ac_switch(pm, i) # including the ac switches in the power balance of the AC part of an AC/DC grid
    end

    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i) # limiting the apparent power flowing through an ac switch
        constraint_switch_voltage_on_off(pm,i) # making sure that the voltage magnitude and angles are equal at the two extremes of a closed switch
        constraint_switch_power_on_off(pm,i) # limiting the maximum active and reactive power through an ac switch
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_branch(pm,i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
        constraint_ZIL_switch(pm,i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end
end

function run_acdcsw_AC_grid_sp(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC_grid_sp; ref_extensions=[_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_AC_grid_sp(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    variable_switch_indicator_sp(pm) # binary variable to indicate the status of an ac switch
    variable_switch_power(pm) # variable to indicate the power flowing through an ac switch (if closed)

    # Objective function
    objective_min_fuel_cost_ac_switch(pm)
    #_PM.objective_min_fuel_cost(pm)
    # Constraints
    _PM.constraint_model_voltage(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_grid_ac_switch(pm, i) # including the ac switches in the power balance of the AC part of an AC/DC grid
    end

    for i in _PM.ids(pm, :switch)
        constraint_switch_thermal_limit(pm, i) # limiting the apparent power flowing through an ac switch
        constraint_switch_voltage_on_off(pm,i) # making sure that the voltage magnitude and angles are equal at the two extremes of a closed switch
        constraint_switch_power_on_off(pm,i) # limiting the maximum active and reactive power through an ac switch
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_branch(pm,i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
        constraint_ZIL_switch(pm,i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end
end



