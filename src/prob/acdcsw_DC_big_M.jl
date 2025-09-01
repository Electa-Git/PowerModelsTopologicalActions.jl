export run_acdcsw_DC_reformulation

"""
    run_acdcsw_DC_big_M(file, model_constructor, optimizer; kwargs...)

Solve DC busbar splitting problem for AC/DC grids using big-M formulation.

This function implements busbar splitting optimization for the DC portion of hybrid 
AC/DC power systems using a big-M mixed-integer formulation. DC busbar splitting 
allows DC buses to be divided into sections with controllable switches, providing:
- Enhanced DC grid flexibility and controllability
- Reduced DC fault currents through bus sectioning
- Optimized DC power flows via selective bus connectivity
- Improved DC grid reliability and maintenance options

# Arguments
- `file`: Path to power system data file or parsed data dictionary
- `model_constructor`: PowerModels formulation type (e.g., ACPPowerModel)
- `optimizer`: JuMP-compatible mixed-integer optimization solver
- `kwargs...`: Additional keyword arguments

# Returns
- `result::Dict`: Solution dictionary containing optimal DC switch positions and power flows

# Big-M Formulation
The big-M approach uses large constants to enforce switching logic:
- When switch is closed (z = 1): normal power flow constraints apply
- When switch is open (z = 0): power flow is forced to zero via big-M constraints

# Mathematical Features
- Binary variables for DC switch status
- Auxiliary bilinear variables for voltage relationships
- DC power flow variables with switching logic
- Mixed-integer nonlinear programming (MINLP) formulation

# Example
```julia
using PowerModelsTopologicalActionsII, PowerModels, PowerModelsACDC
using Ipopt, Juniper

# Setup MINLP solver
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt)

# Solve DC busbar splitting with big-M
result = run_acdcsw_DC_big_M("case_with_dc_switches.m", ACPPowerModel, juniper)
```
"""
function run_acdcsw_DC_big_M(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_DC_big_M; ref_extensions=[add_ref_dcgrid_dcswitch!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

"""
    build_acdcsw_DC_big_M(pm::AbstractPowerModel)

Build the DC busbar splitting optimization model with big-M formulation.

Constructs the complete mathematical formulation for DC busbar splitting including:
- Standard AC grid variables and constraints (fixed topology)
- DC switch indicator variables (binary)
- DC switch power flow variables
- Auxiliary bilinear variables for voltage magnitude relationships
- DC grid variables for power flows and voltages
- Big-M constraints linking switch status to power flows

# Model Components
1. **AC Grid**: Fixed topology with standard AC power flow
2. **DC Switches**: Binary variables controlling bus section connectivity
3. **DC Power Flows**: Variables for power through switchable DC elements
4. **Voltage Relationships**: Auxiliary variables for bilinear voltage terms
5. **Switching Logic**: Big-M constraints enforcing switch operation

# Variables Created
- Binary DC switch indicators
- DC switch power flow variables  
- Auxiliary voltage magnitude variables
- Standard DC grid power flow variables

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure with DC switching data
"""
function build_acdcsw_DC_big_M(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    # DC grid
    variable_dc_switch_indicator(pm) # binary variable to indicate the status of a dc switch
    variable_dc_switch_power(pm) # variable to indicate the power flowing through a dc switch (if closed)

    # Bilinear variables
    auxiliary_variable_DC_switch_voltage_magnitude(pm)
    auxiliary_diff_DC_switch_voltage_magnitude(pm)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # Objective function
    objective_min_fuel_cost_dc_switch(pm)


    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :dcswitch)
        constraint_dc_switch_thermal_limit(pm, i) # limiting the apparent power flowing through a dc switch
        constraint_dc_switch_power_on_off(pm,i)  # limiting the maximum active power through a dc switch
        constraint_dc_switch_voltage_on_off_big_M(pm, i)
    end

    for i in _PM.ids(pm, :dcswitch_couples)
        constraint_exclusivity_dc_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_dcbranch(pm, i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
        constraint_ZIL_dc_switch(pm,i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

   #for i in _PM.ids(pm, :dcline)
   #    _PM.constraint_dcline_power_losses(pm, i)
   #end

    for i in _PM.ids(pm, :busdc)
        constraint_power_balance_dc_switch(pm, i) # taking into account dc switches in the power balance of the dc part of an AC/DC grid
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



function run_acdcsw_DC_big_M_ZIL(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_DC_big_M_ZIL; ref_extensions=[add_ref_dcgrid_dcswitch!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_DC_big_M_ZIL(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    # DC grid
    variable_dc_switch_indicator(pm) # binary variable to indicate the status of a dc switch
    variable_dc_switch_power(pm) # variable to indicate the power flowing through a dc switch (if closed)

    # Bilinear variables
    auxiliary_variable_DC_switch_voltage_magnitude(pm)
    auxiliary_diff_DC_switch_voltage_magnitude(pm)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # Objective function
    objective_min_fuel_cost_dc_switch(pm)


    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :dcswitch)
        constraint_dc_switch_thermal_limit(pm, i) # limiting the apparent power flowing through a dc switch
        constraint_dc_switch_power_on_off(pm,i)  # limiting the maximum active power through a dc switch
        constraint_dc_switch_voltage_on_off_big_M(pm, i)
    end

    for i in _PM.ids(pm, :dcswitch_couples)
        constraint_exclusivity_dc_switch(pm, i) # the sum of the switches in a couple must be lower or equal than one (if OTS is allowed, like here), as each grid element is connected to either part of a split busbar no matter if the ZIL switch is opened or closed
        constraint_BS_OTS_dcbranch(pm, i) # making sure that if the grid element is not reconnected to the split busbar, the active and reactive power flowing through the switch is 0
        constraint_ZIL_dc_switch(pm,i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)
        _PM.constraint_voltage_angle_difference(pm, i)
        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

   #for i in _PM.ids(pm, :dcline)
   #    _PM.constraint_dcline_power_losses(pm, i)
   #end

    for i in _PM.ids(pm, :busdc)
        constraint_power_balance_dc_switch(pm, i) # taking into account dc switches in the power balance of the dc part of an AC/DC grid
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