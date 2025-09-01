
"""
    objective_min_fuel_cost_ac_switch(pm::AbstractPowerModel)

Minimize total system cost including generation costs and AC switching costs.

Sets up the optimization objective to minimize the sum of:
1. Fuel costs for all generators in the system
2. Costs associated with AC switch operations (busbar splitting)

This objective is typically used in AC busbar splitting problems where the goal
is to find the optimal trade-off between generation costs and switching costs.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure

# Objective Function
minimize: ∑(generation costs) + ∑(AC switch costs)

# Notes
Switch costs help prevent unnecessary switching operations and can represent:
- Physical switching operation costs
- Maintenance costs
- Risk/reliability penalties
"""
function objective_min_fuel_cost_ac_switch(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)

    JuMP.add_to_expression!(cost, calc_gen_cost(pm))
    #println("COST GEN: is $(cost)")
    JuMP.add_to_expression!(cost, calc_ac_switch_cost(pm))
    #println("COST SWITCH: is $(cost)")

    JuMP.@objective(pm.model, Min, cost)
end

"""
    objective_min_fuel_cost_dc_switch(pm::AbstractPowerModel)

Minimize total system cost including generation costs and DC switching costs.

Sets up the optimization objective for DC transmission switching and busbar 
splitting problems, balancing generation efficiency against switching costs.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure

# Objective Function
minimize: ∑(generation costs) + ∑(DC switch costs)

# Notes
DC switching costs can represent:
- Converter switching operation costs
- DC breaker operation costs
- System reconfiguration penalties
"""
function objective_min_fuel_cost_dc_switch(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)

    JuMP.add_to_expression!(cost, calc_gen_cost(pm))
    JuMP.add_to_expression!(cost, calc_dc_switch_cost(pm))

    JuMP.@objective(pm.model, Min, cost)
end

"""
    objective_min_fuel_cost_ac_dc_switch(pm::AbstractPowerModel)

Minimize total system cost including generation costs and both AC and DC switching costs.

Sets up the comprehensive optimization objective for combined AC/DC transmission
switching and busbar splitting problems, considering all cost components.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure

# Objective Function
minimize: ∑(generation costs) + ∑(AC switch costs) + ∑(DC switch costs)

# Notes
This is the most comprehensive objective function used when both AC and DC 
topological actions are allowed simultaneously. It balances:
- Economic generation dispatch
- AC network reconfiguration costs  
- DC network reconfiguration costs
"""
function objective_min_fuel_cost_ac_dc_switch(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)

    JuMP.add_to_expression!(cost, calc_gen_cost(pm))
    JuMP.add_to_expression!(cost, calc_ac_switch_cost(pm))
    JuMP.add_to_expression!(cost, calc_dc_switch_cost(pm))

    JuMP.@objective(pm.model, Min, cost)
end

"""
    calc_gen_cost(pm::AbstractPowerModel)

Calculate the total generation cost expression for all generators.

Computes the linear generation cost expression based on the cost coefficients
stored in the generator data. Uses the second-to-last cost coefficient as the
marginal cost per MW.

# Arguments
- `pm::AbstractPowerModel`: PowerModels data structure

# Returns
- `JuMP.AffExpr`: Linear expression representing total generation costs

# Cost Calculation
For each generator with cost vector [c₀, c₁, c₂, ...]:
- Uses coefficient c₁ (second-to-last) as the linear cost coefficient
- Cost contribution = c₁ * pg[g_id]

# Notes
Assumes linear cost functions. For quadratic costs, additional terms would
need to be added to the cost expression.
"""
function calc_gen_cost(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)
    for (g_id,g) in _PM.ref(pm, :gen)
        if length(g["cost"]) ≥ 2
            JuMP.add_to_expression!(cost, g["cost"][end-1], _PM.var(pm,:pg,g_id))
        end
    end
    return cost
end

function calc_ac_switch_cost(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)
    for (sw_id,sw) in _PM.ref(pm, :switch)
        if !haskey(sw,"auxiliary")
            JuMP.add_to_expression!(cost, sw["cost"], (1 - _PM.var(pm,:z_switch,sw_id)))
        end
    end
    return cost
end

function calc_dc_switch_cost(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)
    for (sw_id,sw) in _PM.ref(pm, :dcswitch)
        JuMP.add_to_expression!(cost, sw["cost"], (1 - _PM.var(pm,:z_dcswitch,sw_id)))
    end
    return cost
end