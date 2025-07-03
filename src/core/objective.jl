
function objective_min_fuel_cost_ac_switch(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)

    JuMP.add_to_expression!(cost, calc_gen_cost(pm))
    #println("COST GEN: is $(cost)")
    JuMP.add_to_expression!(cost, calc_ac_switch_cost(pm))
    #println("COST SWITCH: is $(cost)")

    JuMP.@objective(pm.model, Min, cost)
end

function objective_min_fuel_cost_dc_switch(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)

    JuMP.add_to_expression!(cost, calc_gen_cost(pm))
    JuMP.add_to_expression!(cost, calc_dc_switch_cost(pm))

    JuMP.@objective(pm.model, Min, cost)
end

function objective_min_fuel_cost_ac_dc_switch(pm::_PM.AbstractPowerModel)
    cost = JuMP.AffExpr(0.0)

    JuMP.add_to_expression!(cost, calc_gen_cost(pm))
    JuMP.add_to_expression!(cost, calc_ac_switch_cost(pm))
    JuMP.add_to_expression!(cost, calc_dc_switch_cost(pm))

    JuMP.@objective(pm.model, Min, cost)
end


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