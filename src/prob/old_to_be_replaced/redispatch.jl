function run_redispatch_scenarios(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings,n_hours,n_scenarios)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:n_hours
        for i in 1:n_scenarios
            if i == 1
                timestep = (hour - 1)*n_scenarios + i
                result_feasibility_checks["$hour"] = Dict{String,Any}()
                feasibility_check = deepcopy(grid["nw"]["$hour"])
                feasibility_check_input = deepcopy(grid["nw"]["$hour"])
                _SPMTA.prepare_AC_feasibility_check_stochastic(result_bs["solution"]["nw"]["$timestep"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)
                for (g_id,g) in feasibility_check["gen"]
                    g["pg_start"] = result_bs["solution"]["nw"]["$timestep"]["gen"][g_id]["pg"]
                    g["qg_start"] = result_bs["solution"]["nw"]["$timestep"]["gen"][g_id]["qg"]
                
                    if length(g["cost"]) > 0
                        g["redispatch_cost_up"] = g["cost"][1]
                        g["redispatch_cost_down"] = g["cost"][1]
                    else
                        g["redispatch_cost_up"] = 0.0
                        g["redispatch_cost_down"] = 0.0
                    end
                end    
                result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
            end
        end
    end
    return result_feasibility_checks
end

function run_hourly_redispatch_scenarios(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings,n_hours,n_scenarios)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:n_hours
        for i in 1:n_scenarios
            if i == 1
                timestep = (hour - 1)*n_scenarios + i
                result_feasibility_checks["$hour"] = Dict{String,Any}()
                feasibility_check = deepcopy(grid["nw"]["$hour"])
                feasibility_check_input = deepcopy(grid["nw"]["$hour"])
                prepare_AC_feasibility_check_AC_busbars(result_bs["$hour"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)
                for (g_id,g) in feasibility_check["gen"]
                    g["pg_start"] = result_bs["$hour"]["solution"]["nw"]["1"]["gen"][g_id]["pg"]
                    g["qg_start"] = result_bs["$hour"]["solution"]["nw"]["1"]["gen"][g_id]["qg"]
                
                    if length(g["cost"]) > 0
                        g["redispatch_cost_up"] = g["cost"][1]
                        g["redispatch_cost_down"] = g["cost"][1]
                    else
                        g["redispatch_cost_up"] = 0.0
                        g["redispatch_cost_down"] = 0.0
                    end
                end
                result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
            end
        end
    end
    return result_feasibility_checks
end

function run_hourly_redispatch(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:length(grid["nw"])
        result_feasibility_checks["$hour"] = Dict{String,Any}()
        feasibility_check = deepcopy(grid["nw"]["$hour"])
        feasibility_check_input = deepcopy(grid["nw"]["$hour"])
        _PMTP.prepare_AC_feasibility_check(result_bs["$hour"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)

        # Adding set points
        for (g_id,g) in feasibility_check["gen"]
            g["pg_start"] = result_bs["$hour"]["solution"]["gen"][g_id]["pg"]
            g["qg_start"] = result_bs["$hour"]["solution"]["gen"][g_id]["qg"]
            if length(g["cost"]) > 0
                g["redispatch_cost_up"] = g["cost"][1]
                g["redispatch_cost_down"] = g["cost"][1]
            else
                g["redispatch_cost_up"] = 0.0
                g["redispatch_cost_down"] = 0.0
            end
        end
        result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
    end
    return result_feasibility_checks
end

function run_hourly_redispatch_one_topology(grid, result_bs, model, optimizer,switches_couples_ac,extremes_ZILs_ac,test_case_opf,settings)
    result_feasibility_checks = Dict{String,Any}()
    for hour in 1:length(grid["nw"])
        result_feasibility_checks["$hour"] = Dict{String,Any}()
        feasibility_check = deepcopy(grid["nw"]["$hour"])
        feasibility_check_input = deepcopy(grid["nw"]["$hour"])
        _SPMTA.prepare_AC_feasibility_check_stochastic(result_bs["solution"]["nw"]["$hour"],feasibility_check_input,feasibility_check,switches_couples_ac,extremes_ZILs_ac,test_case_opf)

        for (g_id,g) in feasibility_check["gen"]
            g["pg_start"] = result_bs["solution"]["nw"]["$hour"]["gen"][g_id]["pg"]
            g["qg_start"] = result_bs["solution"]["nw"]["$hour"]["gen"][g_id]["qg"]

            if length(g["cost"]) > 0
                g["redispatch_cost_up"] = g["cost"][1]
                g["redispatch_cost_down"] = g["cost"][1]
            else
                g["redispatch_cost_up"] = 0.0
                g["redispatch_cost_down"] = 0.0
            end
        end

        result_feasibility_checks["$hour"] = _SPMTA.solve_acdc_full_redispatch_opf(feasibility_check,model,optimizer)
    end
    return result_feasibility_checks
end