using PowerModels; const _PM = PowerModels
using JuMP
using Ipopt, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using HSL_jll

#######################################################################################
## Define solvers ##
#######################################################################################

gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"MIPGap" => 1e-4,"QCPDual" => 1)
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0,"linear_solver" => "ma97")
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case5_acdc.m"
test_case_67_acdc = "case67.m"
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_file_67_acdc = joinpath(@__DIR__,"data_sources",test_case_67_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_67_acdc = _PM.parse_file(data_file_67_acdc)
_PMACDC.process_additional_data!(data_67_acdc)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_67_ac = _PMACDC.run_acdcopf(data_67_acdc,ACPPowerModel,ipopt; setting = s_dual)
result_opf_67_lpac = _PMACDC.run_acdcopf(data_67_acdc,LPACCPowerModel,gurobi; setting = s_dual)

duals_67 = []
duals_branches_67 = []
for (b_id,b) in data_67_acdc["bus"]
    push!(duals_67,[b_id,result_opf_67_lpac["solution"]["bus"][b_id]["lam_kcl_r"]])
end
for (b_id,b) in data_67_acdc["branch"]
    push!(duals_branches_67,[b_id,b["f_bus"],b["t_bus"],abs(result_opf_67_lpac["solution"]["bus"]["$(b["f_bus"])"]["lam_kcl_r"]-result_opf_67_lpac["solution"]["bus"]["$(b["t_bus"])"]["lam_kcl_r"])])
end
sort(duals_branches_67, by = x -> x[4], rev = true)