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
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer,"QCPDual" => 1, "MIPGap" => 1e-6)
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0,"linear_solver" => "ma97")
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
ipopt_basic = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
juniper_basic = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt_basic, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case5_acdc.m"
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s_dual)

# OTS
result_ots_ac = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_ots_dc = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_ots_ac_dc = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)



#######################################################################################
#######################################################################################
test_case_39_acdc = "case39_acdc.m"
data_file_39_acdc = joinpath(@__DIR__,"data_sources",test_case_39_acdc)

data_39_acdc = _PM.parse_file(data_file_39_acdc)
_PMACDC.process_additional_data!(data_39_acdc)

for (l_id,l) in data_39_acdc["load"]
    if l_id != "4"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*6
        l["qd"] = l["qd"]*6
    end
end

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_39_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_39_acdc,LPACCPowerModel,gurobi; setting = s_dual)

# OTS
result_ots_ac = _PMTP.run_acdcots_AC(data_39_acdc,ACPPowerModel,juniper_basic; setting = s)
result_ots_dc = _PMTP.run_acdcots_DC(data_39_acdc,ACPPowerModel,juniper; setting = s)
result_ots_ac_dc = _PMTP.run_acdcots_AC_DC(data_39_acdc,ACPPowerModel,juniper_basic; setting = s)

result_ots_dc["solution"]["convdc"]["1"]["conv_status"]



#######################################################################################
#######################################################################################
test_case_67_acdc = "case67.m"
data_file_67_acdc = joinpath(@__DIR__,"data_sources",test_case_67_acdc)

data_67_acdc = _PM.parse_file(data_file_67_acdc)
_PMACDC.process_additional_data!(data_67_acdc)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_67_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_67_acdc,LPACCPowerModel,gurobi; setting = s_dual)

# OTS
result_ots_ac = _PMTP.run_acdcots_AC(data_67_acdc,ACPPowerModel,juniper; setting = s)
result_ots_dc = _PMTP.run_acdcots_DC(data_67_acdc,ACPPowerModel,juniper; setting = s)
result_ots_ac_dc = _PMTP.run_acdcots_AC_DC(data_67_acdc,ACPPowerModel,juniper; setting = s)

[result_ots_ac["solution"]["branch"]["$i"]["br_status"] for i in 1:length(data_67_acdc["branch"]) if result_ots_ac["solution"]["branch"]["$i"]["br_status"] == 0.0]



#######################################################################################
#######################################################################################
test_case_588_acdc = "pglib_opf_case588_sdet_acdc.m"
data_file_588_acdc = joinpath(@__DIR__,"data_sources",test_case_588_acdc)

data_588_acdc = _PM.parse_file(data_file_588_acdc)
_PMACDC.process_additional_data!(data_588_acdc)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_588_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_588_acdc,LPACCPowerModel,gurobi; setting = s_dual)

# OTS
result_ots_ac = _PMTP.run_acdcots_AC(data_588_acdc,ACPPowerModel,juniper; setting = s)
result_ots_dc = _PMTP.run_acdcots_DC(data_588_acdc,ACPPowerModel,juniper; setting = s)
result_ots_ac_dc = _PMTP.run_acdcots_AC_DC(data_588_acdc,ACPPowerModel,juniper; setting = s)

[result_ots_ac["solution"]["branch"]["$i"]["br_status"] for i in 1:length(data_67_acdc["branch"]) if result_ots_ac["solution"]["branch"]["$i"]["br_status"] == 0.0]