using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using Mosek, MosekTools
using Plots
using StatsPlots


#######################################################################################
## Define solver ##
#######################################################################################

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => highs, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case5_acdc.m"
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_original_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_original_5_acdc)


#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)


##############
# Showing the utilization of each branch, to be intended as absolute values
for (br_id, br) in result_opf_ac["solution"]["branch"]
    println("AC Branch $(br_id) with f_bus $(data_5_acdc["branch"][br_id]["f_bus"]) and t_bus $(data_5_acdc["branch"][br_id]["t_bus"])")
    println("Utilization AC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",br["pf"])
    #print("Utilization AC branch $(br_id) AC/DC OTS - AC $(result_ots_ac["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ", result_ots_ac["solution"]["branch"][br_id]["pf"],"\n")
end


for (br_id, br) in result_opf_ac["solution"]["branchdc"]
    println("DC Branch $(br_id) with f_bus $(data_5_acdc["branchdc"][br_id]["fbusdc"]) and t_bus $(data_5_acdc["branchdc"][br_id]["tbusdc"])")
    #println("Utilization DC branch $(br_id) AC/DC OTS - AC $(result_ots_ac["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ", result_ots_ac["solution"]["branchdc"][br_id]["pf"])
    print("\n")
end


#######################################################################################
## Busbar splitting models ##
#######################################################################################

###### AC Busbar splitting models ######
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = [1,2,3,4,5]

split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)
data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbars_split(data_busbars_ac_split_5_acdc,splitted_bus_ac)

ac_bs_ac = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac = deepcopy(data_busbars_ac_split_5_acdc)

ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_ac  = _PMTP.run_acdcsw_AC(ac_bs_ac,ACPPowerModel,juniper)
result_switches_AC_lpac  = _PMTP.run_acdcsw_AC(ac_bs_lpac,LPACCPowerModel,gurobi)

# Feasibility check
test_case = deepcopy(data_busbars_ac_split_5_acdc)       
test_case_updated_split = deepcopy(test_case)
test_case_auxiliary_split = deepcopy(test_case)
_PMTP.prepare_AC_feasibility_check_AC_busbars(result_switches_AC_ac,test_case_auxiliary_split,test_case_updated_split,switches_couples_ac_5,extremes_ZILs_5_ac,data_5_acdc)

result_opf_fc = _PMACDC.run_acdcopf(test_case_updated_split,ACPPowerModel,ipopt; setting = s)

for (br_id,br) in test_case_updated_split["branch"]
    println("Branch $(br_id) with f_bus $(test_case_updated_split["branch"][br_id]["f_bus"]) and t_bus $(test_case_updated_split["branch"][br_id]["t_bus"])")
end


for (br_id,br) in data_5_acdc["branch"]
    println("Branch $(br_id) with f_bus $(data_5_acdc["branch"][br_id]["f_bus"]) and t_bus $(data_5_acdc["branch"][br_id]["t_bus"])")
    println("OPF p_fr is $(result_opf_ac["solution"]["branch"]["$br_id"]["pf"]) and p_ft is $(result_opf_ac["solution"]["branch"]["$br_id"]["pt"])")
    println("BuS - FC p_fr is $(result_opf_fc["solution"]["branch"]["$br_id"]["pf"]) and p_ft is $(result_opf_fc["solution"]["branch"]["$br_id"]["pt"])\n")
    println("---")
end

for (br_id,br) in data_5_acdc["bus"]
    println("Bus $(br_id)")
    println("OPF va is $(result_opf_ac["solution"]["bus"]["$br_id"]["va"]), and p_ft is $(result_opf_ac["solution"]["bus"]["$br_id"]["vm"])")
    println("BuS - FC va is $(result_opf_fc["solution"]["bus"]["$br_id"]["va"]) and vm is $(result_opf_fc["solution"]["bus"]["$br_id"]["vm"])\n")
    println("---")
end

for (br_id,br) in data_5_acdc["busdc"]
    println("Bus $(br_id)")
    println("OPF vm is $(result_opf_ac["solution"]["busdc"]["$br_id"]["vm"])")
    println("BuS - FC vm is $(result_opf_fc["solution"]["busdc"]["$br_id"]["vm"])\n")
    println("---")
end

for (br_id,br) in test_case_updated_split["branchdc"]

    println("Branch DC $(br_id) with f_bus $(test_case_updated_split["branchdc"][br_id]["fbusdc"]) and t_bus $(test_case_updated_split["branchdc"][br_id]["tbusdc"])")
    println("OPF p_f is $(result_opf_ac["solution"]["branchdc"]["$br_id"]["pf"]) and p_t is $(result_opf_ac["solution"]["branchdc"]["$br_id"]["pt"])")
    println("BuS - FC p_f is $(result_opf_fc["solution"]["branchdc"]["$br_id"]["pf"]) and p_t is $(result_opf_fc["solution"]["branchdc"]["$br_id"]["pt"])\n")
    println("---")
end

println("BuS - FC p_fr is $(result_opf_fc["solution"]["bus"]["10"]["va"]) and p_ft is $(result_opf_fc["solution"]["bus"]["10"]["vm"])\n")


figures_folder = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/PSCC_2024"
########################

br_bus = [abs(result_opf_fc["solution"]["branch"]["$br_id"]["pf"]) for br_id in 1:length(test_case["branch"])]
br_bus_opf = [abs(result_opf_ac["solution"]["branch"]["$br_id"]["pf"]) for br_id in 1:length(test_case["branch"])]
br_utilization = vcat(br_bus_opf, br_bus)
sx = repeat(["OPF", "BuS - FC"], inner = length(test_case["branch"]))
name_try = collect(1:length(test_case["branch"]))
name = vcat(name_try, name_try)

p_br = groupedbar(name, br_utilization*100, group = sx, ylabel = "Branch utilization [%]", xlabel = "Branch ID", xticks = 1:1:length(test_case["branch"]), yticks = 0:10:100, ylims = (0,101),
title = "", bar_width = 0.7, color = [:grey40 :grey70], grid = :none)

Plots.savefig(p_br,joinpath(figures_folder,"branch_utilization_all_busbars.pdf"))
Plots.savefig(p_br,joinpath(figures_folder,"branch_utilization_all_busbars.svg"))


#####

br_bus_dc = [abs(result_opf_fc["solution"]["branchdc"]["$br_id"]["pf"]) for br_id in 1:length(test_case["branchdc"])]
br_bus_opf_dc = [abs(result_opf_ac["solution"]["branchdc"]["$br_id"]["pf"]) for br_id in 1:length(test_case["branchdc"])]
br_utilization_dc = vcat(br_bus_opf_dc, br_bus_dc)
sx = repeat(["OPF", "BuS - FC"], inner = length(test_case["branchdc"]))
name_try = collect(1:length(test_case["branchdc"]))
name = vcat(name_try, name_try)

p_br_dc_ = groupedbar(name, br_utilization_dc*100, group = sx, ylabel = "Branch utilization [%]", xlabel = "DC Branch ID", xticks = 1:1:length(test_case["branch"]), yticks = 0:10:100, ylims = (0,101),
title = "", bar_width = 0.7, color = [:grey40 :grey70], grid = :none)

Plots.savefig(p_br_dc,joinpath(figures_folder,"branch_dc_utilization_all_busbars.pdf"))
Plots.savefig(p_br_dc,joinpath(figures_folder,"branch_dc_utilization_all_busbars.svg"))


######

diff_va_bus = [result_opf_fc["solution"]["bus"]["$(test_case_updated_split["branch"]["$br_id"]["f_bus"])"]["va"] - result_opf_fc["solution"]["bus"]["$(test_case_updated_split["branch"]["$br_id"]["t_bus"])"]["va"]) for br_id in 1:length(test_case["branch"])]
diff_va_bus_opf = [(result_opf_ac["solution"]["bus"]["$(data_5_acdc["branch"]["$br_id"]["f_bus"])"]["va"] - result_opf_ac["solution"]["bus"]["$(data_5_acdc["branch"]["$br_id"]["t_bus"])"]["va"]) for br_id in 1:length(test_case["branch"])]

diff_va_utilization = vcat(diff_va_bus, diff_va_bus_opf)
sx = repeat(["BuS - FC","OPF"], inner = length(data_5_acdc["branch"]))
name_try = collect(1:length(data_5_acdc["branch"]))
name = vcat(name_try, name_try)

va_diff = groupedbar(name, diff_va_utilization, group = sx, ylabel = "Voltage angle difference over a branch", xlabel = "Branch ID", xticks = 1:1:length(test_case["branch"]), yticks = 0:0.1:pi/8, ylims = (0,pi/8),
title = "", bar_width = 0.7, color = [:grey40 :grey70], grid = :none)

Plots.savefig(va_diff,joinpath(figures_folder,"va_diff_branch.pdf"))
Plots.savefig(va_diff,joinpath(figures_folder,"va_diff_branch.svg"))


#######
va_bus = [result_opf_fc["solution"]["bus"]["$b_id"]["va"] for b_id in 1:(length(data_5_acdc["bus"]))]
push!(va_bus,result_opf_fc["solution"]["bus"]["10"]["va"])
va_bus_opf = [result_opf_ac["solution"]["bus"]["$b_id"]["va"] for b_id in 1:length(data_5_acdc["bus"])]
push!(va_bus_opf,0.0) 

vm_bus = [abs(result_opf_fc["solution"]["bus"]["$b_id"]["vm"]) for b_id in 1:(length(data_5_acdc["bus"]))]
push!(vm_bus,result_opf_fc["solution"]["bus"]["10"]["vm"])
vm_bus_opf = [abs(result_opf_ac["solution"]["bus"]["$b_id"]["vm"]) for b_id in 1:length(data_5_acdc["bus"])]
push!(vm_bus_opf,0.0) 


va_bus_comparison = vcat(va_bus_opf,va_bus)
sx = repeat(["OPF", "BuS - FC"], inner = length(va_bus))
name_try = collect(1:length(va_bus))
name = vcat(name_try, name_try)

bus_names = []
for i in 1:length(data_5_acdc["bus"])
    push!(bus_names,"$i")
end
push!(bus_names, "3'")

p_va = groupedbar(name, va_bus_comparison, group = sx, ylabel = "Voltage angle [rad]", xlabel = "Bus ID", xticks = (1:1:length(va_bus),bus_names), ylims = (-pi/10,pi/10),
title = "", bar_width = 0.7, color = [:grey40 :grey70], grid = :none)

Plots.savefig(p_va,joinpath(figures_folder,"va_buses.pdf"))
Plots.savefig(p_va,joinpath(figures_folder,"va_buses.svg"))


vm_bus_comparison = vcat(vm_bus_opf,vm_bus)
sx = repeat(["OPF", "BuS - FC"], inner = length(vm_bus))
name_try = collect(1:length(vm_bus))
name = vcat(name_try, name_try)
p_vm = groupedbar(name, vm_bus_comparison, group = sx, ylabel = "Voltage magnitude [pu]", xlabel = "Bus ID", xticks = (1:1:length(vm_bus),bus_names), ylims = (0.8,1.15),
title = "", bar_width = 0.7, color = [:grey40 :grey70], grid = :none)

Plots.savefig(p_vm,joinpath(figures_folder,"vm_buses.pdf"))
Plots.savefig(p_vm,joinpath(figures_folder,"vm_buses.svg"))


#######
pg_bus = [abs(result_opf_fc["solution"]["gen"]["$b_id"]["pg"]) for b_id in 1:(length(data_5_acdc["gen"]))]
pg_bus_opf = [abs(result_opf_ac["solution"]["gen"]["$b_id"]["pg"]) for b_id in 1:length(data_5_acdc["gen"])]
pg_bus_comparison = vcat(pg_bus_opf,pg_bus)
pg_max = [data_5_acdc["gen"]["$b_id"]["pmax"] for b_id in 1:length(data_5_acdc["gen"])]
pg_perc = pg_bus_comparison ./ vcat(pg_max,pg_max) * 100

sx = repeat(["OPF", "BuS - FC"], inner = length(pg_bus))
name_try = collect(1:length(pg_bus))
name = vcat(name_try, name_try)
p_g = groupedbar(name, pg_perc, group = sx, ylabel = "Generation [%]", xlabel = "Generator ID", xticks = 1:1:length(pg_bus), ylims = [0,101], yticks = 0:10:100,
title = "", bar_width = 0.7, color = [:grey40 :grey70], grid = :none)

Plots.savefig(p_g,joinpath(figures_folder,"p_g.pdf"))
Plots.savefig(p_g,joinpath(figures_folder,"p_g.svg"))



######################

###### DC Busbar splitting models ######
# DC BS for AC/DC grid with DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_dc = 1

data_busbars_dc_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)

dc_bs_ac = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_lpac = deepcopy(data_busbars_dc_split_5_acdc)

dc_bs_ac_ref = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_lpac_ref = deepcopy(data_busbars_dc_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_DC_ac  = _PMTP.run_acdcsw_DC(dc_bs_ac,ACPPowerModel,juniper)
result_switches_DC_lpac  = _PMTP.run_acdcsw_DC(dc_bs_lpac,LPACCPowerModel,juniper)

result_switches_DC_ac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_ac_ref,ACPPowerModel,juniper)
result_switches_DC_lpac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_lpac_ref,LPACCPowerModel,gurobi)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_DC_BS_opf_lpac = deepcopy(data_busbars_dc_split_5_acdc)
feasibility_check_DC_BS_opf_lpac_ref = deepcopy(data_busbars_dc_split_5_acdc)

for (br_id, br) in result_switches_DC_ac["solution"]["dcswitch"]
    feasibility_check_DC_BS_opf_lpac["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_lpac["solution"]["dcswitch"][br_id]["status"])
end
for (br_id, br) in result_switches_DC_ac["solution"]["dcswitch"]
    feasibility_check_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end
feasibility_check_opf_lpac = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)



###### AC/DC Busbar splitting models ######
# AC/DC BS for AC/DC grid with AC and DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_dc_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2
splitted_bus_dc = collect(1:3)

data_busbars_ac_dc_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_dc)

ac_dc_bs_ac = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_lpac = deepcopy(data_busbars_ac_dc_split_5_acdc)

ac_dc_bs_ac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_lpac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_DC_ac  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_ac,ACPPowerModel,juniper)
result_switches_AC_DC_lpac  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_lpac,LPACCPowerModel,juniper)

result_switches_AC_DC_ac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_DC_lpac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_lpac_ref,LPACCPowerModel,gurobi)


print_connections_DC_switches(result_switches_AC_DC_ac_ref,ac_dc_bs_ac_ref)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_AC_DC_BS_opf_lpac = deepcopy(data_busbars_ac_dc_split_5_acdc)
feasibility_check_AC_DC_BS_opf_lpac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)

for (br_id, br) in result_switches_AC_DC_ac["solution"]["switch"]
    feasibility_check_AC_DC_BS_opf_lpac["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac["solution"]["switch"][br_id]["status"])
end
for (br_id, br) in result_switches_AC_DC_soc_ref["solution"]["switch"]
    feasibility_check_AC_DC_BS_opf_lpac_ref["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac_ref["solution"]["switch"][br_id]["status"])
end

for (br_id, br) in result_switches_AC_DC_soc_ref["solution"]["dcswitch"]
    feasibility_check_AC_DC_BS_opf_soc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_soc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_qc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_qc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end

feasibility_check_opf_lpac = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)

