# File to treat Elia's data
using PowerModels; const _PM = PowerModels
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActionsII; const _PMTP = PowerModelsTopologicalActionsII
using Gurobi
using JuMP
using DataFrames
using CSV
using Plots
using Feather
using JSON
using Ipopt
using Juniper


gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

##################################################################
## Processing input data
folder_data = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Elia_data"

# Belgium grid without energy island
OFW_2023_file = joinpath(folder_data,"Offshore_wind_2023.json")
OFW_2023 = JSON.parsefile(OFW_2023_file)

offshore_timesteps = []
for i in 1:length(OFW_2023)
    if OFW_2023[i]["offshoreonshore"] == "Offshore"
        push!(offshore_timesteps,OFW_2023[i])
    end
end
offshore_timesteps_ordered = reverse(offshore_timesteps)


offshore_timesteps_ordered_measured = []
offshore_timesteps_ordered_hourly = []
offshore_timesteps_ordered_hourly_90 = []
offshore_timesteps_ordered_hourly_10 = []
monitored_capacity = []
for i in 1:35040
    push!(offshore_timesteps_ordered_hourly,offshore_timesteps_ordered[i]["dayahead11hforecast"])
    push!(offshore_timesteps_ordered_hourly_90,offshore_timesteps_ordered[i]["dayahead11hconfidence90"])
    push!(offshore_timesteps_ordered_hourly_10,offshore_timesteps_ordered[i]["dayahead11hconfidence10"])
    push!(offshore_timesteps_ordered_measured,offshore_timesteps_ordered[i]["measured"])
    push!(monitored_capacity,offshore_timesteps_ordered[i]["monitoredcapacity"])
end
#plot(offshore_timesteps_ordered_hourly, label = "Forecast",grid = :none)
#plot!(offshore_timesteps_ordered_hourly_90, label = "P90")
#plot!(offshore_timesteps_ordered_hourly_10, label = "P10")
#plot!(offshore_timesteps_ordered_measured, label = "Measured")



using Distributions

# 10th percentile value
# 90th percentile value
# u mean value
# N normal distribution
#mean_estimate = (P_10 + P_90) / 2
#std_estimate = (P_90 - P_10) / (2 * quantile(Normal(), 0.9))

function compute_pdfs(N_dict,P_90_dict,P_10_dict,n_samples,number_of_timesteps,Dict_values)
    for i in 1:number_of_timesteps
        P_90_timestep = P_90_dict[i]
        P_10_timestep = P_10_dict[i]
        N_timestep = N_dict[i]
        z_010 = -1.2816
        z_090 = 1.2816
        sigma_timestep = deepcopy((P_90_timestep - P_10_timestep)/(2*z_090))
        d_timestep = deepcopy(Normal(N_timestep,sigma_timestep))
        td = deepcopy(truncated(d_timestep, 0.0, Inf))
        samples_timestep = deepcopy(rand(td, n_samples))
        x_timestep = deepcopy(range(N_timestep - 3*sigma_timestep, N_timestep + 3*sigma_timestep, length=n_samples))
        y_timestep = deepcopy(pdf.(d_timestep, x_timestep))
        Dict_values["$(i)"] = Dict{String,Any}()
        Dict_values["$(i)"]["samples"] = deepcopy(samples_timestep)
        Dict_values["$(i)"]["x"] = deepcopy(x_timestep)
        Dict_values["$(i)"]["pdf_original"] = deepcopy(y_timestep)
        Dict_values["$(i)"]["mean"] = deepcopy(N_timestep)
        Dict_values["$(i)"]["std"] = deepcopy(sigma_timestep)
        Dict_values["$(i)"]["datetime"] = deepcopy(offshore_timesteps_ordered[i]["datetime"])
        Dict_values["$(i)"]["pdf"] = deepcopy(Dict_values["$(i)"]["pdf_original"]/sum(Dict_values["$(i)"]["pdf_original"]))
    end
end
Gaussian_samples = Dict{String,Any}()
n_timesteps = 10
compute_pdfs(offshore_timesteps_ordered_hourly,offshore_timesteps_ordered_hourly_90,offshore_timesteps_ordered_hourly_10,n_timesteps,1,Gaussian_samples)

#random_indices = rand(1:2000, 5)
random_indices = rand(1:1, 1)

p1 = plot()
#rand_timestep = rand(Int,10)
for i in random_indices
    scatter!(p1,Gaussian_samples["$(i)"]["x"],Gaussian_samples["$(i)"]["pdf"],xlabel = "Generation [MW]", ylabel = "Probability [-]",legend =:none)
end
display(p1)

Gaussian_samples["1"]["x"]/
Gaussian_samples["1"]["pdf"]
Gaussian_samples["1"]["samples"]

function compute_pdfs(N_dict, P_90_dict, P_10_dict, n_samples, number_of_timesteps, Dict_values, x)
    for i in 1:number_of_timesteps
        P_90_timestep = P_90_dict[i]
        P_10_timestep = P_10_dict[i]
        N_timestep = N_dict[i]

        # Cap P_90, P_10, and N_timestep to ensure they don't exceed the maximum value x
        P_90_timestep = min(P_90_timestep, x)
        P_10_timestep = min(P_10_timestep, x)
        N_timestep = min(N_timestep, x)

        z_010 = -1.2816
        z_090 = 1.2816
        sigma_timestep = deepcopy((P_90_timestep - P_10_timestep) / (2 * z_090))
        d_timestep = deepcopy(Normal(N_timestep, sigma_timestep))
        
        # Ensure the distribution is truncated to [0, x] to respect wind farm constraints
        td = deepcopy(truncated(d_timestep, 0.0, x))
        
        # Generate samples and compute PDF values
        samples_timestep = deepcopy(rand(td, n_samples))
        x_timestep = deepcopy(range(N_timestep - 3 * sigma_timestep, N_timestep + 3 * sigma_timestep, length=n_samples))
        
        # Cap x_timestep to the range [0, x]
        x_timestep = clamp.(x_timestep, 0.0, x)
        y_timestep = deepcopy(pdf.(td, x_timestep))

        # Store results in Dict_values
        Dict_values["$(i)"] = Dict{String, Any}()
        Dict_values["$(i)"]["samples"] = deepcopy(samples_timestep)
        Dict_values["$(i)"]["x"] = deepcopy(x_timestep)
        Dict_values["$(i)"]["pdf_original"] = deepcopy(y_timestep)
        Dict_values["$(i)"]["mean"] = deepcopy(N_timestep)
        Dict_values["$(i)"]["std"] = deepcopy(sigma_timestep)
        Dict_values["$(i)"]["datetime"] = deepcopy(offshore_timesteps_ordered[i]["datetime"])
        Dict_values["$(i)"]["pdf"] = deepcopy(Dict_values["$(i)"]["pdf_original"] / sum(Dict_values["$(i)"]["pdf_original"]))
    end
end
Gaussian_samples = Dict{String,Any}()
n_timesteps = 100
compute_pdfs(offshore_timesteps_ordered_hourly,offshore_timesteps_ordered_hourly_90,offshore_timesteps_ordered_hourly_10,n_timesteps,10,Gaussian_samples,monitored_capacity[1])

#random_indices = rand(1:2000, 5)
random_indices = rand(1:10, 15)

p1 = plot()
#rand_timestep = rand(Int,10)
for i in random_indices
    scatter!(p1,Gaussian_samples["$(i)"]["x"],Gaussian_samples["$(i)"]["pdf"],xlabel = "Generation [MW]", ylabel = "Probability [-]",legend =:none)
end
display(p1)

################################################################################
# Process load data
Load_2024_file = joinpath(folder_data,"Elia_load_2024.json")
Load_2024 = JSON.parsefile(Load_2024_file)
Load_2024[1]