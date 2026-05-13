using Plots, StatsPlots

costs = [457500.00 	 ,508000.00 	 ,643500.00 	 ,1197000.00 	 ,1723500.00 	 ,1823000.00 	 ,2135000.00]
revenues = [551250.00 	 ,735000.00 	 ,918750.00 	 ,1475000.00 	,1921250.00 	 ,1976250.00 	,2348750.00]
profit = [70312.50 	,170250.00 	 ,206437.50 	 ,208500.00 	 ,148312.50 	 ,114937.50 	 ,160312.50 ]
net_income_projection = [330312.50 	 ,505562.50 	,717000.00 	 ,890500.00 	 ,1053812.50 	 ,1186750.00 	 ,1368062.50]
cumulative_profit = cumsum(profit)
avg_valuation = [ 1341875.00 	 ,2150500.00 	 ,2619125.00 	 ,3751166.67 	 ,4099541.67 	 ,4123375.00 	 ,5126791.67 ]

#net_income_projection = [80000.00 	 ,330312.50 	 ,505562.50 	,717000.00 	 ,890500.00 	 ,1053812.50 	 ,1186750.00 	 ,1368062.50]


years = collect(2027:2033)
plot_revenues = Plots.plot(years, net_income_projection/10^6,xticks=years, xlabel="Year", ylabel="M€", label= "Cash flow", grid = :false,ylims = (-0.1,6))
bar!(plot_revenues,years, revenues/10^6,xticks=years, label= "Revenues", grid = :false, bar_width = 0.3, color = :green)
bar!(plot_revenues,years, costs/10^6,xticks=years, label= "Costs", grid = :false, bar_width = 0.3, color = :red)

data = [-costs revenues profit]

plot_revenues = StatsPlots.groupedbar(years, data./10^6, xticks=years, label=["Costs" "Revenues" "Profit"], grid = :false, bar_width = 0.3, color=[:red :green :blue], xlabel="Year", ylabel="Million €",ylims = (-3,6),yticks=-3:1:6)
Plots.plot!(years,cumulative_profit./10^6,xticks=years, xlabel="Year", label= "Cumulative profit",color=:lightgreen, linewidth=4)
Plots.plot!(years,avg_valuation./10^6,xticks=years, xlabel="Year", label= "Average Valuation")
Plots.hline!([0.0], label= :none, color=:black, linestyle=:dash)

results_folder = "/Users/giacomobastianel/Desktop"
savefig(plot_revenues, joinpath(results_folder, "cash_flow_projection.png"))
savefig(plot_revenues, joinpath(results_folder, "cash_flow_projection.svg"))
savefig(plot_revenues, joinpath(results_folder, "cash_flow_projection.pdf"))