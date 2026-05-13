using Plots

# Sector labels and shares (replace with your exact 2024 values if needed)
labels = ["Power & heat", "Transport", "Industry", "Buildings", "Other"]
values = [35, 23, 29, 10, 3]  # %

# Modern, colorblind-friendly palette (Tableau-inspired)
colors = [
    RGB(78/255, 121/255, 167/255),  # blue
    RGB(242/255, 142/255, 43/255),  # orange
    RGB(89/255, 161/255, 79/255),   # green
    RGB(225/255, 87/255, 89/255),   # red
    RGB(176/255, 122/255, 161/255)  # purple
]

pie(
    values,
    labels = labels,
    color = colors,
    legend = :right,
    aspect_ratio = 1,
    title = "Global energy-related CO₂ emissions by sector (2024)",
    linewidth = 1.5
)

# Create the donut hole
plot!(shape = :circle, fillcolor = :white, linecolor = :white, label = "")


using Plots
using Measures
using DataFrames
using StatsPlots  # loads Plots.jl recipes
# If you don't have these:
# import Pkg; Pkg.add.(["DataFrames","StatsPlots"])

# -----------------------------
# Data (min, median, max)
# -----------------------------
df = DataFrame(
    tech = [
        "Coal",
        "Gas – combined cycle",
        "Biomass – Dedicated",
        "Solar PV – Utility scale",
        "Solar PV – rooftop",
        "Geothermal",
        "Concentrated solar power",
        "Hydropower",
        "Wind Offshore",
        "Nuclear",
        "Wind Onshore",
    ],
    min = [740, 410, 130, 18, 26, 6.0, 8.8, 1.0, 8.0, 3.7, 7.0],
    med = [820, 490, 230, 48, 41, 38, 27, 24, 12, 12, 11],
    max = [910, 650, 420, 180, 60, 79, 63, 2200, 35, 110, 56],  # NOTE: "22001" is as you provided.
    lcoe = [0.073, 0.085, 0.087, 0.043, 0.043, 0.060, 0.092, 0.057, 0.079, 0.069, 0.034]
    )

# If that "22001" is a typo (Wikipedia commonly shows hydropower max ~2200), uncomment:
# df[df.tech .== "Hydropower", :max] .= 2200

# Sort to match typical ordering (high → low median); comment this out if you want the original order.
sort!(df, :med, rev=true)

# -----------------------------
# Plot: median bars + min/max whiskers
# -----------------------------
x = 1:nrow(df)
lower = df.med .- df.min
upper = df.max .- df.med
yerr = (lower, upper)  # asymmetric error bars around the median
lcoes = df.lcoe

default(
    legend = false,
    size = (1200, 650),
    framestyle = :box,
    grid = false,
    tickfontsize = 11,
    guidefontsize = 14,
)

x = 1:nrow(df)

figures_folder = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/PhD_thesis/Figures"
p_emission = bar(
    x,
    df.med;
    yerror = yerr,
    xticks = (x, df.tech),
    xrotation = 30,
    ylabel = "Life-cycle greenhouse gas emissions (kgCO₂-eq / MWh)",
    ylims = (0, 10^3),
    linewidth = 0,
    left_margin   = 15mm,
    bottom_margin = 13mm,
    top_margin = 13mm,
)

p_lcoe = bar(
    x,
    df.lcoe;
    xticks = (x, df.tech),
    xrotation = 30,
    ylabel = "Levelized cost of electricity (\$ / kWh)",
    ylims = (0, 0.2),
    linewidth = 0,
    left_margin   = 15mm,
    bottom_margin = 13mm,
    top_margin = 13mm,
)
savefig(p_emission, joinpath(figures_folder,"emissions.pdf"))
savefig(p_lcoe, joinpath(figures_folder,"lcoe.pdf"))
