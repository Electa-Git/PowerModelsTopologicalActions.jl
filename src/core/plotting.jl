"""
    plot_branch_flows_comparison(data, result_before, result_after; kwargs...)

Grouped bar chart of the active power flow through each AC branch before and after busbar
splitting. `data` is the original (unsplit) network; branch ids are preserved by
`AC_busbars_split`, so the flows of the two results are matched by branch id.
Flows are taken at the `f_bus` side and shown in MW, so a sign change means the flow reversed.

Both results must contain branch flows, i.e. be solved with
`setting = Dict("output" => Dict("branch_flows" => true))`.

Keyword arguments:
- `labels` — legend entries of the two series
- `title` — plot title
- any other keyword is forwarded to `StatsPlots.groupedbar`
"""
function plot_branch_flows_comparison(data, result_before, result_after;
        labels = ["Before splitting", "After splitting"],
        title = "Active power flow per branch",
        kwargs...)
    for (name, result) in (("result_before", result_before), ("result_after", result_after))
        if !haskey(result["solution"], "branch") || !haskey(first(values(result["solution"]["branch"])), "pf")
            Memento.error(_LOGGER, "$name has no branch flows: solve it with setting = Dict(\"output\" => Dict(\"branch_flows\" => true))")
        end
    end

    base_mva = data["baseMVA"]
    branch_ids = sort(parse.(Int, collect(keys(data["branch"]))))
    branch_names = ["$(id): $(data["branch"]["$id"]["f_bus"])→$(data["branch"]["$id"]["t_bus"])" for id in branch_ids]

    flow(result, id) = haskey(result["solution"]["branch"], "$id") ? result["solution"]["branch"]["$id"]["pf"] * base_mva : NaN
    flows = hcat([flow(result_before, id) for id in branch_ids],
                 [flow(result_after,  id) for id in branch_ids])
    lo, hi = extrema(filter(!isnan, flows))
    pad = 0.1 * max(hi - lo, 1.0)

    return StatsPlots.groupedbar(branch_names, flows;
        label = permutedims(labels),
        color = ["#2a78d6" "#eb6834"],
        linecolor = :white, linewidth = 1, bar_width = 0.7,
        title = title, xlabel = "Branch (id: from→to bus)", ylabel = "Active power flow [MW]",
        legend = :outertop, legend_column = -1, foreground_color_legend = nothing,
        grid = :y, gridalpha = 0.2, framestyle = :zerolines,
        ylims = (min(lo, 0.0) - pad, max(hi, 0.0) + pad),
        xrotation = length(branch_ids) > 10 ? 90 : 0,
        kwargs...)
end
