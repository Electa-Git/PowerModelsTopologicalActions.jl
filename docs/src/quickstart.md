# Quick start

This page walks through one complete BuS run on the bundled 5-bus
hybrid AC/DC case. The thorough explanation of the results of this test case can be found in the [![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.segan.2026.102182-blue)](https://doi.org/10.1016/j.segan.2026.102182) paper.


## Common preamble

```julia
using PowerModels;                   const _PM     = PowerModels
using PowerModelsACDC;               const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActions; const _PMTP   = PowerModelsTopologicalActions
using JuMP, Ipopt, Gurobi, Juniper

gurobi  = JuMP.optimizer_with_attributes(Gurobi.Optimizer, "MIPGap" => 1e-4)
ipopt   = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

# Parse and augment the network. `process_additional_data!` is required — it adds the
# DC-grid structures that everything downstream expects.
file = joinpath(dirname(dirname(@__DIR__)),"test","data_sources", "case5_acdc.m")
data = _PM.parse_file(file)
_PMACDC.process_additional_data!(data)
```

## Baseline

Always solve the plain AC/DC OPF first. It is the reference against which any topological
action must be judged, and it tells you immediately whether the case itself is feasible.

```julia
result_opf = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)

result_opf["termination_status"]   # LOCALLY_SOLVED
result_opf["primal_status"]        # FEASIBLE_POINT
result_opf["objective"]            # 194.139 $/h
```

## Busbar splitting

BuS is a three-stage workflow: **prepare the data → solve → check AC feasibility**. The
preparation stage is needed as the topology optimization model with BuS expect a network that has
already been expanded with auxiliary buses and switches.

### Stage 1: prepare the data

```julia
bus_to_split = 2

data_bus, switch_couples, extremes_ZIL = _PMTP.AC_busbars_split(data, bus_to_split)
```

This returns three things:

- `data_bus` — a **new** network dictionary (the input is not mutated) in which busbar 2
  has been duplicated, every element formerly attached to it has been moved onto its own
  auxiliary bus, and switches have been inserted to connect them to either half.
- `switch_couples` — the pairs of switches whose exclusivity constraint decides which half
  each element ends up on. You need this again in stage 3.
- `extremes_ZIL` — a map from each original busbar to the two bus indices its parts now
  occupy. Also needed in stage 3.

See [Data model](data_model.md) for exactly what the transformation produces.

To split several busbars, pass a vector:

```julia
buses_to_split = [1, 2, 3, 4, 5]
data_multiple_buses, switch_couples_multiple_buses, extremes_ZIL_multiple_buses = _PMTP.AC_busbars_split(data, buses_to_split)
```

### Stage 2: solve

```julia
# exact MINLP
result_bus_ac   = _PMTP.run_acdc_BuS_AC(data_bus, ACPPowerModel,  juniper)

# LPAC approximation — far faster, and what you will use in practice
result_bus_lpac = _PMTP.run_acdc_BuS_AC(data_bus, LPACCPowerModel, gurobi)
```

The switching decisions are on the switches:
```julia
for sw_id in 1:length(data_bus["switch"])
    if !haskey(data_bus["switch"]["$sw_id"], "auxiliary") 
        println("switch $sw_id is a busbar coupler with status $(result_bus_lpac["solution"]["switch"]["$sw_id"]["status"])")
    else
        if result_bus_lpac["solution"]["switch"]["$sw_id"]["status"] > 0.9
            if data_bus["switch"]["$sw_id"]["t_bus"] == extremes_ZIL["$bus_to_split"][1]
                println("switch $sw_id, linked to element $(data_bus["switch"]["$sw_id"]["auxiliary"]) $(data_bus["switch"]["$sw_id"]["original"]) is connected to the original busbar $bus_to_split")
            else
                println("switch $sw_id, linked to element $(data_bus["switch"]["$sw_id"]["auxiliary"]) $(data_bus["switch"]["$sw_id"]["original"]) is connected to the second half of busbar $bus_to_split, which is $bus_to_split'")
            end
        end
    end
end
```

A busbar has actually been split when the **busbar coupler** — the ZIL switch 1, is open. The auxiliary switches have been used to connect each element to either half of the busbar.

### Stage 3: check AC feasibility

The LPAC result is an approximation, so its objective value is not directly comparable to
an AC-OPF objective and its topology carries no formal feasibility guarantee. To settle
both questions, freeze the optimized topology into a fixed network and solve a plain AC/DC
OPF on it.

```julia
data_fc = deepcopy(data_bus)

_PMTP.prepare_AC_feasibility_check_AC_busbars(
    result_bus_lpac,   # result whose topology you want to test
    data_bus,          # the split network the result came from
    data_fc,           # MUTATED IN PLACE — becomes the fixed-topology network
    switch_couples,    # from stage 1
    extremes_ZIL,      # from stage 1
    data,              # the original, unsplit network
)

result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)
```

Now the comparison is apples to apples:

```julia
result_fc["termination_status"]      # LOCALLY_SOLVED → the topology is AC-feasible
result_fc["objective"]               # 186.349 $/h
result_opf["objective"]              # 194.139 $/h → a genuine 4.0 % saving
println("Saving of (LPAC_BuS + AC-FC) vs AC-OPF: $(100*(result_opf["objective"] - result_fc["objective"])/result_opf["objective"]) %")
```

If `result_fc` comes back infeasible, the approximated topology is not physically
realizable and must be discarded. This is the whole point of the check, in my experience it barely happens.

!!! note "The third argument is modified in place"
    `prepare_AC_feasibility_check_*` mutates its third argument and returns nothing useful.
    Always pass a `deepcopy`. The function is also verbose by design for now — it prints every
    reconnection it makes, which is genuinely useful when a check fails unexpectedly.



## Choosing which busbar to split

Splitting every busbar is combinatorially expensive, so the practical approach from the
paper is a screening pass: run the cheap LPAC-BuS model once per candidate busbar, check
each resulting topology for AC feasibility, and keep the winner.

```julia
best = (bus = nothing, obj = result_opf["objective"])

for b in keys(data["bus"])
    bus_id = parse(Int, b)
    data_b, sw, ext = _PMTP.AC_busbars_split(data, bus_id)
    res = _PMTP.run_acdc_BuS_AC(data_b, LPACCPowerModel, gurobi)

    data_fc = deepcopy(data_b)
    _PMTP.prepare_AC_feasibility_check_AC_busbars(res, data_b, data_fc, sw, ext, data)
    fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)

    if fc["termination_status"] == LOCALLY_SOLVED && fc["objective"] < best.obj
        best = (bus = bus_id, obj = fc["objective"])
    end
end

best
```

A priori metrics for shortlisting promising busbars — rather than enumerating them - have been developed in this paper: [![DOI](https://img.shields.io/badge/DOI-10.1016/j.epsr.2026.113611-blue)](https://www.sciencedirect.com/science/article/pii/S0378779626009041).

## What to expect

On `case5_acdc.m`, splitting AC busbar 2:

| Model | Objective [\$/h] | Time [s] | AC-feasible? | Saving vs AC-OPF |
|---|---|---|:-:|---|
| AC-OPF (baseline) | 194.139 | 0.014 | — | — |
| AC-BuS big-M (MINLP) | 185.209 | 12.81 | ✅ | 4.60 % |
| LPAC-BuS + AC-FC | 186.349 | 0.09 | ✅ | 4.01 % |

