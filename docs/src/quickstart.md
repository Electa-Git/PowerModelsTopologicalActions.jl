# Quick start

This page walks through one complete OTS run and one complete BuS run on the bundled 5-bus
hybrid AC/DC case. If you only read one page, read this one.

## Common preamble

```julia
using PowerModels;                   const _PM     = PowerModels
using PowerModelsACDC;               const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActions; const _PMTP   = PowerModelsTopologicalActions
using JuMP, Ipopt, Gurobi, Juniper

gurobi  = JuMP.optimizer_with_attributes(Gurobi.Optimizer, "MIPGap" => 1e-4)
ipopt   = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer,
              "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

# Parse and augment the network. `process_additional_data!` is required — it adds the
# DC-grid structures that everything downstream expects.
file = joinpath(@__DIR__, "data_sources", "case5_acdc.m")
data = _PM.parse_file(file)
_PMACDC.process_additional_data!(data)
```

## Baseline

Always solve the plain AC/DC OPF first. It is the reference against which any topological
action must be judged, and it tells you immediately whether the case itself is feasible.

```julia
result_opf = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)

result_opf["termination_status"]   # LOCALLY_SOLVED
result_opf["objective"]            # 194.139 $/h
```

## Optimal transmission switching

OTS needs no data preparation. Point it at the network and pick which side of the grid may
be switched.

```julia
# AC branches switchable
result_ots_ac    = _PMTP.run_acdcots_AC(data, ACPPowerModel, juniper; setting = s)

# DC branches and AC/DC converters switchable
result_ots_dc    = _PMTP.run_acdcots_DC(data, ACPPowerModel, juniper; setting = s)

# everything switchable, optimized jointly
result_ots_acdc  = _PMTP.run_acdcots_AC_DC(data, ACPPowerModel, juniper; setting = s)
```

Read the switching decisions out of the solution dictionary:

```julia
# which AC branches were de-energized?
opened_ac = [i for (i, br) in result_ots_ac["solution"]["branch"] if br["br_status"] < 0.1]

# which DC branches, and which converters?
opened_dc   = [i for (i, br) in result_ots_dc["solution"]["branchdc"] if br["br_status"]   < 0.1]
opened_conv = [i for (i, c)  in result_ots_dc["solution"]["convdc"]   if c["conv_status"] < 0.1]

# the saving
100 * (result_opf["objective"] - result_ots_ac["objective"]) / result_opf["objective"]  # ≈ 5 %
```

Statuses come back as floats from the solver, so compare against a tolerance rather than
testing `== 0`.

## Busbar splitting

BuS is a three-stage workflow: **prepare the data → solve → check AC feasibility**. The
preparation stage is not optional — the optimization functions expect a network that has
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

See [Data model](@ref) for exactly what the transformation produces.

To split several busbars, pass a vector:

```julia
data_bus, switch_couples, extremes_ZIL = _PMTP.AC_busbars_split(data, [2, 3, 4])
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
for (sw_id, sw) in result_bus_lpac["solution"]["switch"]
    sw["status"] < 0.1 && println("switch $sw_id is OPEN")
end
```

A busbar has actually been split when the **busbar coupler** — the ZIL switch, identified
by `data_bus["switch"][id]["ZIL"] == true` — is open. Auxiliary switches being open just
means an element chose the other half.

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
```

If `result_fc` comes back infeasible, the approximated topology is not physically
realizable and must be discarded. This is the whole point of the check.

!!! note "The third argument is modified in place"
    `prepare_AC_feasibility_check_*` mutates its third argument and returns nothing useful.
    Always pass a `deepcopy`. The function is also verbose by design — it prints every
    reconnection it makes, which is genuinely useful when a check fails unexpectedly.

## DC and combined busbar splitting

DC busbars work identically, through `DC_busbars_split` and `run_acdc_BuS_DC`:

```julia
data_bus_dc, dcswitch_couples, extremes_ZIL_dc = _PMTP.DC_busbars_split(data, 2)
result_dc = _PMTP.run_acdc_BuS_DC(data_bus_dc, LPACCPowerModel, gurobi)

data_fc_dc = deepcopy(data_bus_dc)
_PMTP.prepare_AC_feasibility_check_DC_busbars(
    result_dc, data_bus_dc, data_fc_dc, dcswitch_couples, extremes_ZIL_dc, data)
result_fc_dc = _PMACDC.solve_acdcopf(data_fc_dc, ACPPowerModel, ipopt; setting = s)
```

To split an AC busbar and a DC busbar in the same problem, chain the two preparation
functions:

```julia
data_both, sw_ac, ext_ac = _PMTP.AC_busbars_split(data, 2)
data_both, sw_dc, ext_dc = _PMTP.DC_busbars_split(data_both, 2)

result_both = _PMTP.run_acdc_BuS_AC_DC(data_both, LPACCPowerModel, gurobi)
```

!!! warning "AC first, then DC"
    This order is required. `AC_busbars_split` resets `data["dcswitch_couples"]`, so calling
    it second wipes the DC switch couples — and `run_acdc_BuS_AC_DC` will then silently skip
    every DC exclusivity, ZIL, and BuS-OTS constraint, solving a different and
    over-optimistic problem without any error. `DC_busbars_split` preserves an existing
    `switch_couples`, so the AC-then-DC direction is safe. See
    [Known issues and gotchas](@ref).

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

A priori metrics for shortlisting promising busbars — rather than enumerating them — are
identified in the paper as future work.

## What to expect

On `case5_acdc.m`, splitting all AC busbars:

| Model | Objective [\$/h] | Time [s] | AC-feasible? | Saving vs AC-OPF |
|---|---|---|:-:|---|
| AC-OPF (baseline) | 194.139 | 0.014 | — | — |
| AC-BuS big-M (MINLP) | 184.972 | 232.4 | ✅ | 5.24 % |
| SOC-BuS | 183.763 | 0.30 | ✅ | no split found |
| QC-BuS | 183.761 | 0.33 | ✅ | no split found |
| LPAC-BuS | 181.909 | 0.45 | ✅ | 4.01 % |

Two things are worth internalizing from this table. First, the convex relaxations produce
low objective values but do not find a beneficial split — their bound is loose enough that
the original topology already looks optimal. Second, LPAC gets within about one percentage
point of the exact MINLP's saving in roughly 1/500th of the time. That trade is why LPAC is
the recommended default.