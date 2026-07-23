# AC feasibility check

## Why it exists

A relaxation gives you a lower bound on cost but a solution that may not correspond to any
physically realizable operating point. An approximation gives you a solution fast but with
no guarantee at all. In both cases the *topology* the model recommends might not admit a
valid AC power flow.

Solving the exact MINLP avoids the issue but is slow — on `case5_acdc.m` with all busbars
splittable, 232 s against 0.45 s for LPAC.

The feasibility check resolves this. It takes the topology from a fast model, freezes it
into a fixed network, and solves an ordinary AC/DC OPF on that. If the OPF converges, the
topology is AC-feasible and its objective is directly comparable to the baseline. If it does
not, the topology is discarded.

This is what makes the fast formulations usable: LPAC finds candidate topologies, the check
validates and prices them.

## Usage

```julia
prepare_AC_feasibility_check_AC_busbars(
    result_dict,      # result from run_acdc_BuS_*
    input_dict,       # the split network that result came from
    input_ac_check,   # deepcopy of input_dict — MUTATED IN PLACE
    switch_couples,   # from AC_busbars_split
    extremes_dict,    # from AC_busbars_split
    input_base,       # the original, unsplit network
)
```

The DC counterpart is `prepare_AC_feasibility_check_DC_busbars`, with the same signature and
the DC-side arguments.

```julia
data_split, sw_couples, extremes = _PMTP.AC_busbars_split(data, 2)
result = _PMTP.run_acdc_BuS_AC(data_split, LPACCPowerModel, gurobi)

data_fc = deepcopy(data_split)
_PMTP.prepare_AC_feasibility_check_AC_busbars(
    result, data_split, data_fc, sw_couples, extremes, data)

result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)
```

The three points that catch people out:

1. **The third argument is mutated.** The function's return value is not meaningful; the
   output is the transformed `input_ac_check`. Always pass a `deepcopy`.
2. **All six arguments are required**, including the original unsplit network, which is used
   to work out which bus indices predate the split.
3. **It prints extensively.** Every reconnection is logged. This is deliberate — when a
   check fails, the log is how you find out which element ended up where.

## What it does

The function walks the switch states in `result_dict` and rebuilds a fixed network:

**If the coupler is closed** (`status ≥ 0.9`) — no split occurred. Every element is
reconnected to the original busbar and the auxiliary buses are deleted, returning the
substation to its original form.

**If the coupler is open** (`status < 0.9`) — the busbar is genuinely split. Each element is
reconnected to whichever half its closed switch pointed at. The two halves persist as
separate buses. Elements whose switches are both open are left disconnected.

In both cases the switch and auxiliary-bus scaffolding is removed, leaving a network that a
standard AC/DC OPF can consume without any knowledge of switches.

## Interpreting the outcome

```julia
result_fc["termination_status"]   # LOCALLY_SOLVED / INFEASIBLE / ...
result_fc["objective"]            # comparable to the AC-OPF baseline
```

| Outcome | Meaning |
|---|---|
| Converged, objective below baseline | The topology is AC-feasible and beneficial. Use it. |
| Converged, objective above baseline | AC-feasible but not helpful — the approximation's apparent saving was an artefact. |
| Infeasible | The topology is not realizable. Discard it. |

The published results show no infeasible outcomes from LPAC-BuS topologies across the tested
cases, but that is empirical, not a guarantee. LPAC is an approximation, not a relaxation,
so nothing rules infeasibility out.

You will regularly see the second outcome with SOC and QC. Those relaxations typically leave
the topology unchanged, so the check returns exactly the AC-OPF objective — a null result,
correctly reported.

## Comparing across formulations

The objective from a relaxed or approximated model is **not** comparable to an AC-OPF
objective. LPAC-BuS reporting 181.909 \$/h against an AC-OPF baseline of 194.139 \$/h does
not mean a 6.3 % saving; the LPAC-OPF of the same network already reports 183.924 \$/h, so
most of that gap is formulation error, not topology benefit.

Only compare like with like:

```julia
opf_ac   = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)   # 194.139
bus_lpac = _PMTP.run_acdc_BuS_AC(data_split, LPACCPowerModel, gurobi)       # 181.909 ← not comparable

data_fc = deepcopy(data_split)
_PMTP.prepare_AC_feasibility_check_AC_busbars(bus_lpac, data_split, data_fc, sw, ext, data)
fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)      # 186.349 ← comparable

saving = 100 * (opf_ac["objective"] - fc["objective"]) / opf_ac["objective"]  # 4.01 %
```

## Combined AC and DC splits

Apply both checks to the same target network, in sequence:

```julia
data_fc = deepcopy(data_both)
_PMTP.prepare_AC_feasibility_check_AC_busbars(res, data_ac_split, data_fc, sw_ac, ext_ac, data)
_PMTP.prepare_AC_feasibility_check_DC_busbars(res, data_dc_split, data_fc, sw_dc, ext_dc, data)
result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)
```

Both calls mutate `data_fc` cumulatively. Note that each takes the network *it* was prepared
from as its second argument, not the combined one.

## Multiconductor variants

For bipolar DC modelling, use `prepare_AC_feasibility_check_AC_busbars_multiconductor` and
`prepare_AC_feasibility_check_DC_busbars_multiconductor`. Signatures are identical; they
additionally handle the per-terminal switch structure.

## Troubleshooting

**`KeyError` on a bus index.** Usually the wrong `input_base`. The function computes
`maximum(parse.(Int, keys(input_base["bus"])))` to tell original buses from generated ones,
so it must receive the network as it was *before* any splitting.

**Result is unchanged from the original network.** The coupler was closed — the model found
no benefit in splitting. Confirm by inspecting
`result["solution"]["switch"][coupler_id]["status"]`.

**AC-OPF fails on the reconstructed network.** Check the print log for an element that was
reconnected to a bus you did not expect, or for an element that was disconnected entirely
(both couple switches open). A disconnected generator or load will often render the case
infeasible.