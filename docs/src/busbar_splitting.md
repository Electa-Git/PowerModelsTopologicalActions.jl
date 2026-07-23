# Busbar splitting

Busbar Splitting reconfigures a substation instead of removing a line. A double-busbar
substation whose coupler is closed behaves as a single electrical node; open the coupler and
it becomes two nodes that are physically adjacent but electrically distinct, with
independent voltage magnitudes and angles. Every element attached to the substation must
then choose a side.

The effect is to redistribute power flows without sacrificing any transmission capacity —
which is why it is often more attractive than switching a line out entirely.

## The three-stage workflow

Unlike OTS, busbar splitting requires the network to be restructured before it can be
optimized. There are three stages, and skipping any of them produces either an error or a
misleading answer.

```
   ┌─────────────────────┐
   │  1. PREPARE DATA    │   AC_busbars_split / DC_busbars_split
   │                     │   → expands the network with auxiliary
   │                     │     buses and switches
   └──────────┬──────────┘
              │  data_split, switch_couples, extremes_ZIL
              ▼
   ┌─────────────────────┐
   │  2. OPTIMIZE        │   run_acdc_BuS_AC / _DC / _AC_DC
   │                     │   → switch states as decision variables
   └──────────┬──────────┘
              │  result
              ▼
   ┌─────────────────────┐
   │  3. FEASIBILITY     │   prepare_AC_feasibility_check_*
   │     CHECK           │   then a plain AC/DC OPF
   │                     │   → verifies and prices the topology
   └─────────────────────┘
```

Stage 3 is skippable only when stage 2 used `ACPPowerModel`, since that formulation is
already exact. For every relaxation and approximation it is mandatory — see
[AC feasibility check](@ref) for why.

## Stage 1: preparing the network

```julia
data_split, switch_couples, extremes_ZIL = AC_busbars_split(data, bus_to_be_split)
data_split, dcswitch_couples, extremes_ZIL_dc = DC_busbars_split(data, busdc_to_be_split)
```

`bus_to_be_split` is either an `Int` or a vector of `Int`s. Both functions `deepcopy` their
input, so the original network is left untouched.

For each busbar you nominate, the transformation:

1. duplicates the busbar into parts `i` and `i′`;
2. inserts a **busbar coupler** — a Zero Impedance Line (ZIL) modelled as a switch —
   between them;
3. detaches every element attached to the busbar (generators, loads, branches, converters)
   and gives each its own **auxiliary bus**;
4. connects each auxiliary bus to *both* halves through a **pair of switches**.

The optimizer then chooses, per element, which of its two switches is closed, plus whether
the coupler is open or closed. The total switch count is

```
n_switches = Σ_b (2 · n_b) + B
```

where `B` is the number of busbars being split and `n_b` the number of elements attached to
busbar `b`. This grows fast, and it is the reason splitting every busbar in a large network
is not practical.

See [Data model](@ref) for the resulting dictionary layout and the meaning of every key the
transformation adds.

### The returned tuple

All three elements matter, and two of them are needed again in stage 3.

- **`data_split`** — the expanded network. Pass this to the `run_acdc_BuS_*` functions.
- **`switch_couples`** — a dictionary of switch pairs. Each entry has `f_sw` and `t_sw`
  (the two switches connecting one element to the two busbar halves), `bus_split` (which
  original busbar), and `switch_split` (the index of that busbar's coupler). It is also
  stored on `data_split["switch_couples"]`, where the model reads it from.
- **`extremes_ZIL`** — maps each split busbar to the indices of its two parts.

### Variants

| Function | Use case |
|---|---|
| `AC_busbars_split` | Standard AC busbar splitting in a hybrid AC/DC grid. **Start here.** |
| `DC_busbars_split` | DC busbar splitting in a hybrid AC/DC grid. |
| `AC_busbar_split_AC_grid` | AC-only networks with no DC components. Mutates its input. |
| `AC_busbars_split_ordered` | Preserves bus ordering. Mutates its input. |
| `AC_busbars_split_multiconductor` | Multiconductor / bipolar DC modelling. |
| `DC_busbars_split_multiconductor` | Ditto, DC side. |
| `AC_busbar_split_more_buses_fixed` | **Marked `DO NOT USE` in the source.** |
| `DC_busbar_split_more_buses_fixed` | **Marked `DO NOT USE` in the source.** |

Only `AC_busbars_split` and `DC_busbars_split` copy their input. The others mutate, so
`deepcopy` first if you need the original.

## Stage 2: optimizing

```julia
run_acdc_BuS_AC(data_split, model_constructor, optimizer; kwargs...)
run_acdc_BuS_DC(data_split, model_constructor, optimizer; kwargs...)
run_acdc_BuS_AC_DC(data_split, model_constructor, optimizer; kwargs...)
```

!!! note "These are not exported"
    Unlike the OTS functions, the BuS entry points must be called qualified:
    `_PMTP.run_acdc_BuS_AC(...)`.

Choose the function matching the preparation you performed. `run_acdc_BuS_AC_DC` requires a
network prepared by *both* `AC_busbars_split` and `DC_busbars_split`, applied in that order —
see the warning in [Quick start](@ref).

### The constraint set

For each switch, the model enforces:

| Constraint | Meaning |
|---|---|
| `constraint_switch_voltage_on_off_big_M` | when closed, the two buses share voltage magnitude and angle; when open, they are free |
| `constraint_switch_power_on_off` | active and reactive power through an open switch is zero |
| `constraint_switch_thermal_limit` | apparent power through a closed switch respects its rating |

For each switch couple:

| Constraint | Meaning |
|---|---|
| `constraint_exclusivity_switch` | `z_f + z_t ≤ 1` — an element connects to at most one half |
| `constraint_ZIL_switch` | if the coupler is closed (no split), elements stay on the original half |
| `constraint_BS_OTS_branch` | if neither switch closes, the element is disconnected and carries no power |

The exclusivity constraint is an inequality, not an equality, which is what allows an
element to be dropped entirely — busbar splitting with OTS on the affected elements. If you
want to forbid that and force every element back onto one half or the other, the
`constraint_exclusivity_switch_no_OTS` variant provides the equality form, though no
shipped problem specification uses it.

### The big-M reformulation

The natural way to state "these two buses have equal voltage when the switch is closed" is
bilinear:

```
z · θ_m = z · θ_i
z · U_m = z · U_i
```

which would make even the linear formulations non-convex. The package instead uses the
standard big-M form:

```
−(1 − z) · M_θ  ≤  θ_m − θ_i  ≤  (1 − z) · M_θ
−(1 − z) · M_U  ≤  U_m − U_i  ≤  (1 − z) · M_U
```

The constants are hardcoded per formulation file:

| Constant | Value | Applies to |
|---|---|---|
| `M_va` | `2π` | AC voltage angle difference |
| `M_vm` | `1.0` | AC voltage magnitude difference (p.u.) |
| `M_dc` | `1.0` | DC voltage magnitude difference (p.u.) |

These are deliberately conservative. Tightening them would strengthen the LP relaxation and
speed up branch-and-bound considerably; the reference paper flags optimal big-M selection
as future work and points to Pineda et al. (2024) for the methodology. If you are fighting
solve times on a large case, this is the highest-leverage thing to change, and it means
editing the constants in `src/formdcgrid/acp.jl`, `lpac.jl`, `shared.jl`, and `dcp.jl`.

### The objective

```
min  Σ_k c₁ₖ · Pᵍₖ  +  Σ_ZIL c_sw · (1 − z_sw)
```

Two things to note.

First, **only the linear generation cost coefficient is used**. `calc_gen_cost` reads
`g["cost"][end-1]`, so a quadratic term in your case data is silently ignored. This matches
the paper, which sets quadratic coefficients to zero, but it will surprise you if your case
relies on them.

Second, a **small penalty is charged for each open busbar coupler**. Auxiliary switches are
free (`cost = 0.0`); couplers cost `1.0` by default. This ensures the model only splits a
busbar when there is a real economic benefit, rather than splitting gratuitously among
equal-cost optima. If splitting never happens on a case where you expect it to, check
whether this penalty is swamping a small saving — you can adjust it per switch:

```julia
for (id, sw) in data_split["switch"]
    get(sw, "ZIL", false) && (sw["cost"] = 0.1)
end
```

## Stage 3: checking AC feasibility

Covered in full on the [AC feasibility check](@ref) page. In brief:

```julia
data_fc = deepcopy(data_split)
prepare_AC_feasibility_check_AC_busbars(
    result, data_split, data_fc, switch_couples, extremes_ZIL, data_original)
result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)
```

## Reading the results

```julia
for (sw_id, sw) in result["solution"]["switch"]
    status = sw["status"] < 0.1 ? "OPEN" : "closed"
    is_zil = get(data_split["switch"][sw_id], "ZIL", false)
    println("switch $sw_id ($(is_zil ? "coupler" : "element")): $status")
end
```

DC switches appear under `result["solution"]["dcswitch"]` with the same `status` key.

**A busbar was actually split if and only if its coupler is open.** Auxiliary switches
being open only tells you which side each element picked, or — if both switches of a couple
are open — that the element was dropped from the network entirely.

## Practical guidance

**Split one busbar at a time.** The published results split a single busbar in each of the
larger cases, precisely because binary count is the binding constraint. All-busbars-split is
demonstrated only on the 5-bus case, where it takes 232 s against 13 s for the single-busbar
version.

**Screen with LPAC, confirm with AC.** Run LPAC-BuS per candidate busbar, keep the topology
with the best AC-feasible objective. This is the procedure used in the paper and it is
roughly two orders of magnitude cheaper than screening with the MINLP.

**Expect the convex relaxations to find nothing.** SOC and QC produce tight-looking
objective values but, on the published cases, leave the topology unchanged. They are useful
as bounds, not as topology generators. LPAC, despite being an approximation rather than a
relaxation, is the formulation that actually finds beneficial splits.

**Fix elements that cannot move.** In a real substation, some elements are hard-wired to one
half. Modelling those as fixed rather than switchable removes two binaries each. The
underlying data model supports it — see [Data model](@ref) — though there is no convenience
API for it yet.

## Limitations

- Only **double-busbar** configurations are modelled. Breaker-and-a-half and double-bus
  double-breaker arrangements are not supported.
- All switching units are treated identically. Real substations distinguish circuit
  breakers, disconnectors, and load-break switches, each with different operating
  capabilities; the model gives them all the same treatment and the same thermal rating.
- Switch ratings default to `psw = qsw = thermal_rating = 100.0` p.u., which is effectively
  unlimited on the bundled cases. Override them on `data_split["switch"]` if you want them
  to bind.
- **No N-1 security constraints.** The optimized topology is not verified against
  contingencies. Given that splitting a busbar reduces redundancy, this is a material
  limitation for operational use and is listed as future work.
- **No stochastic RES modelling.** Single deterministic operating point only.