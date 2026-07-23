# API reference

Function-by-function listing of the public surface. The package carries almost no
docstrings, so this page is maintained by hand; signatures below were read directly from the
source.

Names marked **(exported)** can be called unqualified after `using
PowerModelsTopologicalActions`. Everything else needs the `PowerModelsTopologicalActions.`
prefix (or an alias such as `_PMTP.`).

---

## Problem specifications

### Optimal transmission switching

#### `run_acdcots_AC` **(exported)**

```julia
run_acdcots_AC(file::String, model_type::Type, solver; kwargs...)
run_acdcots_AC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
```

OTS with switchable AC branches. The `String` method parses the file and calls
`PowerModelsACDC.process_additional_data!` before delegating to the `Dict` method.

- `model_type` — `ACPPowerModel` (only validated option)
- `solver` — a JuMP optimizer; MINLP-capable, e.g. Juniper
- `kwargs` — forwarded to `PowerModels.solve_model`; commonly `setting = s`

Returns a PowerModels result dictionary. Switching states appear at
`result["solution"]["branch"][id]["br_status"]`.

Model builder: `build_acdcots_AC`.

#### `run_acdcots_DC` **(exported)**

```julia
run_acdcots_DC(file::String, model_type::Type, solver; kwargs...)
run_acdcots_DC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
```

OTS with switchable DC branches and AC/DC converters. AC branches remain fixed.

Results at `result["solution"]["branchdc"][id]["br_status"]` and
`result["solution"]["convdc"][id]["conv_status"]`.

Model builder: `build_acdcots_DC`.

#### `run_acdcots_AC_DC` **(exported)**

```julia
run_acdcots_AC_DC(file::String, model_type::Type, solver; kwargs...)
run_acdcots_AC_DC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
```

Joint AC and DC OTS. All of the above are switchable simultaneously.

Model builder: `build_acdcots_AC_DC`.

### Busbar splitting

#### `run_acdc_BuS_AC`

```julia
run_acdc_BuS_AC(data, model_constructor, optimizer; kwargs...)
```

Busbar splitting on AC busbars. `data` must be a network prepared by `AC_busbars_split`.

- `model_constructor` — `ACPPowerModel`, `SOCWRPowerModel`, `QCRMPowerModel`, or
  `LPACCPowerModel`
- `optimizer` — matched to the formulation, see [Formulations](@ref)

Results at `result["solution"]["switch"][id]["status"]`.

Model builder: `build_acdc_BuS_AC`. Objective: `objective_min_fuel_cost_ac_switch`.

#### `run_acdc_BuS_DC`

```julia
run_acdc_BuS_DC(data, model_constructor, optimizer; kwargs...)
```

Busbar splitting on DC busbars. Requires a network prepared by `DC_busbars_split`.

Results at `result["solution"]["dcswitch"][id]["status"]`.

Model builder: `build_acdc_BuS_DC`. Objective: `objective_min_fuel_cost_dc_switch`.

#### `run_acdc_BuS_AC_DC`

```julia
run_acdc_BuS_AC_DC(data, model_constructor, optimizer; kwargs...)
```

Simultaneous AC and DC busbar splitting. Requires a network prepared by **both**
`AC_busbars_split` and `DC_busbars_split`.

!!! note "Call order matters"
    Call `AC_busbars_split` **before** `DC_busbars_split`. The reverse order clears
    `data["dcswitch_couples"]`. See [Known issues and gotchas](@ref).

Model builder: `build_acdc_BuS_AC_DC`. Objective: `objective_min_fuel_cost_ac_dc_switch`.

---

## Data preparation

### `AC_busbars_split`

```julia
AC_busbars_split(data_original, bus_to_be_split)
    → (data, switch_couples, extremes_ZIL)
```

Prepares a hybrid AC/DC network for AC busbar splitting. `bus_to_be_split` is an `Int` or a
`Vector{Int}`. **Copies** its input.

Returns the expanded network, the switch-couple dictionary, and a map from each split busbar
to the indices of its two halves. See [Data model](@ref).

Requires `data["switch"]` to exist and to be empty. Resets `data["dcswitch_couples"]` to an
empty dictionary, so call this before `DC_busbars_split`, not after.

### `DC_busbars_split`

```julia
DC_busbars_split(data_original, bus_to_be_split)
    → (data, dcswitch_couples, extremes_ZIL_dc)
```

DC-side equivalent. **Copies** its input. Creates `data["dcswitch"]`, overwriting any
existing content. Preserves an existing `data["switch_couples"]`, so it is safe to call
after `AC_busbars_split`.

### `AC_busbar_split_AC_grid`

```julia
AC_busbar_split_AC_grid(data, bus_to_be_split) → (data, switch_couples, extremes_ZIL)
```

For AC-only networks with no DC components. **Mutates** its input.

### `AC_busbars_split_ordered`

```julia
AC_busbars_split_ordered(data, bus_to_be_split) → (data, switch_couples, extremes_ZIL)
```

Variant preserving bus ordering. **Mutates** its input.

### Multiconductor variants

```julia
AC_busbars_split_multiconductor(data, bus_to_be_split)
DC_busbars_split_multiconductor(data, bus_to_be_split)
DC_busbars_split_multiconductor_updated(data, bus_to_be_split)
```

For bipolar / multiconductor DC modelling. DC switch entries carry an additional `terminal`
key. Prefer `DC_busbars_split_multiconductor_updated` over the older variant.

### Switch-couple helpers

```julia
compute_couples_of_switches(data)          → Dict
compute_couples_of_dcswitches(data)        → Dict
compute_couples_of_dcswitches_mc(data)     → Dict
compute_couples_of_switches_feas_check(data) → Dict
eliminate_duplicates_couple_of_switches(switch_couples)
```

Called internally by the split functions. Use directly if you have modified the switch set
and need to rebuild the couples. `compute_couples_of_switches_feas_check` skips the
duplicate-elimination step and keeps both orientations.

### Element inspection

```julia
elements_AC_busbar_split(data) → Dict
elements_DC_busbar_split(data) → Dict
```

Reports which generators, loads, branches, and converters are attached to each busbar flagged
for splitting. Prints as it goes. Lives in the module marked `DO NOT USE`, but is read-only
and safe.

### Not for use

```julia
AC_busbar_split_more_buses_fixed(data, bus_to_be_split)
DC_busbar_split_more_buses_fixed(data, bus_to_be_split)
```

`src/core/busbar_splitting_fixed.jl` opens with `## TO BE FIXED, DO NOT USE ###`. Listed for
completeness only.

### Warm starts

```julia
prepare_starting_value_dict(result, grid)      → Dict
prepare_starting_value_dict_lpac(result, grid) → Dict
```

Build starting-value dictionaries from a previous result, e.g. seeding an ACP solve with an
LPAC solution.

---

## Feasibility checking

```julia
prepare_AC_feasibility_check_AC_busbars(
    result_dict, input_dict, input_ac_check, switch_couples, extremes_dict, input_base)

prepare_AC_feasibility_check_DC_busbars(
    result_dict, input_dict, input_ac_check, switch_couples, extremes_dict, input_base)

prepare_AC_feasibility_check_AC_busbars_multiconductor(...)
prepare_AC_feasibility_check_DC_busbars_multiconductor(...)
```

Reconstruct a fixed-topology network from a busbar-splitting result.

- `result_dict` — result from a `run_acdc_BuS_*` call
- `input_dict` — the split network that result came from
- `input_ac_check` — **mutated in place**; pass a `deepcopy`
- `switch_couples`, `extremes_dict` — from the corresponding split function
- `input_base` — the original, unsplit network

Returns nothing meaningful; the output is the mutated third argument. Then solve
`PowerModelsACDC.solve_acdcopf` on it. Verbose by design. See
[AC feasibility check](@ref).

---

## Reference extensions

### `add_ref_dcgrid_dcswitch!`

```julia
add_ref_dcgrid_dcswitch!(ref::Dict{Symbol,<:Any}, data::Dict{String,<:Any})
```

Ref extension building the DC-grid arc structures plus DC switch arcs (`:arcs_dc_sw`,
`:busdc_arcs_sw`). Applied automatically by the BuS problem specifications; you only need it
if you are writing your own builder.

### `buspair_parameters_dc`

```julia
buspair_parameters_dc(arcs_dcgrid_from, branches, buses)
```

Computes bus-pair level structures for the DC grid.

---

## Objectives

```julia
objective_min_fuel_cost_ac_switch(pm)      # gen cost + AC coupler penalty
objective_min_fuel_cost_dc_switch(pm)      # gen cost + DC coupler penalty
objective_min_fuel_cost_ac_dc_switch(pm)   # gen cost + both

calc_gen_cost(pm)         # Σ cost[end-1] · pg  — LINEAR TERM ONLY
calc_ac_switch_cost(pm)   # Σ cost · (1 − z_switch) over non-auxiliary switches
calc_dc_switch_cost(pm)   # Σ cost · (1 − z_dcswitch) over non-auxiliary switches
```

!!! warning
    `calc_gen_cost` uses only `g["cost"][end-1]`, the linear coefficient. Quadratic cost
    terms in your case data are ignored.

---

## Variables

Switch indicators and powers:

```julia
variable_switch_indicator(pm; nw, relax=false, report=true)      # z_switch    → :switch/:status
variable_dc_switch_indicator(pm; nw, relax=false, report=true)   # z_dcswitch  → :dcswitch/:status
variable_switch_power(pm; kwargs...)                             # psw, qsw
variable_dc_switch_power(pm; nw, bounded=true, report=true)      # p_dcsw
variable_switch_current(pm; kwargs...)
```

OTS indicators:

```julia
variable_dc_branch_indicator(pm; nw, relax=false, report=true)   # z_ots_dc  → :branchdc/:br_status
variable_dc_conv_indicator(pm; nw, relax=false, report=true)     # z_conv_dc → :convdc/:conv_status
variable_branch_ots(pm; nw, relax=false, report=true)
variable_voltage_slack_ots(pm; nw, bounded=true, report=false)
```

Setting `relax = true` replaces the binary declaration with box bounds `[0, 1]`, which is
useful for diagnosing whether a difficult solve is driven by the combinatorics or the
physics.

Linearised and single-period variants exist with `_linearised` and `_sp` suffixes.

---

## Constraints

Constraint templates are in `src/core/constraint_template.jl`; formulation-specific
implementations are in `src/formconv/` (converters) and `src/formdcgrid/` (DC grid and
switches), split by model type: `acp.jl`, `wr.jl`, `wrm.jl`, `lpac.jl`, `dcp.jl`,
`shared.jl`.

Switch constraints:

| Function | Purpose |
|---|---|
| `constraint_switch_voltage_on_off_big_M` | big-M voltage coupling across an AC switch |
| `constraint_dc_switch_voltage_on_off_big_M` | ditto, DC |
| `constraint_switch_power_on_off` | zero power through an open AC switch |
| `constraint_dc_switch_power_on_off` | ditto, DC |
| `constraint_switch_thermal_limit` | apparent power limit on an AC switch |
| `constraint_dc_switch_thermal_limit` | active power limit on a DC switch |
| `constraint_exclusivity_switch` | `z_f + z_t ≤ 1` |
| `constraint_exclusivity_switch_no_OTS` | equality form, forbids disconnection |
| `constraint_ZIL_switch` | coupler closed ⇒ elements on the original half |
| `constraint_BS_OTS_branch` | zero flow for a fully disconnected element |
| `constraint_power_balance_ac_switch` | AC nodal balance including switch flows |
| `constraint_power_balance_dc_switch` | DC nodal balance including switch flows |

OTS constraints:

| Function | Purpose |
|---|---|
| `constraint_ohms_ots_dc_branch` | DC Ohm's law with on/off |
| `constraint_branch_limit_on_off_dc_ots` | DC branch limits with on/off |
| `constraint_converter_losses_dc_ots` | converter losses, zeroed when off |
| `constraint_converter_current_ots` | converter current with on/off |
| `constraint_converter_limit_on_off_dc_ots` | converter power limits with on/off |
| `constraint_conv_transformer_dc_ots` | converter transformer with on/off |
| `constraint_conv_reactor_dc_ots` | converter reactor with on/off |
| `constraint_conv_filter_dc_ots` | converter filter with on/off |

---

## Relaxation helpers

```julia
relaxation_complex_product_conic(m, a, b, c)
relaxation_complex_product_conic_on_off(m, a, b, c, d, z)
relaxation_complex_product_on_off(m, a, b, c, d)
```

Convex relaxations of `c² + d² ≤ a·b`, with on/off variants for the switched case.