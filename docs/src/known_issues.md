# Known issues and gotchas

Every item here was found by reading the source. They are documented rather than hidden
because most of them fail *silently* — the model solves, returns a plausible number, and is
wrong. Knowing about them up front is considerably cheaper than discovering them.

---

## Critical

### Split AC busbars before DC busbars

**Affects:** any combined AC + DC busbar splitting workflow.

`AC_busbars_split` ends by resetting `data["dcswitch_couples"]` to an empty dictionary:

```julia
data["switch_couples"] = deepcopy(switch_couples)
data["dcswitch_couples"] = Dict{String,Any}()     # ← unconditional reset
```

`DC_busbars_split` no longer does the mirror-image thing — it preserves an existing
`switch_couples` — so the AC-then-DC order is safe:

```julia
data, sw_ac, ext_ac = AC_busbars_split(data, ac_bus)   # ✅
data, sw_dc, ext_dc = DC_busbars_split(data, dc_bus)
```

The reverse order is not. Calling `AC_busbars_split` second wipes the DC couples, and
`build_acdc_BuS_AC_DC` then iterates an empty `:dcswitch_couples`, so
`constraint_exclusivity_dc_switch`, `constraint_ZIL_dc_switch`, and
`constraint_BS_OTS_dcbranch` are never posted. The model still solves; the answer is
meaningless, and typically over-optimistic, because DC elements are free to connect to both
halves of a busbar at once.

There is no error and no warning. If you ever reorder these calls, assert first:

```julia
@assert !isempty(data["switch_couples"])
@assert !isempty(data["dcswitch_couples"])
```

### `run_acdcsw_AC_DC` is exported but does not exist

`src/prob/acdc_BuS_AC_DC.jl` declares `export run_acdcsw_AC_DC`, but no such function is
defined in any included file — the implementation lives in `src/prob/acdcsw_AC_DC.jl`, which
is not in the module's include list. The name shows up in tab-completion and in
`names(PowerModelsTopologicalActions)`, then fails with `UndefVarError` when called. Use
`run_acdc_BuS_AC_DC`.

---

## Silent behavioural surprises

### Quadratic generation costs are ignored

`calc_gen_cost` reads only the linear coefficient:

```julia
if length(g["cost"]) ≥ 2
    JuMP.add_to_expression!(cost, g["cost"][end-1], _PM.var(pm, :pg, g_id))
end
```

So `cost = [c₂, c₁, c₀]` contributes only `c₁ · Pg`. Both `c₂` and the constant `c₀` are
dropped. This is consistent with the reference paper, which zeroes quadratic terms, but if
your case relies on them the objective will not be what you expect — and the objective of a
BuS run will not be comparable to a PowerModelsACDC OPF on the same data, which *does* use
the full cost curve.

**Workaround:** linearize your cost curves before use, or compare only against other runs
from this package.

### Switch ratings default to effectively infinite

Every generated switch gets `psw = qsw = thermal_rating = 100.0` p.u. On a 100 MVA base that
is 10 GVA — never binding on the bundled cases. If you intended switch ratings to constrain
the solution, set them explicitly:

```julia
for (id, sw) in data_split["switch"]
    sw["thermal_rating"] = 3.0
    sw["psw"] = 3.0
    sw["qsw"] = 3.0
end
```

### The busbar-coupler penalty may suppress small savings

Each coupler carries `cost = 1.0`, charged as `cost · (1 − z)` when opened. That is
deliberate — it stops the model splitting busbars for no gain — but on a case with a small
absolute saving it can dominate. If splitting never occurs where you expect it, reduce the
penalty on the ZIL switches and re-run.

### `AC_busbars_split` assumes no pre-existing switches

It writes `data["switch"]["1"]`, `["2"]`, … from scratch. A case that already contains
switches will have them overwritten.

It also indexes `data["switch"]` without creating it, so if that key is absent — an unusual
but possible state for a hand-built dictionary — you get a `KeyError`. Initialize it first:

```julia
haskey(data, "switch") || (data["switch"] = Dict{String,Any}())
```

### Some split functions mutate, some copy

| Function | Behaviour |
|---|---|
| `AC_busbars_split` | copies |
| `DC_busbars_split` | copies |
| `AC_busbar_split_AC_grid` | **mutates** |
| `AC_busbars_split_ordered` | **mutates** |

`deepcopy` before calling the mutating ones.

### Binary results are floats

Even declared-binary variables come back as `0.9999999` or `3.2e-9`. Always threshold:

```julia
is_open = sw["status"] < 0.1        # ✅
is_open = sw["status"] == 0         # ✗ will silently never fire
```

---

## Structural gaps

### `Pkg.test()` still checks nothing

`test/runtests.jl` is a zero-byte file, so `Pkg.test()` passes trivially and gives no
regression protection.

`test/scripts/5_buses_test_case_BuS_test.jl` now contains the full 5-bus busbar-splitting
workflow, which is a useful worked reference, but it is a demo script rather than a test: it
contains no `@test` assertions, nothing calls it from `runtests.jl`, and its data path
(`joinpath(@__DIR__, "data_sources", ...)`) resolves to `test/scripts/data_sources/` while
the case files actually live in `test/data_sources/`.

Until that is wired up, validate changes against the published results in
[Formulations](@ref). The 5-bus AC-OPF objective of 194.139 \$/h and the AC-BuS big-M result
of 184.972 \$/h are good canaries.

### `src/prob/old_to_be_replaced/`

Contains `acdcsw_AC_different_formulations.jl`, `acdcsw_AC_multiconductor.jl`,
`redispatch.jl`, `acdcsw_AC_droop.jl`, `acdcsw_AC_hour.jl`. None are included in the module.
`acdcsw_AC_droop.jl` declares four exports that consequently do nothing. Treat as dead code.

### `busbar_splitting_fixed.jl` is marked unusable

The file opens `## TO BE FIXED, DO NOT USE ###`. `AC_busbar_split_more_buses_fixed` and
`DC_busbar_split_more_buses_fixed` should not be called. `elements_AC_busbar_split` and
`elements_DC_busbar_split`, in the same file, are read-only inspection helpers and are fine.

### Big-M values are hardcoded

`M_va = 2π`, `M_vm = 1.0`, `M_dc = 1.0`, written inline in `src/formdcgrid/acp.jl`,
`lpac.jl`, `shared.jl`, and `dcp.jl`. There is no way to configure them from the data or the
call site. They are conservative, which weakens the LP relaxation and slows branch-and-bound.
Tightening them is the highest-leverage performance change available, and requires editing
the source.

### No N-1 constraints

The optimized topology is not checked against contingencies. Splitting a busbar reduces
redundancy, so a topology that is optimal here may violate N-1. Listed as future work in the
reference paper.

### OTS has no relaxed formulations

`run_acdcots_*` work only with `ACPPowerModel`. There is no LPAC or SOC path for OTS, which
is why the larger cases do not converge within four hours.

---

## Performance troubleshooting

### The MINLP will not converge

Expected behaviour above roughly 100 buses with all elements switchable. In order of
effectiveness:

1. Switch to `LPACCPowerModel` and run an AC feasibility check afterwards.
2. Split one busbar rather than all of them.
3. Restrict the switchable element set.
4. Warm-start the ACP solve from an LPAC solution via `prepare_starting_value_dict`.
5. Tighten the big-M constants.

### Memory blow-up on large cases

Splitting all busbars in a 3120-bus network generates tens of thousands of switches. The
count is `Σ_b (2·n_b) + B`. Split one busbar at a time.

### The model runs but nothing switches

Work through, in order:

1. Is `data["switch_couples"]` non-empty? (See the first critical issue on this page.)
2. Is the coupler penalty larger than the available saving?
3. Is the case congested at all? An uncongested network has nothing for topology
   optimization to fix, and returning the original topology is the correct answer.
4. Are you using SOC or QC? On the published cases those consistently leave the topology
   unchanged. Try LPAC.