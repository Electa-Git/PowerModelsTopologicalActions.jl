# Known issues and gotchas

Every item here was found by reading the source. They are documented rather than hidden
because most of them fail *silently* — the model solves, returns a plausible number, and is
wrong. Knowing about them up front is considerably cheaper than discovering them.

---

## Critical

### Some split variants still wipe the other side's switch couples

`build_BuS_AC_DC` posts the DC exclusivity, ZIL and BuS-OTS constraints by iterating
`:dcswitch_couples` (and the AC ones by iterating `:switch_couples`). If either dictionary is
empty, those constraints are silently never posted: the model still solves, but the answer
is meaningless, and typically over-optimistic, because elements are free to connect to both
halves of a busbar at once.

**Fixed for the main split functions.** `AC_busbars_split` and `DC_busbars_split` now only
initialize the other side's couple dictionary when it is missing, so they can be combined in
either order. On v0.1.0, calling `AC_busbars_split` after `DC_busbars_split` still wipes
`data["dcswitch_couples"]`; call AC first there.

**Still present** in the variants:

| Function | Resets |
|---|---|
| `AC_busbars_split_ordered` | `data["dcswitch_couples"]` |
| `AC_busbars_split_multiconductor` | `data["dcswitch_couples"]` |
| `DC_busbars_split_multiconductor` | `data["switch_couples"]` |
| `DC_busbars_split_multiconductor_updated` | `data["switch_couples"]` |

When combining these with a split on the other side, call them first, and in any case assert
before solving — there is no error and no warning otherwise:

```julia
@assert !isempty(data["switch_couples"])
@assert !isempty(data["dcswitch_couples"])
```

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

### Some split functions mutate, some copy the original input data dictionary

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
is_open = sw["status"] < 0.01        # ✅
is_open = sw["status"] == 0         # ✗ will silently never fire
```

---

## Structural gaps

### Test coverage is limited to the 5-bus case

`Pkg.test()` now runs a real suite (`test/runtests.jl`):

- solver-free structural checks of `AC_busbars_split` and `DC_busbars_split` (number of
  couplers and switches, couples pointing at existing switches, input not mutated,
  multi-busbar splits);
- the 5-bus AC/DC-OPF baseline (194.139 \$/h);
- LPAC-BuS on busbar 2 followed by the AC feasibility check (186.349 \$/h), and, when a
  Gurobi licence is available, the exact AC-BuS MINLP through Juniper. Without a licence the
  suite falls back to SCIP and skips the MINLP check.

Not covered: OTS, DC and combined AC/DC busbar splitting, the multiconductor variants, the
screening metrics and the plotting helper. Validate changes to those against the published
results in [Formulations](formulations.md).

`test/example_scripts/5_buses_test_case_BuS_test.jl` is a worked reference script for the
5-bus workflow, not a test: it has no `@test` assertions and is not called from
`runtests.jl`.


### Big-M values are hardcoded

`M_va = 2π`, `M_vm = 1.0`, `M_dc = 1.0`, written inline in `src/formdcgrid/acp.jl`,
`lpac.jl`, `shared.jl`, and `dcp.jl`. There is no way to configure them from the data or the
call site. They are conservative, which supposedly weakens the LP relaxation and slows branch-and-bound. Tightening them is the highest-leverage performance change available, and requires editing the source. [Indicator constraints](https://support.gurobi.com/hc/en-us/articles/4414392016529-How-do-I-model-general-conditional-statements-in-Gurobi) seem not to speed up the formulation.

### No N-1 constraints

The optimized topology is not checked against contingencies. Splitting a busbar reduces
redundancy, so a topology that is optimal here may violate N-1. Listed as future work in the
reference paper.

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

