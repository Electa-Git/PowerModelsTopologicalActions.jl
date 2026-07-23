# Optimal transmission switching

Optimal Transmission Switching decides the on/off state of network elements so as to
minimize generation cost. Counter-intuitively, removing a line can *reduce* cost: it
redistributes power flows away from congested corridors, letting cheaper generation
dispatch that would otherwise be constrained off.

Each switchable element carries a binary variable equal to `1` when energized and `0` when
disconnected. The package provides three problem specifications, differing only in which
elements get one.

## Problem specifications

```julia
run_acdcots_AC(data, model_type, solver; kwargs...)
run_acdcots_DC(data, model_type, solver; kwargs...)
run_acdcots_AC_DC(data, model_type, solver; kwargs...)
```

| Function | AC branches | DC branches | AC/DC converters |
|---|:-:|:-:|:-:|
| `run_acdcots_AC` | ✅ | — | — |
| `run_acdcots_DC` | — | ✅ | ✅ |
| `run_acdcots_AC_DC` | ✅ | ✅ | ✅ |

All three are exported, so the `_PMTP.` prefix is optional. Each accepts either a parsed
data dictionary or a file path:

```julia
# from a Dict (preferred — you usually want to inspect or modify the data first)
data = _PM.parse_file("case5_acdc.m")
_PMACDC.process_additional_data!(data)
result = run_acdcots_AC(data, ACPPowerModel, juniper; setting = s)

# from a path (parses and calls process_additional_data! for you)
result = run_acdcots_AC("case5_acdc.m", ACPPowerModel, juniper; setting = s)
```

`kwargs` are forwarded to `PowerModels.solve_model`, so `setting`, `solution_processors`,
and the rest of the usual PowerModels machinery work as expected.

## What each model builds

### `run_acdcots_AC`

Adds `_PM.variable_branch_indicator` — one binary `z_ac` per AC branch — and swaps the AC
branch constraints for their on/off counterparts:

- Ohm's law from/to (`constraint_ohms_yt_*_on_off`)
- voltage angle difference (`constraint_voltage_angle_difference_on_off`)
- thermal limits from/to (`constraint_thermal_limit_*_on_off`)

Bus voltages use `variable_bus_voltage_on_off`, which relaxes voltage bounds on buses that
may become islanded. The DC grid is modelled exactly as in a standard AC/DC OPF.

### `run_acdcots_DC`

Adds `variable_dc_branch_indicator` (`z_dc`, one per DC branch), `variable_dc_conv_indicator`
(`z_cv`, one per converter), and a voltage slack variable. The corresponding constraints
are all package-local:

| Constraint | Purpose |
|---|---|
| `constraint_ohms_ots_dc_branch` | DC Ohm's law, deactivated when `z_dc = 0` |
| `constraint_branch_limit_on_off_dc_ots` | forces DC branch flow to zero when open |
| `constraint_converter_losses_dc_ots` | converter loss curve, zeroed when `z_cv = 0` |
| `constraint_converter_current_ots` | converter current, zeroed when `z_cv = 0` |
| `constraint_conv_transformer_dc_ots` | converter transformer |
| `constraint_conv_reactor_dc_ots` | converter phase reactor |
| `constraint_converter_limit_on_off_dc_ots` | zeroes AC- and DC-side converter power |

The pairing of the loss and current constraints is what guarantees that a de-energized
converter contributes exactly zero losses, rather than the constant term `a_cv` of its loss
curve. This matters: without it the model would pay standby losses for equipment it has
switched off.

### `run_acdcots_AC_DC`

The union of the two. Binaries on AC branches, DC branches, and converters, all optimized
in one problem.

## Reading the results

Switching decisions are reported into the standard PowerModels solution dictionary:

| Element | Path | Key |
|---|---|---|
| AC branch | `result["solution"]["branch"][id]` | `br_status` |
| DC branch | `result["solution"]["branchdc"][id]` | `br_status` |
| Converter | `result["solution"]["convdc"][id]` | `conv_status` |

```julia
opened = [id for (id, br) in result["solution"]["branch"] if br["br_status"] < 0.1]
println("de-energized AC branches: ", opened)
```

These are continuous solver outputs. Even with a binary declaration they arrive as `0.9997`
rather than `1.0`, so always threshold rather than testing for equality.

## Restricting which elements may switch

The models make every branch and converter switchable by default. This is convenient for
small studies and untenable for anything else — binary count is what drives solve time, and
in a real network most lines cannot be arbitrarily de-energized anyway.

There is no built-in API for selecting a subset. The practical workaround is to fix the
binaries you do not want free, via `PowerModels`' variable start/bound machinery, or by
pre-filtering the data. Fixing *all* AC binaries to 1 and leaving the DC ones free
reproduces `run_acdcots_DC`; fixing all of them reduces the problem to a conventional
AC/DC OPF, which is a useful sanity check that your setup is correct.

Automatic selection of the most promising lines is listed as future work in the reference
paper. The literature it points to for line-ranking heuristics — LMP-difference screening,
sensitivity analysis, congestion-zone identification — is a reasonable starting point if
you need to build this yourself.

## Formulation support

OTS is implemented for the exact non-convex formulation only. Relaxations and linear
approximations are available for busbar splitting, not for OTS.

```julia
result = run_acdcots_AC_DC(data, ACPPowerModel, juniper; setting = s)   # ✅
result = run_acdcots_AC_DC(data, LPACCPowerModel, gurobi; setting = s)  # ✗ not supported
```

The rationale is that the paper's contribution is the BuS model; the same relaxation
strategy would apply to OTS, but it has not been implemented or validated here.

## Scalability

Expect the exact MINLP to become intractable well before the largest bundled cases. Timings
from the reference paper, with a four-hour limit:

| Case | Binaries (AC & DC) | AC-OTS | DC-OTS | AC/DC-OTS |
|---|---|---|---|---|
| 5-bus | 13 | 0.23 s | 0.35 s | 1.18 s |
| 39-bus | 68 | 9.8 s | 11.6 s | 35.9 s |
| 67-bus | 122 | 1.7 s | 4.8 s | 5.0 s |
| 588-bus | — | did not converge | did not converge | did not converge |
| 3120-bus | — | did not converge | did not converge | did not converge |

The 67-bus case being faster than the 39-bus one despite having nearly twice the binaries
is a reminder that branch-and-bound time is governed by how quickly the bound tightens, not
by problem size alone.

For the larger cases, restricting the switchable set is not an optimization — it is a
precondition for getting an answer at all.

## Interpreting a zero saving

`run_acdcots_DC` returning exactly the AC-OPF objective is a normal, informative outcome
rather than a failure. It means no DC switching action improves on the base topology for
that operating point. In the published results this is what happens on the 5-bus and 67-bus
cases; only the 39-bus case, under modified low-load conditions, sees a benefit from
switching DC elements.

The economic benefit of DC switching comes mainly from avoiding losses in lightly loaded
converters and DC branches while routing power over parallel paths. That mechanism only has
something to work with when the DC grid is meshed and lightly loaded. On a radial or
heavily loaded DC grid, expect no saving.

## Caveat: protection

The model tells you that a topology is cheaper. It does not tell you that it is safe to
operate. Coordination between the elements selected for switching and the grid's protection
strategy has to be verified separately, at the planning stage, before any of these actions
would be used in practice.