# Formulations

The AC power flow equations are non-convex. Combined with binary switching variables, the
exact problem is a MINLP — solvable, but slowly. The package therefore implements several
formulations that trade exactness for tractability, and this page is about choosing between
them.

Note that **every** formulation here remains non-convex overall, because the binaries are
non-convex regardless. What the relaxations and approximations change is the *continuous*
part of each node in the branch-and-bound tree.

## Overview

| Formulation | Model type | Class | Solver | Guarantee |
|---|---|---|---|---|
| AC polar | `ACPPowerModel` | MINLP | Juniper + Ipopt + MIP | Exact (local optimum) |
| SOC relaxation | `SOCWRPowerModel` | MISOCP | Gurobi, Mosek | Lower bound |
| QC relaxation | `QCRMPowerModel` | MIQCP | Gurobi | Lower bound, tighter than SOC |
| LPAC approximation | `LPACCPowerModel` | MIQCP | Gurobi | None — approximation |
| DC approximation | `DCPPowerModel` | MILP | Gurobi, HiGHS | None — approximation |

## AC polar (`ACPPowerModel`)

The exact non-convex formulation. Voltages in polar coordinates, full AC branch equations,
DC grid in its exact quadratic form. Bilinear switch-voltage coupling constraints are
reformulated with big-M so the continuous relaxation stays tractable.

Use it when: the case is small, or you need a reference solution to validate a faster model
against.

Because the underlying NLP is non-convex, Juniper returns a **local** optimum. There is no
global optimality certificate, even when the solver reports success. The optimized topology
is still valid and still reduces cost — it just may not be the best possible topology.

## SOC relaxation (`SOCWRPowerModel`)

Lifts the voltage products into a higher-dimensional W-space:

```
(Uᵐᵢ)² → Wᵢ        Uᵐᵢ · Uᵐⱼ → Wᵢⱼ
(U^dc_e)² → W^dc_e  U^dc_e · U^dc_f → W^dc_ef
```

which convexifies the branch flow equations into second-order cone constraints. Solvable by
MISOCP solvers, and a valid lower bound on the exact problem.

In practice, on the published test cases, SOC-BuS does not find beneficial splits — it
returns the original topology. It is useful for bounding, not for generating topologies.

## QC relaxation (`QCRMPowerModel`)

Uses the same W-space lifting plus convex envelopes around the trigonometric and bilinear
terms, built with McCormick relaxations. Tighter than SOC, at higher computational cost.

For the DC part of the grid, the Bus Injection SOC and QC relaxations are mathematically
equivalent, so the difference between the two shows up only on the AC side.

Same practical caveat as SOC: on the tested cases it leaves the topology unchanged.

## LPAC approximation (`LPACCPowerModel`)

**The recommended default.** Unlike the classical DC approximation, LPAC retains voltage
magnitudes and reactive power, so it represents the physics well enough that its topologies
survive an AC feasibility check. It uses a piecewise-linear approximation of the cosine term
and a Taylor expansion for the remaining nonlinearities.

This is the Cold-Start variant: target voltages `Ṽ = 1` p.u., with power flows expressed
through the voltage magnitude change `φᵢ − φⱼ`. In this package it is built as a
convex-quadratic model, following the PowerModels.jl implementation, for solver efficiency.

Why it is the default:

- 10–200× faster than the exact MINLP on the published cases;
- topologies were AC-feasible in every tested case;
- captures most of the achievable saving (4.01 % vs 5.24 % on the 5-bus, all-busbars case).

Being an approximation rather than a relaxation, its objective is neither an upper nor a
lower bound on the true optimum, and its feasibility is not guaranteed. Always run the
[AC feasibility check](@ref).

## DC approximation (`DCPPowerModel`)

The classical active-power-only linearization. Constraint implementations exist in
`src/formconv/dcp.jl` and `src/formdcgrid/dcp.jl`, but this path is not exercised by the
reference paper and is not validated here.

The known problem with DC-based topology optimization is well documented in the literature:
solutions frequently turn out AC-infeasible, and because the DC model is an approximation
rather than a relaxation, its objective is not a valid bound either. If you need speed,
LPAC gives you comparable tractability with far better physical fidelity.

## Support matrix

| Problem | ACP | SOC | QC | LPAC | DCP |
|---|:-:|:-:|:-:|:-:|:-:|
| `run_acdcots_AC` | ✅ | — | — | — | — |
| `run_acdcots_DC` | ✅ | — | — | — | — |
| `run_acdcots_AC_DC` | ✅ | — | — | — | — |
| `run_acdc_BuS_AC` | ✅ | ✅ | ✅ | ✅ | ~ |
| `run_acdc_BuS_DC` | ✅ | ✅ | ✅ | ✅ | ~ |
| `run_acdc_BuS_AC_DC` | ✅ | ✅ | ✅ | ✅ | ~ |

`~` = code exists, not validated. OTS is implemented only for the exact formulation; the
same relaxation approach would apply, but it has not been done here.

## Empirical comparison

From the reference paper, `case5_acdc.m` with all AC busbars splittable:

| Model | BuS obj. [\$/h] | Time [s] | Feasibility-check obj. [\$/h] | AC-feasible | Benefit |
|---|---|---|---|:-:|---|
| AC-OPF (baseline) | — | 0.014 | — | ✅ | — |
| AC-BuS big-M | 184.972 | 232.377 | 183.972 | ✅ | 5.24 % |
| SOC-BuS | 183.763 | 0.301 | 194.139 | ✅ | none |
| QC-BuS | 183.761 | 0.331 | 194.139 | ✅ | none |
| LPAC-BuS | 181.909 | 0.453 | 186.349 | ✅ | 4.01 % |

The relaxations' feasibility-check objectives landing exactly on the 194.139 baseline is the
tell: they returned the unsplit topology.

Scaling to larger cases, AC busbar split:

| Case | AC-BuS obj. | time [s] | LPAC-BuS obj. | time [s] | speed-up |
|---|---|---|---|---|---|
| 39-bus | 2507.830 | 30.3 | 2468.200 | 0.5 | 61× |
| 67-bus | 122.232 | 118.1 | 120.942 | 0.5 | 236× |
| 588-bus | 376.523 | 958.1 | 370.248 | 58.8 | 16× |
| 3120-bus | 214.231 | 12560.0 | 211.154 | 534.0 | 24× |

At 3120 buses the exact model takes three and a half hours. LPAC takes nine minutes. For any
iterative workflow — screening candidate busbars, sweeping operating points — that
difference is the difference between feasible and not.

## Choosing

- **Small case, need a reference** → `ACPPowerModel`
- **Screening many busbars, or a large network** → `LPACCPowerModel` + feasibility check
- **Need a valid lower bound** → `SOCWRPowerModel` or `QCRMPowerModel`
- **Production workflow** → LPAC for search, ACP to confirm the final candidate