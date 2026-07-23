# PowerModelsTopologicalActions.jl

`PowerModelsTopologicalActions.jl` is a Julia/JuMP package for **steady-state grid topology
optimization in hybrid AC/DC power systems**. It computes which lines to de-energize
(Optimal Transmission Switching, OTS) and how to reconfigure substations (Busbar Splitting,
BuS) so as to minimize total generation cost, subject to the full physics of the coupled
AC and DC grid.

It is built on top of [PowerModels.jl](https://github.com/lanl-ansi/PowerModels.jl) and
[PowerModelsACDC.jl](https://github.com/Electa-Git/PowerModelsACDC.jl), and follows their
conventions throughout: the same network data dictionaries, the same `model_type` /
`solver` calling signature, the same `result["solution"]` layout.

What makes this package distinct is that **the switching actions apply to the DC side as
well as the AC side**. Existing topology-optimization tools treat HVDC links as fixed
injections; here DC branches, AC/DC converters, and DC busbars are first-class switchable
objects, and AC and DC actions can be optimized jointly in a single problem.

## Capabilities

**Topological actions**

| Action | AC part | DC part | Combined |
|---|:-:|:-:|:-:|
| Optimal Transmission Switching | ✅ | ✅ | ✅ |
| Busbar Splitting | ✅ | ✅ | ✅ |
| Busbar Splitting with OTS on the affected elements | ✅ | ✅ | ✅ |

**Power flow formulations**

| Formulation | Model type | Class | Notes |
|---|---|---|---|
| AC polar | `ACPPowerModel` | MINLP | Exact; big-M reformulated for BuS |
| SOC relaxation | `SOCWRPowerModel` | MISOCP | W-space lifting |
| QC relaxation | `QCRMPowerModel` | MIQCP | W + λ-space, McCormick envelopes |
| LPAC approximation | `LPACCPowerModel` | MIQCP | Cold-start; keeps voltage magnitudes and reactive power |
| DC approximation | `DCPPowerModel` | MILP | Partial support, see [Formulations](@ref) |

In practice the **LPAC formulation is the workhorse**: on the test cases reported in the
reference paper it runs 10–200× faster than the exact MINLP while still producing
AC-feasible topologies.

## Where to start

- [Installation](@ref) — getting the package and a solver stack running.
- [Quick start](@ref) — a complete OTS and a complete BuS run on the 5-bus AC/DC case.
- [Optimal transmission switching](@ref) — the OTS problem specifications.
- [Busbar splitting](@ref) — the three-stage BuS workflow, which is the more involved one.
- [Data model](@ref) — what `AC_busbars_split` actually does to your network dictionary.
- [AC feasibility check](@ref) — how to verify that a relaxed or approximated topology is
  AC-feasible.
- [API reference](@ref) — function-by-function listing.
- [Known issues and gotchas](@ref) — read this before you spend a day debugging.

## A note on maturity

This is research code that accompanies a journal paper. It is capable and the models are
sound, but it is not a hardened production library: `Pkg.test()` currently runs no
assertions, one exported name does not resolve, and a few helper functions carry
`DO NOT USE` markers in the source. The [Known issues and gotchas](@ref) page documents
every rough edge we are aware of, rather than leaving you to discover them.

## Citing

If you use this package in your research, please cite:

> G. Bastianel, M. Vanin, D. Van Hertem, H. Ergun, "Optimal transmission switching and
> busbar splitting in hybrid AC/DC grids", *Sustainable Energy, Grids and Networks*, vol.
> 46, 2026, 102182. [doi:10.1016/j.segan.2026.102182](https://doi.org/10.1016/j.segan.2026.102182)

## Acknowledgements

Developed as part of WP1 of the ETF DIRECTIONS project, funded by the FOD Economie of the
Belgian Government. Primary developer: Giacomo Bastianel
([@GiacomoBastianel](https://github.com/GiacomoBastianel)), with contributions from Marta
Vanin ([@MartaVanin](https://github.com/MartaVanin)).