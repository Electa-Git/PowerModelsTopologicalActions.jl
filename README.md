# PowerModelsTopologicalActions.jl
 
[![CI](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/actions/workflows/CI.yml)
[![Docs](https://img.shields.io/badge/docs-dev-blue.svg)](https://electa-git.github.io/PowerModelsTopologicalActions.jl/dev/)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-green.svg)](LICENSE)
[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.segan.2026.102182-blue)](https://doi.org/10.1016/j.segan.2026.102182)
 
A Julia/JuMP package for **steady-state grid topology optimization in hybrid AC/DC power systems**. It computes which lines to de-energize (Optimal Transmission Switching, OTS) and how to reconfigure substations (Busbar Splitting, BuS) to minimize total generation cost, subject to the full physics of hybrid AC/DC grids.
 
Built on [PowerModels.jl](https://github.com/lanl-ansi/PowerModels.jl) and
[PowerModelsACDC.jl](https://github.com/Electa-Git/PowerModelsACDC.jl).
 
This is the first package able to perform both OTS and busbar splitting on **either part**
of a hybrid AC/DC grid. While OTS is well established for AC systems, busbar splitting
remains largely unexplored — particularly on the DC side.
 
## Capabilities
 
**Topological actions**
 
- AC / DC / combined AC-DC optimal transmission switching
- AC / DC / combined AC-DC busbar splitting
- Busbar splitting combined with OTS on the affected elements
**Power flow formulations**
 
- AC polar coordinates — exact, MINLP
- SOC relaxation (W-space) — MISOCP
- QC relaxation (W + λ-space) — MIQCP
- LPAC approximation (cold start) — MIQCP
- DC approximation — MILP, partial support
## Installation
 
```julia
using Pkg
Pkg.add(url = "https://github.com/Electa-Git/PowerModelsTopologicalActions.jl")
```
 
Requires Julia ≥ 1.10 and a solver appropriate to your formulation — Juniper + Ipopt + a MIP
solver for the exact MINLP, or Gurobi/Mosek/HiGHS for the relaxations and approximations.
 
## Quick example
 
```julia
using PowerModels;                   const _PM     = PowerModels
using PowerModelsACDC;               const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActions; const _PMTP   = PowerModelsTopologicalActions
using JuMP, Ipopt, Gurobi, Juniper
 
gurobi  = JuMP.optimizer_with_attributes(Gurobi.Optimizer, "MIPGap" => 1e-4)
ipopt   = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer,
              "nl_solver" => ipopt, "mip_solver" => gurobi)
 
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
 
data = _PM.parse_file("data_sources/case5_acdc.m")
_PMACDC.process_additional_data!(data)
 
# --- baseline ---
result_opf = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)
 
# --- optimal transmission switching ---
result_ots = _PMTP.run_acdcots_AC_DC(data, ACPPowerModel, juniper; setting = s)
 
# --- busbar splitting: prepare → solve → check ---
data_split, switch_couples, extremes = _PMTP.AC_busbars_split(data, 2)
result_bus = _PMTP.run_acdc_BuS_AC(data_split, LPACCPowerModel, gurobi)
 
data_fc = deepcopy(data_split)
_PMTP.prepare_AC_feasibility_check_AC_busbars(
    result_bus, data_split, data_fc, switch_couples, extremes, data)
result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)
 
println("baseline:  ", result_opf["objective"])   # 194.139 $/h
println("after BuS: ", result_fc["objective"])    # 186.349 $/h  → 4.0 % saving
```
 
Busbar splitting always takes three stages: prepare the data, optimize, then verify the
resulting topology is AC-feasible. The verification step is mandatory for every formulation
except the exact `ACPPowerModel`.
 
## Documentation
 
Full documentation is in [`docs/`](docs/src):
 
| Page | Contents |
|---|---|
| [Installation](docs/src/installation.md) | solver stacks, settings, bundled test cases |
| [Quick start](docs/src/quickstart.md) | complete worked OTS and BuS examples |
| [Optimal transmission switching](docs/src/ots.md) | OTS problem specifications and scaling |
| [Busbar splitting](docs/src/busbar_splitting.md) | the three-stage workflow in detail |
| [AC feasibility check](docs/src/feasibility_check.md) | validating relaxed and approximated topologies |
| [Formulations](docs/src/formulations.md) | choosing between ACP, SOC, QC, LPAC, DC |
| [Data model](docs/src/data_model.md) | what the split functions do to your network dictionary |
| [API reference](docs/src/api.md) | function-by-function listing |
| [Known issues and gotchas](docs/src/known_issues.md) | **read this first** |
 
To build the HTML docs locally:
 
```console
$ julia --project=docs -e 'using Pkg; Pkg.develop(PackageSpec(path=pwd())); Pkg.instantiate()'
$ julia --project=docs docs/make.jl
```
 
## Running the tutorials
 
`tutorials/` contains worked demo scripts. They use their own environment, which needs to be
pointed at your local copy of the package before first use:
 
```julia
using Pkg
Pkg.activate("tutorials")
Pkg.develop(PackageSpec(path = pwd()))   # run from the repository root
Pkg.instantiate()
```
 
Then:
 
```console
$ julia --project=tutorials tutorials/PTMP_demo.jl
```
 
The tutorial environment includes Gurobi. If you do not have a licence, replace the solver
definitions in the script with the open-source stack used by the test suite (Ipopt + HiGHS +
Juniper) — see [Installation](docs/src/installation.md).
 
## Performance
 
The LPAC approximation is the recommended default. It is 10–200× faster than the exact
MINLP and produced AC-feasible topologies on every published test case.
 
| Case | AC-BuS (MINLP) | LPAC-BuS | Speed-up |
|---|---|---|---|
| 39-bus | 30.3 s | 0.5 s | 61× |
| 67-bus | 118.1 s | 0.5 s | 236× |
| 588-bus | 958.1 s | 58.8 s | 16× |
| 3120-bus | 12560.0 s | 534.0 s | 24× |
 
## Status
 
Research code accompanying a peer-reviewed publication. The models are sound and validated
against the published results, but the package is not hardened: `Pkg.test()` currently runs
no assertions, one exported name does not resolve, and some helper functions carry
`DO NOT USE` markers. Every rough edge we know about is documented in
[Known issues and gotchas](docs/src/known_issues.md) rather than left to be discovered.
 
Contributions are welcome, particularly:
 
- turning `test/scripts/5_buses_test_case_BuS_test.jl` into an asserting regression test
  driven from `runtests.jl` (the 5-bus objectives quoted above make good fixtures)
- docstrings on the exported problem specifications
- configurable big-M values, currently hardcoded per formulation file
- an API for restricting the switchable element set
- N-1 security constraints
## Citing
 
> G. Bastianel, M. Vanin, D. Van Hertem, H. Ergun, "Optimal transmission switching and
> busbar splitting in hybrid AC/DC grids", *Sustainable Energy, Grids and Networks*, vol. 46,
> 2026, 102182. [doi:10.1016/j.segan.2026.102182](https://doi.org/10.1016/j.segan.2026.102182)
 
```bibtex
@article{bastianel2026topological,
  title   = {Optimal transmission switching and busbar splitting in hybrid AC/DC grids},
  author  = {Bastianel, Giacomo and Vanin, Marta and Van Hertem, Dirk and Ergun, Hakan},
  journal = {Sustainable Energy, Grids and Networks},
  volume  = {46},
  pages   = {102182},
  year    = {2026},
  doi     = {10.1016/j.segan.2026.102182}
}
```
 
## Acknowledgements
 
Developed as part of WP1 of the ETF DIRECTIONS project, funded by the FOD Economie of the Belgian Government.
 
Primary developer: Giacomo Bastianel ([@GiacomoBastianel](https://github.com/GiacomoBastianel)).
Contributor: Marta Vanin ([@MartaVanin](https://github.com/MartaVanin)).