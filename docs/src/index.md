# PowerModelsTopologicalActions.jl

`PowerModelsTopologicalActions.jl` is a Julia/JuMP package for **steady-state grid topology optimization in AC and hybrid AC/DC grids**. It computes which lines to de-energize (Optimal Transmission Switching, OTS) and how to reconfigure selected substations (Busbar Splitting, BuS) to minimize total generation costs in the system, subject to the full physics of hybrid AC/DC grids.

It is built on [PowerModels.jl](https://github.com/lanl-ansi/PowerModels.jl) and
[PowerModelsACDC.jl](https://github.com/Electa-Git/PowerModelsACDC.jl), and follows their
conventions throughout: the same network data dictionaries, the same `model_type` /
`solver` calling signature, the same `result["solution"]` layout.
This is the first package able to perform both OTS and busbar splitting on **either part** of a hybrid AC/DC grid. While OTS and BuS have been used for decades in AC grids, they remain largely unexplored on the DC side. Long story short, with this package, one can optimize the grid topology with OTS and BuS, with a deep focus on BuS.

## Capabilities
 
**Topological actions**
 
- AC / DC / combined AC-DC optimal transmission switching
- AC / DC / combined AC-DC busbar splitting
- Busbar splitting combined with OTS on selected busbars.

## Power Flow formulations

**Most refined**
- AC polar coordinates — exact, MINLP
- LPAC approximation (cold start) — MIQCP


**To be further refined**
- SOC relaxation — MISOCP
- QC relaxation — MIQCP
- DC approximation — MILP, partial support

## Where to start

- [Installation](installation.md) — getting the package and a solver stack running.
- [Quick start](quickstart.md) — a complete OTS and a complete BuS run on the 5-bus AC/DC case.
- [Optimal transmission switching](ots.md) — the OTS problem specifications.
- [Busbar splitting](busbar_splitting.md) — the three-stage BuS workflow, which is the more involved one.
- [Data model](data_model.md) — what `AC_busbars_split` actually does to your network dictionary.
- [AC feasibility check](feasibility_check.md) — how to verify that a relaxed or approximated topology is
  AC-feasible.
- [API reference](api.md) — function-by-function listing.
- [Known issues and gotchas](known_issues.md) — read this before you spend a day debugging.


## Status
 
Research code accompanying a peer-reviewed publication. The models are sound and validated
against the published results, but the package is still being developed to include functionalities described in further publications.
 
Contributions are welcome, particularly: 
- docstrings on the exported problem specifications
- configurable big-M values, currently hardcoded per formulation file
- an API for restricting the switchable element set
- N-1 security constraints
- an API visually plotting the results of the grid topology optimization models (switches status and substation topology)

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
 
Developed as part of WP1 of the [ETF DIRECTIONS project](https://etch.be/en/directions-design-protection-and-control-offshore-dc-power-grids-and-power-hubs), funded by the FOD Economie of the Belgian Government in which [Etch - Energy Transmission Competence Hub](https://etch.be/en) - [EnergyVille](https://energyville.be/en/) and [KU Leuven](https://www.kuleuven.be/kuleuven) collaborated with Elia Group to explore the future of electrical energy hubs.

Primary developer: 
Giacomo Bastianel ([@GiacomoBastianel](https://github.com/GiacomoBastianel)).

Contributors: 
Marta Vanin ([@MartaVanin](https://github.com/MartaVanin)) $\rightarrow$ conceptualization of the package

