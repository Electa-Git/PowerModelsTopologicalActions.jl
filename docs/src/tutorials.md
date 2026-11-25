# Tutorials

All tutorial material lives in the
[`tutorials/`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/tree/main/tutorials)
folder of the repository.

## ETCH webinar tutorial (November 2026)

| File | Contents |
|---|---|
| [`26_11_23_Tutorial_PMTA.jl`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/blob/main/tutorials/26_11_23_Tutorial_PMTA.jl) | The tutorial script, written in [Literate.jl](https://github.com/fredrikekre/Literate.jl) format: prose in `#` comments, code in between |
| [`26_11_23_Tutorial_PMTA.pdf`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/blob/main/tutorials/26_11_23_Tutorial_PMTA.pdf) | The slides presented during the webinar |

The tutorial was presented at the Energy Transmission Competence Hub (ETCH) webinar on
23 November 2026. No prior Julia/JuMP experience is assumed; familiarity with optimal power
flow helps.

### What it covers

| Section | Mode | Contents |
|---|---|---|
| 0. Pre-flight | code | installs the packages, defines the solvers, runs the 5-bus AC/DC OPF baseline (194.139 \$/h) |
| 1–3 | slides | state of play of grid topology optimization, the model formulation (decision variables, constraints, big-M reformulation, objective), applications |
| 4. Tutorial I | live coding | busbar splitting on busbar 2 of `case5_acdc.m`: prepare the data, inspect the switches, solve with AC (Juniper) and LPAC (SCIP), run the AC feasibility check, compare branch flows with `plot_branch_flows_comparison`; exercise on splitting all busbars |
| 5. Tutorial II | live coding | busbar screening on `pglib_opf_case30_ieee.m` with `compute_metrics_per_bus`, with exercises comparing the shortlist against splitting every busbar one at a time, trying the IEEE 118-bus case, and splitting combinations of busbars |
| 6. Limitations and roadmap | slides | double-busbar only, identical switching units, no N-1, no protection coordination, deterministic operating point |

### Running it

The tutorial uses only free solvers — Ipopt, Juniper and SCIP — so no licence is needed:

```julia
using Pkg
Pkg.add(["PowerModels", "PowerModelsACDC", "JuMP", "Ipopt", "Juniper", "SCIP", "Plots", "StatsPlots"])
Pkg.add(url = "https://github.com/Electa-Git/PowerModelsTopologicalActions.jl")  # or `add PowerModelsTopologicalActions#main`
```

The script uses the post-v0.1.0 function names (`run_BuS_AC`, …), so install the
development version as above rather than the registered release.

Then open the script in an editor with a Julia REPL (e.g. VS Code) and run it block by block.
The bundled test cases are found through `pathof(PowerModelsTopologicalActions)`, so the
script works from any directory.

With a Gurobi licence, add `using Gurobi`, define
`gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer, "OutputFlag" => 0)` and swap
`scip` for `gurobi` in the calls — including Juniper's `mip_solver`.

!!! note "Work in progress"
    Some cells are deliberately left as exercises (e.g. the list of busbars in section 4.5
    and the three parts of Tutorial II), and the LPAC-OPF baseline is commented out pending a
    fix upstream in PowerModelsACDC. Cells that depend on these will not run as-is.

## Other scripts

| File | Contents |
|---|---|
| [`webinar_busbar_splitting.jl`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/blob/main/tutorials/webinar_busbar_splitting.jl) | Earlier version of the webinar material. The Jupyter-notebook and Markdown builds in `tutorials/build/` have been removed; `make_webinar.jl` can regenerate them with Literate.jl |
| [`PTMP_demo.jl`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/blob/main/tutorials/PTMP_demo.jl) | Busbar-splitting demo script (uses Gurobi) |
| [`OTS_demo.jl`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/blob/main/tutorials/OTS_demo.jl) | OTS demo script. Still imports the package under an old name (`PowerModelsTopologicalActionsII`) and needs that line changed before it runs |
| [`results/`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl/tree/main/tutorials/results) | Pre-computed AC- and LPAC-BuS results for busbar 2 of the 5-bus case, in JSON, for checking without a MIP solver |
