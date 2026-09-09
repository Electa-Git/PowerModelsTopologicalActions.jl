# Installation

## Requirements

- Julia ≥ 1.10
- `PowerModels.jl`
- `PowerModelsACDC.jl`
- At least one solver appropriate to the formulation you intend to use (see below)

## Installing the package

The package is not yet in the Julia General registry, so install it directly from GitHub:

```julia
   ] add PowerModelsTopologicalActions
```
 

Or, for development:

```console
$ git clone https://github.com/Electa-Git/PowerModelsTopologicalActions.jl
$ cd PowerModelsTopologicalActions.jl
$ julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

Then load it. The module name is long, so an alias is conventional:

```julia
using PowerModels;        const _PM     = PowerModels
using PowerModelsACDC;    const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActions; const _PMTP = PowerModelsTopologicalActions
```

## Choosing solvers

Which solver you need depends entirely on the formulation, because the formulation
determines the problem class. Getting this pairing wrong is the single most common source
of confusing failures. I recommend using Gurobi for the solving the topology optimization models. 
If you do not have access to it, I made available a test of the AC-feasibility check with some results obtained with Gurobi. 
Using a free solver instead of Gurobi is still something which is work in progress.

| Formulation | Problem class | Solver |
|---|---|---|
| `ACPPowerModel` | MINLP | [Juniper](https://github.com/lanl-ansi/Juniper.jl) wrapping an NLP solver (Ipopt) and a MIP solver (Gurobi) |
| `LPACCPowerModel` | MIQCP | Gurobi, or another MIQCP-capable solver |



### A working solver stack

This is the configuration used to produce the published results:

```julia
using JuMP, Ipopt, Gurobi, Juniper

gurobi  = JuMP.optimizer_with_attributes(Gurobi.Optimizer,
              "MIPGap" => 1e-4)

ipopt   = JuMP.optimizer_with_attributes(Ipopt.Optimizer,
              "tol" => 1e-6,
              "print_level" => 0)

juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer,
              "nl_solver"  => ipopt,
              "mip_solver" => gurobi,
              "time_limit" => 36000)
```

Use `juniper` for `ACPPowerModel`, `gurobi` for everything else. `ipopt` is more stable than `gurobi` for the `LPACCPowerModel` simulation of the OPF model. `gurobi` should be used for every grid topology optimization model. 


### Faster Ipopt

For the larger cases, Ipopt's default linear solver is a bottleneck. If you have access to
the HSL library, `ma97` is markedly faster:

```julia
using HSL_jll
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer,
            "tol" => 1e-6, "print_level" => 0, "linear_solver" => "ma97")
```

## Solver settings for the models

Both PowerModels and PowerModelsACDC take a `setting` dictionary. Topological-action
problems need branch flows reported in the solution, and converter losses modelled:

```julia
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

# if you also want dual variables from the convex/linear formulations
s_dual = Dict("output" => Dict("branch_flows" => true, "duals" => true),
              "conv_losses_mp" => true)
```

Pass it through as `setting = s` on every `run_*` call.

## Bundled test cases

Test networks live in `test/data_sources/` 

| File | Description |
|---|---|
| `case5_acdc.m` | 5 AC buses, 3 DC buses, 3 converters — the worked example throughout these docs |
| `case39_acdc.m` | 39 AC buses, 10 DC buses, 10 converters |
| `case67.m` | 67 AC buses, 9 DC buses, 9 converters |
| `pglib_opf_case588_sdet_acdc.m` | 588 AC buses, 7 DC buses |
| `case3120sp_mcdc.m` | 3120 AC buses, 5 DC buses |
| `cigre_b4_dc_grid.m` | CIGRE B4 DC grid |
| `case5.m`, `case14.m`, `case24.m`, `case30_ieee.m`, `case57_ieee.m`, `case118_ieee.m`, `case793_goc.m`, `case3375wp_k.m` | AC-only cases |