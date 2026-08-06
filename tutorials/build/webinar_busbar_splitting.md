```@meta
EditURL = "../webinar_busbar_splitting.jl"
```

# Grid Topology Optimization with `PowerModelsTopologicalActions.jl`

**Energy Transmission Competence Hub (ETCH) Webinar & Tutorial**

** Giacomo Bastianel, PhD candidate at KU Leuven / Etch - EnergyVille **
| | |
|---|---|
| **Level** | familiar with optimal power flow simulations is a plus; no prior Julia/JuMP experience assumed |
| **Julia Package** | [`Electa-Git/PowerModelsTopologicalActions.jl`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl) |
| **References** | G. Bastianel, M. Vanin, D. Van Hertem, H. Ergun, *Optimal transmission switching and busbar splitting in hybrid AC/DC grids*, SEGAN 46 (2026) 102182 — [doi:10.1016/j.segan.2026.102182](https://doi.org/10.1016/j.segan.2026.102182) <br><br> G. Bastianel, D. Van Hertem, H. Ergun and L. A. Roald, *Identifying Best Candidates for Busbar Splitting*, Electric Power Systems Research, Volume 263, 2027, ISSN 0378-7796 - [doi.org/10.1016/j.epsr.2026.113611](https://doi.org/10.1016/j.epsr.2026.113611). |

### Menu of the session

| Step | Section | Mode |
|---|---|---|
| 0 | Pre-flight — *preparing for the session* | self-service |
| 1 | Grid topology optimization: state of play | slides presentation |
| 2 | Our topology optimization models | slides presentation |
| 3 | Applications of the proposed grid topology optimization models | slides presentation |
| 4 | Tutorial I — Topology optimization: basics | live coding |
| 5 | Tutorial II — Identifying relevant areas for topology optimization | live coding |
| 6 | Next steps | slides presentation |
| 7 | Q&A | open discussion |

!!! tip "How to use this page"
    Step 0 prepares all the packages needed to run the optimization models
    Steps 1–3 and 6 are complemented by a slide deck, which is available in the repository
    Steps 4–5 are meant to be run cell by cell, in order. Every code cell depends on the ones above it.

---
# 0. Pre-flight — preparing for the session

**Requirements**

Run the next cell. The `PowerModelsTopologicalActions.jl` package is live in the Julia General registry too, so it can be added as the other mature packages.
```julia
using Pkg
Pkg.add(["PowerModels", "PowerModelsACDC", "PowerModelsTopologicalActions" , "JuMP", "Ipopt", "Juniper", "SCIP"])
```

### Getting to know the code

If the next block prints **194.139**, you are all set. That number is the baseline for tutorial I and represents the generation costs for the OPF simulation on the 5-bus hybrid AC/DC test case.

````@example webinar_busbar_splitting
# Load the packages and define the solvers (note, we use SCIP as the MIP solver for the tutorial as it is available for free, but you can use Gurobi if you have a license)
using PowerModels;                   const _PM     = PowerModels
using PowerModelsACDC;               const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActions; const _PMTP   = PowerModelsTopologicalActions
using JuMP, Ipopt, Juniper, SCIP #Gurobi

# we are setting up the base directory
const PMTA_DIR = dirname(dirname(pathof(PowerModelsTopologicalActions)))

# --- solvers and their pairing ---------------------------
ipopt   = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0) # nonlinear solver
scip    = JuMP.optimizer_with_attributes(SCIP.Optimizer, "display/verblevel" => 0, "limits/gap" => 1e-4) # MIP/MINLP solver (free)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer,
              "nl_solver" => ipopt, "mip_solver" => scip, "time_limit" => 36000) # MINLP solver
# If you have a Gurobi licence: `using Gurobi` in the block above, define the
# optimizer here, and swap `scip` → `gurobi` in the calls below.
# gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer, "OutputFlag" => 0)

# output settings passed to every AC/DC OPF solve (branch flows + converter losses)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
````

`process_additional_data!` is needed when one has hybrid AC/DC grids —> it builds the DC-grid structures
that every downstream function expects.

````@example webinar_busbar_splitting
file = joinpath(PMTA_DIR, "test", "data_sources", "case5_acdc.m")
data = _PM.parse_file(file)
_PMACDC.process_additional_data!(data)

println("# AC buses: ", length(data["bus"]))
println("# DC buses: ", length(data["busdc"]))
println("# Converters: ", length(data["convdc"]))

result_opf = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)

println("termination: ", result_opf["termination_status"])   ## LOCALLY_SOLVED
println("primal status: ", result_opf["primal_status"])      ## FEASIBLE_POINT
println("objective:   ", result_opf["objective"], " USD/h")  ## 194.139
````

---
# 1. Grid topology optimization: state of play

**The framing.** Transmission grids are the bottleneck of the energy transition. New
lines are slow, expensive and politically contested, so the question becomes: how much
more can we get out of the network we already have?

Congestion is managed today mostly by **redispatch** — paying generators to move away
from their market schedule. It works, and it is expensive: roughly €2.77 bn in Germany
in 2024, USD 8.33 bn in the US, with the JRC projecting European redispatch and
congestion-management costs toward hundreds of bn€ per year by 2040 under
business as usual.

**Ideal** Comparing the transmission grid to a road network, congestion can be
compared to traffic jams spreading over the main roads, limiting the electricity
flow. The tutorial describes mathematical models for grid topology
optimisation, which reroutes electricity around grid congestion by dinamically
modifying the transmission grid topology through topological actions. These
topological actions can be compared to traffic lights on the road coupled to
direction signs on the asphalt, which help vehicles to flow smoothly.

```

   ┌───────────────────────────────┐
   │                               │
   │      SLIDES PRESENTATION      │
   │                               │
   └──────────-────────────────────┘

```
---
# 2. Our topology optimization models

We assume substations to have a double-busbar configuration, where the two busbars are linked by a **busbar coupler**.
Coupler closed → the substation is one electrical node.
Coupler open → two nodes, physically adjacent, electrically distinct, each with its own voltage magnitude and angle.

That is the entire physical idea: *splitting a busbar makes electrically close things
electrically distant, without moving anything.*
Every element that was attached to the
substation (generators, loads, lines, converters) is then assigned to either one part of the split busbar
or the other.

![Busbar splitting](https://raw.githubusercontent.com/Electa-Git/PowerModelsTopologicalActions.jl/main/docs/src/images/BuS_illustration_README.png)

- **Left** — the original busbar `i`, four elements attached.
- **Centre** — the modelling construct. Busbar `i` is duplicated into `i` and `i′`,
  joined by a **Zero Impedance Line (ZIL)**, the busbar coupler. Each element is
  detached onto its own **auxiliary bus** and linked to *both* halves through a **pair
  of switches**. Every switch, plus the coupler, carries a binary variable.
- **Right** — after optimization the open switches are deleted, leaving the optimized
  topology.

**The one-sentence takeaway:** the model does not choose a substation configuration
from a subset of available topologies. It optimizes the status of the busbar coupler and *per element*, which side of the bar it is connected to.

**Where the benefit comes from.** Splitting the busbar and connecting elements to either part of the split busbar reroutes flows around the binding element
inside the substation itself. In the 5-bus example we are about to run, splitting a
single AC busbar buys ≈ 4 % of total generation cost.

```
   ┌───────────────────────────────┐
   │                               │
   │      SLIDES PRESENTATION      │
   │                               │
   └──────────-────────────────────┘

```
---
# 2.1 Formulation of the optimization model

## 2.1.1 Decision variables

| Variable | Count | Meaning |
|---|---|---|
| `z_sw` on each **element switch** | 2 per element attached to a split busbar | element connects to this half (1) or not (0) |
| `z_sw` on each **coupler (ZIL)** | 1 per split busbar | busbar is one node (1) or split (0) |
| all standard OPF variables | — | voltages, angles, generation, converter set-points |

Switch count grows as

```math
n_{\text{switches}} = \sum_b \left(2 \, n_b\right) + B
```

with ``B`` the number of split busbars and ``n_b`` the elements attached to busbar ``b``

## 2.2 Constraints

Per switch:

| Constraint | Meaning |
|---|---|
| `constraint_switch_voltage_on_off_big_M` | closed → both buses share voltage magnitude and angle; open → decoupled |
| `constraint_switch_power_on_off` | no power flows through an open switch |
| `constraint_switch_thermal_limit` | a closed switch respects its rating |


Per switch couple:

| Constraint | Meaning |
|---|---|
| `constraint_exclusivity_switch` | ``z_f + z_t \leq 1`` — an element sits on at most one half |
| `constraint_ZIL_switch` | coupler closed (no split) → elements stay on the original half |
| `constraint_BS_OTS_branch` | both switches open → element disconnected, carries no power |


> **The inequality needs some attention.** Writing exclusivity as ``\leq 1`` rather
> than ``= 1`` lets the model leave *both* switches open, i.e. disconnecting the element
> entirely. That is busbar splitting **with OTS on the affected elements**, in a single
> formulation. The equality variant `constraint_exclusivity_switch_no_OTS` exists if
> you want to forbid it, but no shipped problem specification uses it.


## 2.3 The big-M reformulation

"Equal voltage when the switch is closed" is naturally bilinear:

```math
z \cdot \theta_m = z \cdot \theta_i \qquad z \cdot U_m = z \cdot U_i
```

which would make even the linear formulations non-convex. The package uses the standard
big-M form instead:

```math
-(1-z)\,M_\theta \;\leq\; \theta_m - \theta_i \;\leq\; (1-z)\,M_\theta
```
```math
-(1-z)\,M_U \;\leq\; U_m - U_i \;\leq\; (1-z)\,M_U
```

with `M_va = 2π`, `M_vm = 1.0`, `M_dc = 1.0`. These are deliberately conservative and
**hardcoded** in `src/formdcgrid/{acp,lpac,shared,dcp}.jl`.

## 2.4 The objective

```math
\min \; \sum_k c_{1k} \, P^g_k \;+\; \sum_{\text{ZIL}} c_{sw}\,(1 - z_{sw})
```

One thing to hear **before** running anything:
   **Opening a coupler costs 1.0 by default.** Element switches are free. This penalty
   stops the model splitting busbars gratuitously among equal-cost optimal solutions and on a
   case with a genuinely small saving it can swamp the benefit, so one sees no split at all.
---
# 3. Applications of the proposed grid topology optimization models
  Examples from SEGAN and PSCC papers

```

   ┌───────────────────────────────┐
   │                               │
   │      SLIDES PRESENTATION      │
   │                               │
   └──────────-────────────────────┘

```
---

# 4. Tutorial I — Topology optimization: basics

This is the core of the proposed grid topologyoptimization model.

```
  ┌─────────────────────┐
  │                     │   AC_busbars_split / DC_busbars_split
  │   1. PREPARE DATA   │   → expands the network with auxiliary
  │                     │     buses and switches by selecting which busbars to split
  └──────────┬──────────┘
             │  created dictionaries: data_split, switch_couples, extremes_ZIL
             ▼
  ┌─────────────────────┐
  │                     │   model:
  │    2. OPTIMIZE      │   run_acdc_BuS_AC / _DC / _AC_DC
  │                     │   → switch states as decision binary variables + normal OPF variables
  └──────────┬──────────┘
             │  result
             ▼
  ┌─────────────────────┐
  │                     │
  │  3. FEASIBILITY     │   prepare_AC_feasibility_check_* (fixing the switch states and the optimized topology)
  │        CHECK        │   then a plain AC/DC OPF
  │                     │   → feasibility check and computation of the benefits brought by the optimized topology
  └─────────────────────┘
```
## 4.1 Prepare data

Running an AC/DC OPF on the 5-bus hybrid test case, and storing the result for later comparison.

````@example webinar_busbar_splitting
result_opf = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)

println("termination: ", result_opf["termination_status"])   ## LOCALLY_SOLVED
println("objective:   ", result_opf["objective"], " USD/h")  ## 194.139
````

Preparation of the busbar splitting data structures. The busbar to split is bus 2

````@example webinar_busbar_splitting
bus_to_split = 2

data_bus, switch_couples, extremes_ZIL = _PMTP.AC_busbars_split(data, bus_to_split)
````

**Before:**

````@example webinar_busbar_splitting
```
        gen 1     load 1
           │         │
    ───────┴────┬────┴──┬────      busbar 2
                │       │
          branch 3   branch 5
```
````

**After `AC_busbars_split(data, bus_to_split)`:**

````@example webinar_busbar_splitting
```
     gen 1        load 1      branch 3     branch 5
        │            │            │            │
    aux bus 9    aux bus 10   aux bus 11   aux bus 12
       ╱ ╲          ╱ ╲          ╱ ╲          ╱ ╲
    sw4   sw5    sw6   sw7    sw8   sw9   sw10  sw11
     │     │      │     │      │     │      │     │
 ────┴─────┼──────┴─────┼──────┴─────┼──────┴─────┼────────      busbar 2
           │            |            |            |      │
           |            |            |            |      └───┐
           |            |            |            |          ／  sw1 (ZIL)
           |            |            |            |      ┌───┘
           |            |            |            |      │
   ────────┴────────────┴────────────┴────────────┴──────┴─      busbar 2'


```
````

Every element now has its own auxiliary bus and a pair of switches, one to each half. The ZIL busbar coupler `sw1 (ZIL)` decides whether the two halves are one node or two.

Three return values, and **two of them are needed again in the feasibility check**:

- `data_bus` — a *new* dictionary (the input is not mutated) with busbar 2 duplicated,
  its elements moved onto auxiliary buses, and switches inserted.
- `switch_couples` — the switch pairs whose exclusivity constraint decides where each
  element lands →  one can check all the couples of switches for each network element.
- `extremes_ZIL` — maps each split busbar to the indices of its two halves, thus the extremes of each busbar coupler

## 4.1 Inspect the data structure:
### The original busbar and its second half:

````@example webinar_busbar_splitting
@show data_bus["bus"]["2"]["split"]          ## true  — nominated for splitting
@show data_bus["bus"]["6"]["ZIL"]            ## true  — the second half
@show data_bus["bus"]["6"]["bus_split"]      ## 2     — derives from busbar 2
````

### An auxiliary bus hosting a detached element:

````@example webinar_busbar_splitting
@show data_bus["bus"]["9"]["auxiliary_bus"]  ## true
@show data_bus["bus"]["9"]["auxiliary"]      ## "gen"
@show data_bus["bus"]["9"]["original"]       ## 1  — which generator
````

### The two kinds of switch:

````@example webinar_busbar_splitting
@show data_bus["switch"]["1"]["ZIL"]         ## true  — the coupler, index 1
@show data_bus["switch"]["4"]["auxiliary"]   ## "gen" — an element switch
@show switch_couples["4"];
nothing #hide
````

To split several busbars at once, pass a vector, e.g.
`_PMTP.AC_busbars_split(data, [1, 2, 3, 4, 5])`.

## 4.2 Optimize

> **The discriminator used throughout the source** is the presence or absence of the
> `"auxiliary"` key: **couplers lack it, element switches have it.**
> `calc_ac_switch_cost` charges only switches without it; `compute_couples_of_switches`
> pairs only switches with it.

Two formulations, same problem specification.
The exact AC model is a MINLP, so it goes to `juniper`;
every approximate/relaxed formulation goes to the free MIP solver `scip`.

````@example webinar_busbar_splitting
# exact MINLP — slow, exact (local optimum)
@time result_bus_ac = _PMTP.run_acdc_BuS_AC(data_bus, ACPPowerModel, juniper)

# LPAC approximation — far faster, and what you will actually use
@time result_bus_lpac = _PMTP.run_acdc_BuS_AC(data_bus, LPACCPowerModel, scip)

println("AC-BuS MINLP objective: ", result_bus_ac["objective"])
println("LPAC-BuS objective: ", result_bus_lpac["objective"],
        "  ← not comparable to the AC baseline!")
````

### Reading the answer

````@example webinar_busbar_splitting
for sw_id in 1:length(data_bus["switch"])
    sw         = result_bus_lpac["solution"]["switch"]["$sw_id"]
    is_coupler = !haskey(data_bus["switch"]["$sw_id"], "auxiliary")
    status     = sw["status"] < 0.01 ? "OPEN" : "closed"

    if is_coupler
        println("switch $sw_id — BUSBAR COUPLER: $status")
    else
        elem = data_bus["switch"]["$sw_id"]
        side = elem["t_bus"] == extremes_ZIL["$bus_to_split"][1] ? "original half" : "second half"
        status == "closed" && println("switch $sw_id — $(elem["auxiliary"]) $(elem["original"]) → $side")
    end
end
````

> **The interpretationL** A busbar was actually split **if and only
> if its coupler is open**, and at least one network element is connected to each part of the split busbar. An element switch being open only tells you which side that
> element chose — or, if *both* switches of a couple are open, that the element was
> dropped from the network entirely.

!!! warning "Always threshold, never compare to zero"
    Even declared-binary variables come back as `0.9999999` or `3.2e-9`.
    Use `sw["status"] < 0.01`, never `sw["status"] == 0` — the latter silently never fires.

## 4.4 Stage 3 — check AC feasibility, and price the topology

LPAC is an **approximation**, not a relaxation. Its objective is neither an upper nor a
lower bound of the full AC formulation, and its topology carries no feasibility guarantee.

The check settles both questions at once: freeze the topology into a fixed network,
removes all the switches, and solve an ordinary AC/DC OPF on it.

!!! warning "The third argument is mutated in place"
    `prepare_AC_feasibility_check_*` mutates its third argument and its return value is
    not meaningful. Always pass a `deepcopy`.

````@example webinar_busbar_splitting
data_fc = deepcopy(data_bus)          ## ← MUTATED by the call below

_PMTP.prepare_AC_feasibility_check_AC_busbars(
    result_bus_lpac,   ## the result whose topology you want to test
    data_bus,          ## the split network it came from
    data_fc,           ## mutated in place → becomes the fixed-topology network
    switch_couples,    ## from stage 1
    extremes_ZIL,      ## from stage 1
    data,              ## the original, unsplit network
)

result_fc = _PMACDC.solve_acdcopf(data_fc, ACPPowerModel, ipopt; setting = s)

saving_ac = 100 * (result_opf["objective"] - result_bus_ac["objective"]) / result_opf["objective"]
saving_lpac = 100 * (result_opf["objective"] - result_fc["objective"]) / result_opf["objective"]

println("termination feasibility check: ", result_fc["termination_status"])
println("---")
println("baseline AC OPF:    ", result_opf["objective"], " USD/h")
println("after BuS AC (MINLP):   ", result_bus_ac["objective"],  " USD/h")
println("after BuS LPAC (MIQCP):   ", result_fc["objective"],  " USD/h")
println("saving AC:      ", round(saving_ac, digits = 2), " %")
println("saving LPAC:      ", round(saving_lpac, digits = 2), " %")
````

### Interpreting the outcome

| Outcome | Meaning |
|---|---|
| Converged, objective **below** baseline | AC-feasible and beneficial. Use it. |
| Converged, objective **above** baseline | AC-feasible but useless — the apparent saving was formulation error. |
| Infeasible | Not physically realizable. Discard it. |


Across the published cases, LPAC-BuS topologies produced no infeasible outcomes — but
that is empirical, not a guarantee.

!!! danger "The comparison trap"
    LPAC-BuS reports **181.909 USD/h** against an AC-OPF baseline of **194.139 USD/h**.
    That is **not** a 6.3 % saving. The LPAC-*OPF* of the same untouched network already
    reports **183.924 USD/h**, so most of that gap is *formulation difference*, not topology
    benefit. Only ever compare like with like: AC-OPF baseline vs AC-OPF on the
    optimized topology. This is the single most common way people overstate
    topology-optimization results. One can also compare LPAC-BuS vs LPAC-OPF, but that is a comparison of two approximations, not a physical AC reality check.

````@example webinar_busbar_splitting
# Demonstrating the trap explicitly
result_opf_lpac = _PMACDC.solve_acdcopf(data, LPACCPowerModel, ipopt; setting = s)

println("AC-OPF   (baseline, untouched network): ", result_opf["objective"])
println("LPAC-OPF (baseline, untouched network): ", result_opf_lpac["objective"], "  ← already lower!")
println("LPAC-BuS (optimized topology):          ", result_bus_lpac["objective"])
println("AC-OPF   (optimized topology, via FC):  ", result_fc["objective"], "  ← the only honest comparison")
````

## 4.5 Exercise

> Split busbar **3** instead of 2. Does the coupler open? Is the resulting topology
> AC-feasible, and is it better or worse than splitting busbar 2?

````@example webinar_busbar_splitting
bus_try = 3

data_try, sw_try, ext_try = _PMTP.AC_busbars_split(data, bus_try)
res_try = _PMTP.run_acdc_BuS_AC(data_try, LPACCPowerModel, scip)

fc_try = deepcopy(data_try)
_PMTP.prepare_AC_feasibility_check_AC_busbars(res_try, data_try, fc_try, sw_try, ext_try, data)
result_try = _PMACDC.solve_acdcopf(fc_try, ACPPowerModel, ipopt; setting = s)

println("coupler status: ", res_try["solution"]["switch"]["1"]["status"])
println("objective:      ", result_try["objective"], " vs baseline ", result_opf["objective"])
````

---
# 5. Tutorial II — Identifying relevant areas for topology optimization
To be completed

---
# 6. Next steps

---
# 7. Limitations, roadmap, Q&A *(5 min)*

- **Double-busbar configurations only.** Breaker-and-a-half and double-bus-double-breaker
  are not modelled.
- **All switching units treated identically.** Real substations distinguish circuit
  breakers, disconnectors and load-break switches, with different capabilities. The
  model gives them one rating and one behaviour.
- **No N-1 security constraints.** The optimized topology is not tested against
  contingencies — and splitting a busbar *reduces* redundancy. This is a material
  limitation for operational use, and the first thing a TSO will ask about.
- **No protection coordination.** The model tells you a topology is cheaper. It does not
  tell you it is safe to operate. That verification is a separate, planning-stage exercise.
- **Deterministic operating point only.** One snapshot, one forecast. Real renewable
  injections are uncertain. The extension to day-ahead topology optimization under RES
  uncertainty is published — [doi:10.1016/j.ijepes.2025.111527](https://doi.org/10.1016/j.ijepes.2025.111527)
  — and that code is planned for the repository.
- **Ranking, not enumeration.** See Section 6.


---
# Appendix — Bundled test cases and further reading

Test networks live in `test/data_sources/`.

| File | Description |
|---|---|
| `case5_acdc.m` | 5 AC buses, 3 DC buses, 3 converters — the worked example |
| `case39_acdc.m` | 39 AC buses, 10 DC buses, 10 converters |
| `case67.m` | 67 AC buses, 9 DC buses, 9 converters |
| `pglib_opf_case588_sdet_acdc.m` | 588 AC buses, 7 DC buses |
| `case3120sp_mcdc.m` | 3120 AC buses, 5 DC buses |
| `cigre_b4_dc_grid.m` | CIGRE B4 DC grid |
| AC-only | `case5.m`, `case14.m`, `case24.m`, `case30_ieee.m`, `case57_ieee.m`, `case118_ieee.m`, `case793_goc.m`, `case3375wp_k.m` |

**Further reading**

- **Main reference.** G. Bastianel, M. Vanin, D. Van Hertem, H. Ergun, "Optimal
  transmission switching and busbar splitting in hybrid AC/DC grids," *SEGAN* 46 (2026)
  102182. [doi:10.1016/j.segan.2026.102182](https://doi.org/10.1016/j.segan.2026.102182)
- **Candidate selection.** "Identifying Best Candidates for Busbar Splitting," *EPSR*
  263 (2027). [doi:10.1016/j.epsr.2026.113611](https://doi.org/10.1016/j.epsr.2026.113611)
- **Under uncertainty.** "Day-ahead transmission grid topology optimization considering
  renewable energy sources' uncertainty," *IJEPES* 174 (2026) 111527.
  [doi:10.1016/j.ijepes.2025.111527](https://doi.org/10.1016/j.ijepes.2025.111527)
- **Upstream packages.** [PowerModels.jl](https://github.com/lanl-ansi/PowerModels.jl) ·
  [PowerModelsACDC.jl](https://github.com/Electa-Git/PowerModelsACDC.jl)
- **Package documentation.**
  [electa-git.github.io/PowerModelsTopologicalActions.jl/dev](https://electa-git.github.io/PowerModelsTopologicalActions.jl/dev/)

---

Developed in WP1 of the ETF DIRECTIONS project (FOD Economie, Belgian Government), with
Etch — Energy Transmission Competence Hub, EnergyVille, KU Leuven and Elia Group.
Primary developer: Giacomo Bastianel.

