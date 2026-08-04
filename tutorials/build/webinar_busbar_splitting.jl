# # Grid Topology Optimization in Practice
#
# ## Busbar splitting with `PowerModelsTopologicalActions.jl`
#
# **Webinar — 90 minutes, hands-on**
#
# | | |
# |---|---|
# | **Level** | familiar with OPF; no prior Julia/JuMP experience assumed |
# | **Package** | [`Electa-Git/PowerModelsTopologicalActions.jl`](https://github.com/Electa-Git/PowerModelsTopologicalActions.jl) |
# | **Reference** | Bastianel, Vanin, Van Hertem, Ergun, *Optimal transmission switching and busbar splitting in hybrid AC/DC grids*, SEGAN 46 (2026) 102182 — [doi:10.1016/j.segan.2026.102182](https://doi.org/10.1016/j.segan.2026.102182) |
#
# ### Run sheet
#
# | # | Section | Mode | Time |
# |---|---|---|---|
# | 0 | Pre-flight — *do this before the session* | self-service | — |
# | 1 | Why topology optimization at all | discussion | 10 min |
# | 2 | What busbar splitting actually is | discussion | 10 min |
# | 3 | The optimization model | discussion | 15 min |
# | 4 | Hands-on I — the three-stage workflow | live coding | 20 min |
# | 5 | Hands-on II — choosing a formulation | live coding | 10 min |
# | 6 | Hands-on III — which busbar to split | live coding | 10 min |
# | 7 | The DC side and combined AC/DC splitting | live coding | 5 min |
# | 8 | Gotchas that fail silently | discussion | 5 min |
# | 9 | Limitations, roadmap, Q&A | discussion | 5 min |

# ---
# # 0. Pre-flight — do this *before* the session
#
# Participants who arrive with a working environment get four times as much out of
# the hour. Send this out a week ahead.
#
# **Requirements**
#
# - Julia ≥ 1.10 (plus [IJulia](https://github.com/JuliaLang/IJulia.jl) for the notebook)
# - A solver stack matching the formulation (see Section 5). For the hands-on you need
#   at least one MIQCP-capable solver — Gurobi is what the package is developed and
#   validated against, and academic licences are free.
#
# The package is not in the Julia General registry yet, hence the URL form:
#
# ```julia
# using Pkg
# Pkg.add(url = "https://github.com/Electa-Git/PowerModelsTopologicalActions.jl")
# Pkg.add(["PowerModels", "PowerModelsACDC", "JuMP", "Ipopt", "Juniper", "Gurobi"])
# ```
#
# ### The canary test
#
# If the next block prints **194.139**, you are ready. That number is the baseline for
# everything we do today — keep it somewhere visible.

using PowerModels;                   const _PM     = PowerModels
using PowerModelsACDC;               const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActions; const _PMTP   = PowerModelsTopologicalActions
using JuMP, Ipopt, Gurobi, Juniper

# ← point this at your local clone, so the bundled test cases are on disk
const PMTA_DIR = dirname(dirname(pathof(PowerModelsTopologicalActions)))

# --- solvers: the pairing matters, see Section 5 ---------------------------
gurobi  = JuMP.optimizer_with_attributes(Gurobi.Optimizer, "MIPGap" => 1e-4)
ipopt   = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer,
              "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

# `process_additional_data!` is **not optional** — it builds the DC-grid structures
# that every downstream function expects.

file = joinpath(PMTA_DIR, "test", "data_sources", "case5_acdc.m")
data = _PM.parse_file(file)
_PMACDC.process_additional_data!(data)

println("AC buses: ", length(data["bus"]),
        " | DC buses: ", length(data["busdc"]),
        " | converters: ", length(data["convdc"]))

# > **Facilitator note**
# >
# > Open a shared channel 48 h before the webinar for setup problems. Solver
# > licensing is the only thing that reliably eats webinar time, and it cannot be
# > fixed live.

# ---
# # 1. Why topology optimization at all *(10 min)*
#
# **The framing.** Transmission grids are the bottleneck of the energy transition. New
# lines are slow, expensive and politically contested, so the question becomes: how much
# more can we get out of the network we already have?
#
# Congestion is managed today mostly by **redispatch** — paying generators to move away
# from their market schedule. It works, and it is expensive: roughly €2.77 bn in Germany
# in 2024, USD 8.33 bn in the US, with the JRC projecting European redispatch and
# congestion-management costs toward hundreds of bn€ per year by 2040 under
# business as usual.
#
# **The alternative lever.** Before you pay anyone, you can change how power flows by
# changing the network's topology. Opening a line, or reconfiguring a substation,
# redistributes flows according to Kirchhoff's laws — at essentially zero marginal cost.
#
# > **Analogy worth using:** redispatch is paying drivers to take a different route.
# > Topology optimization is changing the traffic lights and the direction signs painted
# > on the asphalt. Same road network, different flow pattern, no payment to anyone.
#
# ### Two families of action
#
# | | **Optimal Transmission Switching (OTS)** | **Busbar Splitting (BuS)** |
# |---|---|---|
# | Decision | de-energize a line / DC branch / converter | split a substation into two electrical nodes |
# | Granularity | branch level | node level |
# | Effect | removes a path | creates a new node, increases electrical distance *inside* the substation |
# | Maturity in literature | well established | comparatively unexplored, especially on the DC side |
# | Operator familiarity | high | high in the control room, low in optimization tools |
#
# **Why this package exists.** OTS models are everywhere. BuS models are rare, and none
# before this one handled both actions on **both the AC and the DC side** of a hybrid
# AC/DC grid. That matters as offshore multi-terminal DC grids get built out: they will be
# meshed, and their topology will need optimizing too.
#
# Both actions are increasingly institutionalised: "grid-enhancing technologies" in the
# US, and the CACM (day-ahead/intraday) and ROSC (real-time security coordination)
# methodologies in Europe.

# ---
# # 2. What busbar splitting actually is *(10 min)*
#
# **Start from the substation, not the model.**
#
# A double-busbar substation has two bars joined by a **coupler**. Coupler closed → the
# whole thing behaves as one electrical node. Coupler open → two nodes, physically
# adjacent, electrically distinct, each with its own voltage magnitude and angle.
#
# That is the entire physical idea: *splitting a busbar makes electrically close things
# electrically distant, without moving anything.* Every element that was attached to the
# substation — generators, loads, lines, converters — must then be assigned to one half
# or the other.
#
# ![Busbar splitting](https://raw.githubusercontent.com/Electa-Git/PowerModelsTopologicalActions.jl/main/docs/src/images/BuS_illustration_README.png)
#
# - **Left** — the original busbar `i`, four elements attached.
# - **Centre** — the modelling construct. Busbar `i` is duplicated into `i` and `i′`,
#   joined by a **Zero Impedance Line (ZIL)** acting as the coupler. Each element is
#   detached onto its own **auxiliary bus** and linked to *both* halves through a **pair
#   of switches**. Every switch, plus the coupler, carries a binary variable.
# - **Right** — after optimization the open switches are deleted, leaving the realised
#   topology.
#
# **The one-sentence takeaway:** the model does not choose a substation configuration
# from a catalogue — it decides, *per element*, which side of the bar it sits on, and
# whether the bar is one node or two.
#
# **Where the benefit comes from.** Splitting reroutes flows around the binding element
# inside the substation itself. In the 5-bus example we are about to run, splitting a
# single AC busbar buys ≈ 4 % of total generation cost. In a redispatch context, that is
# 4 % you did not have to pay a generator for.

# ---
# # 3. The optimization model *(15 min)*
#
# ## 3.1 Decision variables
#
# | Variable | Count | Meaning |
# |---|---|---|
# | `z_sw` on each **element switch** | 2 per element attached to a split busbar | element connects to this half (1) or not (0) |
# | `z_sw` on each **coupler (ZIL)** | 1 per split busbar | busbar is one node (1) or split (0) |
# | all standard OPF variables | — | voltages, angles, generation, converter set-points |
#
# Switch count grows as
#
# ```math
# n_{\text{switches}} = \sum_b \left(2 \, n_b\right) + B
# ```
#
# with ``B`` the number of split busbars and ``n_b`` the elements attached to busbar
# ``b``. **This is the single most important number in the whole method** — it is why
# splitting every busbar in a 3000-bus network is not something you do.
#
# ## 3.2 Constraints
#
# Per switch:
#
# | Constraint | Meaning |
# |---|---|
# | `constraint_switch_voltage_on_off_big_M` | closed → both buses share voltage magnitude and angle; open → decoupled |
# | `constraint_switch_power_on_off` | no power flows through an open switch |
# | `constraint_switch_thermal_limit` | a closed switch respects its rating |
#
# Per switch couple:
#
# | Constraint | Meaning |
# |---|---|
# | `constraint_exclusivity_switch` | ``z_f + z_t \leq 1`` — an element sits on at most one half |
# | `constraint_ZIL_switch` | coupler closed (no split) → elements stay on the original half |
# | `constraint_BS_OTS_branch` | both switches open → element disconnected, carries no power |
#
# > **The inequality is the interesting part.** Writing exclusivity as ``\leq 1`` rather
# > than ``= 1`` lets the model leave *both* switches open, i.e. drop the element
# > entirely. That is busbar splitting **with OTS on the affected elements**, in a single
# > formulation. The equality variant `constraint_exclusivity_switch_no_OTS` exists if
# > you want to forbid it, but no shipped problem specification uses it.
#
# ## 3.3 The big-M reformulation
#
# "Equal voltage when the switch is closed" is naturally bilinear:
#
# ```math
# z \cdot \theta_m = z \cdot \theta_i \qquad z \cdot U_m = z \cdot U_i
# ```
#
# which would make even the linear formulations non-convex. The package uses the standard
# big-M form instead:
#
# ```math
# -(1-z)\,M_\theta \;\leq\; \theta_m - \theta_i \;\leq\; (1-z)\,M_\theta
# ```
# ```math
# -(1-z)\,M_U \;\leq\; U_m - U_i \;\leq\; (1-z)\,M_U
# ```
#
# with `M_va = 2π`, `M_vm = 1.0`, `M_dc = 1.0`. These are deliberately conservative and
# **hardcoded** in `src/formdcgrid/{acp,lpac,shared,dcp}.jl`. Tightening them is the
# highest-leverage performance change available if you are fighting solve times — and it
# currently means editing the source.
#
# ## 3.4 The objective
#
# ```math
# \min \; \sum_k c_{1k} \, P^g_k \;+\; \sum_{\text{ZIL}} c_{sw}\,(1 - z_{sw})
# ```
#
# Two things to hear **before** running anything:
#
# 1. **Only the linear generation cost coefficient is used.** `calc_gen_cost` reads
#    `g["cost"][end-1]`. A quadratic term in your case data is silently ignored —
#    consistent with the paper, which zeroes them, but it means a BuS objective is *not*
#    directly comparable to a PowerModelsACDC OPF objective on the same data.
# 2. **Opening a coupler costs 1.0 by default.** Element switches are free. This penalty
#    stops the model splitting busbars gratuitously among equal-cost optima — and on a
#    case with a genuinely small saving it can swamp the benefit, so you see no split at
#    all. If splitting never happens where you expect it, check this first.

# ---
# # 4. Hands-on I — the three-stage workflow *(20 min)*
#
# This is the core of the session. Everything else is a variation on it.
#
# ```
#    ┌─────────────────────┐
#    │   1. PREPARE DATA   │  AC_busbars_split / DC_busbars_split
#    │                     │  → expand the network with auxiliary buses + switches
#    └──────────┬──────────┘
#               │  data_split, switch_couples, extremes_ZIL
#               ▼
#    ┌─────────────────────┐
#    │    2. OPTIMIZE      │  run_acdc_BuS_AC / _DC / _AC_DC
#    │                     │  → switch states as decision variables
#    └──────────┬──────────┘
#               │  result
#               ▼
#    ┌─────────────────────┐
#    │  3. FEASIBILITY     │  prepare_AC_feasibility_check_*  then a plain AC/DC OPF
#    │      CHECK          │  → verifies AND prices the topology
#    └─────────────────────┘
# ```
#
# ## 4.1 Always solve the baseline first
#
# Two reasons: it is the reference any topological action must beat, and it tells you
# immediately whether the case itself is feasible. Debugging a BuS model on an infeasible
# case is a bad afternoon.

result_opf = _PMACDC.solve_acdcopf(data, ACPPowerModel, ipopt; setting = s)

println("termination: ", result_opf["termination_status"])   ## LOCALLY_SOLVED
println("objective:   ", result_opf["objective"], " USD/h")  ## 194.139

# ## 4.2 Stage 1 — prepare the network
#
# Three return values, and **two of them are needed again in stage 3**:
#
# - `data_bus` — a *new* dictionary (the input is not mutated) with busbar 2 duplicated,
#   its elements moved onto auxiliary buses, and switches inserted.
# - `switch_couples` — the switch pairs whose exclusivity constraint decides where each
#   element lands.
# - `extremes_ZIL` — maps each split busbar to the indices of its two halves.
#
# To split several busbars at once, pass a vector:
# `_PMTP.AC_busbars_split(data, [1, 2, 3, 4, 5])`.

bus_to_split = 2

data_bus, switch_couples, extremes_ZIL = _PMTP.AC_busbars_split(data, bus_to_split)

# defend yourself — see Section 7 for why this assertion exists
@assert !isempty(data_bus["switch_couples"])

println("buses before: ", length(data["bus"]), " → after: ", length(data_bus["bus"]))
println("switches created: ", length(data_bus["switch"]))
println("switch couples:   ", length(switch_couples))

# ### Pause and inspect the data model
#
# Worth two minutes live — it demystifies everything that follows.
#
# **Before** `AC_busbars_split(data, 2)`:
#
# ```
#         gen 1     load 1
#            │         │
#     ───────┴────┬────┴──┬────      busbar 2
#                 │       │
#           branch 3   branch 5
# ```
#
# **After:**
#
# ```
#      gen 1        load 1      branch 3     branch 5
#         │            │            │            │
#     aux bus 9    aux bus 10   aux bus 11   aux bus 12
#        ╱ ╲          ╱ ╲          ╱ ╲          ╱ ╲
#     sw4   sw5    sw6   sw7    sw8   sw9   sw10  sw11
#      │     │      │     │      │     │      │     │
#  ────┴─────┼──────┴─────┼──────┴─────┼──────┴─────┼────────      busbar 2
#            │            │            │            │      │
#            │            │            │            │      └───┐
#            │            │            │            │          ╱  sw1 (ZIL coupler)
#            │            │            │            │      ┌───┘
#            │            │            │            │      │
#    ────────┴────────────┴────────────┴────────────┴──────┴─      busbar 2'
# ```

# the original busbar and its second half
@show data_bus["bus"]["2"]["split"]          ## true  — nominated for splitting
@show data_bus["bus"]["6"]["ZIL"]            ## true  — the second half
@show data_bus["bus"]["6"]["bus_split"]      ## 2     — derives from busbar 2

# an auxiliary bus hosting a detached element
@show data_bus["bus"]["9"]["auxiliary_bus"]  ## true
@show data_bus["bus"]["9"]["auxiliary"]      ## "gen"
@show data_bus["bus"]["9"]["original"]       ## 1  — which generator

# the two kinds of switch
@show data_bus["switch"]["1"]["ZIL"]         ## true  — the coupler, index 1
@show data_bus["switch"]["4"]["auxiliary"]   ## "gen" — an element switch
@show switch_couples["4"];

# > **The discriminator used throughout the source** is the presence or absence of the
# > `"auxiliary"` key: **couplers lack it, element switches have it.**
# > `calc_ac_switch_cost` charges only switches without it; `compute_couples_of_switches`
# > pairs only switches with it.

# ## 4.3 Stage 2 — optimize
#
# Two formulations, same problem specification. We look at *why* in Section 5; for now,
# run both and watch the clock.

# exact MINLP — slow, exact (local optimum)
@time result_bus_ac = _PMTP.run_acdc_BuS_AC(data_bus, ACPPowerModel, juniper)

# LPAC approximation — far faster, and what you will actually use
@time result_bus_lpac = _PMTP.run_acdc_BuS_AC(data_bus, LPACCPowerModel, gurobi)

println("AC-BuS   objective: ", result_bus_ac["objective"])
println("LPAC-BuS objective: ", result_bus_lpac["objective"],
        "  ← not comparable to the AC baseline!")

# ### Reading the answer

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

# > **The interpretation rule — say it twice.** A busbar was actually split **if and only
# > if its coupler is open**. An element switch being open only tells you which side that
# > element chose — or, if *both* switches of a couple are open, that the element was
# > dropped from the network entirely.
#
# > **Always threshold, never compare to zero**
# >
# > Even declared-binary variables come back as `0.9999999` or `3.2e-9`.
# > Use `sw["status"] < 0.01`, never `sw["status"] == 0` — the latter silently never fires.
#
# DC switch results live under `result["solution"]["dcswitch"]`, with the same `status` key.

# ## 4.4 Stage 3 — check AC feasibility, and price the topology
#
# LPAC is an **approximation**, not a relaxation. Its objective is neither an upper nor a
# lower bound, and its topology carries no feasibility guarantee.
#
# The check settles both questions at once: freeze the topology into a fixed network,
# strip out all the switch scaffolding, and solve an ordinary AC/DC OPF on it.
#
# > **The third argument is mutated in place**
# >
# > `prepare_AC_feasibility_check_*` mutates its third argument and its return value is
# > not meaningful. Always pass a `deepcopy`.

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

saving = 100 * (result_opf["objective"] - result_fc["objective"]) / result_opf["objective"]

println("termination: ", result_fc["termination_status"])
println("baseline:    ", result_opf["objective"], " USD/h")
println("after BuS:   ", result_fc["objective"],  " USD/h")
println("saving:      ", round(saving, digits = 2), " %")

# The function prints every reconnection it makes. That is deliberate — when a check
# fails unexpectedly, the log is how you find out which element ended up where.
#
# ### Interpreting the outcome
#
# | Outcome | Meaning |
# |---|---|
# | Converged, objective **below** baseline | AC-feasible and beneficial. Use it. |
# | Converged, objective **above** baseline | AC-feasible but useless — the apparent saving was formulation error. |
# | Infeasible | Not physically realizable. Discard it. |
#
# Across the published cases, LPAC-BuS topologies produced no infeasible outcomes — but
# that is empirical, not a guarantee.
#
# > **The comparison trap — spend a minute here**
# >
# > LPAC-BuS reports **181.909 USD/h** against an AC-OPF baseline of **194.139 USD/h**.
# > That is **not** a 6.3 % saving. The LPAC-*OPF* of the same untouched network already
# > reports **183.924 USD/h**, so most of that gap is *formulation error*, not topology
# > benefit. Only ever compare like with like: AC-OPF baseline vs AC-OPF on the
# > optimized topology. This is the single most common way people overstate
# > topology-optimization results.

# Demonstrating the trap explicitly
result_opf_lpac = _PMACDC.solve_acdcopf(data, LPACCPowerModel, ipopt; setting = s)

println("AC-OPF   (baseline, untouched network): ", result_opf["objective"])
println("LPAC-OPF (baseline, untouched network): ", result_opf_lpac["objective"], "  ← already lower!")
println("LPAC-BuS (optimized topology):          ", result_bus_lpac["objective"])
println("AC-OPF   (optimized topology, via FC):  ", result_fc["objective"], "  ← the only honest comparison")

# ## 4.5 Exercise *(3 minutes)*
#
# > Split busbar **3** instead of 2. Does the coupler open? Is the resulting topology
# > AC-feasible, and is it better or worse than splitting busbar 2?

bus_try = 3

data_try, sw_try, ext_try = _PMTP.AC_busbars_split(data, bus_try)
res_try = _PMTP.run_acdc_BuS_AC(data_try, LPACCPowerModel, gurobi)

fc_try = deepcopy(data_try)
_PMTP.prepare_AC_feasibility_check_AC_busbars(res_try, data_try, fc_try, sw_try, ext_try, data)
result_try = _PMACDC.solve_acdcopf(fc_try, ACPPowerModel, ipopt; setting = s)

println("coupler status: ", res_try["solution"]["switch"]["1"]["status"])
println("objective:      ", result_try["objective"], " vs baseline ", result_opf["objective"])

# ---
# # 5. Hands-on II — choosing a formulation *(10 min)*
#
# The AC power flow equations are non-convex. Add binaries and you have a MINLP. The
# package offers a ladder of trade-offs.
#
# | Formulation | Class | Solver | Guarantee |
# |---|---|---|---|
# | `ACPPowerModel` — AC polar | MINLP | Juniper + Ipopt + MIP | exact, **local** optimum |
# | `LPACCPowerModel` — LPAC cold start | MIQCP | Gurobi | none — approximation |
# | `SOCWRPowerModel` | MISOCP | Gurobi, Mosek | lower bound |
# | `QCRMPowerModel` | MIQCP | Gurobi | lower bound, tighter than SOC |
# | `DCPPowerModel` | MILP | Gurobi, HiGHS | none — approximation |
#
# **Getting the solver–formulation pairing wrong is the most common source of confusing
# failures:** `juniper` for `ACPPowerModel`, `gurobi` for everything else.
#
# ### Results on `case5_acdc.m`, all AC busbars splittable
#
# | Model | BuS obj. [USD/h] | Time [s] | After feasibility check [USD/h] | Benefit |
# |---|---|---|---|---|
# | AC-OPF baseline | — | 0.014 | — | — |
# | AC-BuS big-M | 184.972 | 232.4 | 183.972 | 5.24 % |
# | LPAC-BuS | 181.909 | 0.45 | 186.349 | 4.01 % |
# | SOC-BuS | 183.763 | 0.30 | 194.139 | none |
# | QC-BuS | 183.761 | 0.33 | 194.139 | none |
#
# ### Scaling, AC busbar split
#
# | Case | AC-BuS [s] | LPAC-BuS [s] | speed-up |
# |---|---|---|---|
# | 39-bus | 30.3 | 0.5 | 61× |
# | 67-bus | 118.1 | 0.5 | 236× |
# | 588-bus | 958.1 | 58.8 | 16× |
# | 3120-bus | 12 560 | 534 | 24× |
#
# At 3120 buses the exact model takes three and a half hours. LPAC takes nine minutes.
#
# ### Three conclusions worth stating explicitly
#
# 1. **LPAC is the default.** Unlike the classical DC approximation it keeps voltage
#    magnitudes and reactive power, so its topologies survive the AC feasibility check —
#    it captures most of the achievable saving at 10–200× the speed.
# 2. **SOC and QC are for bounding, not for generating topologies.** On the tested cases
#    they return the original topology unchanged. Useful for knowing how far from optimal
#    you might be; useless as a search method.
# 3. **Production recipe:** LPAC to search → AC feasibility check to validate and price →
#    ACP to confirm the final candidate if you need the exact number.
#
# Note also that **OTS is implemented for the exact formulation only** — `run_acdcots_*`
# with `LPACCPowerModel` is not supported. The relaxation strategy would transfer; it
# just has not been done.

# Reproduce the formulation comparison: SOC and QC are expected to leave the topology alone
for (name, form) in [("SOC", SOCWRPowerModel), ("QC", QCRMPowerModel)]
    res = _PMTP.run_acdc_BuS_AC(data_bus, form, gurobi)
    println(rpad(name, 5), " coupler status: ", res["solution"]["switch"]["1"]["status"],
            "  objective: ", res["objective"])
end

# ---
# # 6. Hands-on III — which busbar do you split? *(10 min)*
#
# The combinatorics forbid splitting everything, so the practical method from the paper
# is a **screening pass**: run the cheap model once per candidate, check each topology,
# keep the winner.

best = (bus = nothing, obj = result_opf["objective"])

for b in keys(data["bus"])
    bus_id = parse(Int, b)

    data_b, sw, ext = _PMTP.AC_busbars_split(data, bus_id)
    res = _PMTP.run_acdc_BuS_AC(data_b, LPACCPowerModel, gurobi)

    data_b_fc = deepcopy(data_b)
    _PMTP.prepare_AC_feasibility_check_AC_busbars(res, data_b, data_b_fc, sw, ext, data)
    fc = _PMACDC.solve_acdcopf(data_b_fc, ACPPowerModel, ipopt; setting = s)

    println("busbar $bus_id → ", fc["termination_status"], "  obj ", fc["objective"])

    if fc["termination_status"] == LOCALLY_SOLVED && fc["objective"] < best.obj
        global best = (bus = bus_id, obj = fc["objective"])
    end
end

best

# This is *n* cheap solves instead of one intractable one. It scales linearly in the
# number of candidate busbars — fine for 5 or 67 buses, still unpleasant for 3120.
#
# **The better answer is a priori ranking:** screen candidates by structural and
# sensitivity metrics *before* solving anything. That is exactly the subject of the
# follow-up work — G. Bastianel, D. Van Hertem, H. Ergun, L. A. Roald, *Identifying Best
# Candidates for Busbar Splitting*, EPSR 263 (2027),
# [doi:10.1016/j.epsr.2026.113611](https://doi.org/10.1016/j.epsr.2026.113611).
#
# It is the difference between a research demo and something that fits inside an
# operational time window.

# ---
# # 7. The DC side and combined AC/DC splitting *(5 min)*
#
# Same three stages, different entry points. This DC capability is the package's novelty
# — and it is where it will matter most, since offshore multi-terminal DC grids are being
# planned now and nobody is currently optimizing their substation topologies.

# --- DC side only ---------------------------------------------------------
data_dc, dcswitch_couples, extremes_dc = _PMTP.DC_busbars_split(data, 2)
result_dc = _PMTP.run_acdc_BuS_DC(data_dc, LPACCPowerModel, gurobi)

println("DC coupler status: ", result_dc["solution"]["dcswitch"]["1"]["status"])

# > **Order matters — and getting it wrong fails *silently***
# >
# > `AC_busbars_split` must come **first**. Called second, it wipes the DC couples. The
# > model then iterates an empty `:dcswitch_couples`, never posts
# > `constraint_exclusivity_dc_switch`, `constraint_ZIL_dc_switch` or
# > `constraint_BS_OTS_dcbranch`, solves happily, and returns an **over-optimistic,
# > meaningless** answer — because DC elements are free to connect to both halves of a
# > busbar at once. There is no error and no warning. Assert.

# --- both sides: AC FIRST, then DC -----------------------------------------
data_both, sw_ac, ext_ac = _PMTP.AC_busbars_split(data, 2)
data_both, sw_dc, ext_dc = _PMTP.DC_busbars_split(data_both, 2)

@assert !isempty(data_both["switch_couples"])
@assert !isempty(data_both["dcswitch_couples"])

result_both = _PMTP.run_acdc_BuS_AC_DC(data_both, LPACCPowerModel, gurobi)

println("AC coupler: ", result_both["solution"]["switch"]["1"]["status"])
println("DC coupler: ", result_both["solution"]["dcswitch"]["1"]["status"])

# ---
# # 8. Gotchas that fail silently *(5 min)*
#
# Every item below produces a plausible number that is wrong. Worth keeping open whenever
# you use the package.
#
# | Gotcha | Symptom | Fix |
# |---|---|---|
# | Binaries returned as floats | `status == 0` never fires | threshold at `< 0.01` |
# | Coupler penalty (`cost = 1.0`) | no split where you expect one | lower the penalty on ZIL switches, re-run |
# | Quadratic gen costs ignored | objective ≠ PowerModelsACDC OPF objective | linearize cost curves, or compare only within this package |
# | Switch ratings default to 100 p.u. | ratings never bind | set `psw`, `qsw`, `thermal_rating` explicitly after preparation |
# | `AC_busbars_split` owns `data["switch"]` | pre-existing switches overwritten; `KeyError` if the key is absent | start from a case with no switches |
# | Some split functions mutate | original data silently modified | `AC_busbars_split` / `DC_busbars_split` **copy**; `AC_busbar_split_AC_grid` and `AC_busbars_split_ordered` **mutate** — `deepcopy` first |
# | Empty `switch_couples` | constraints silently not posted, model still solves | assert non-empty before solving |
# | `runtests.jl` is empty | `Pkg.test()` passes trivially | validate against published numbers: **194.139** baseline, **184.972** AC-BuS |
#
# ### If the MINLP will not converge
#
# Expected above roughly 100 buses with everything switchable. In order of effectiveness:
#
# 1. Switch to `LPACCPowerModel` + AC feasibility check.
# 2. Split one busbar instead of all of them.
# 3. Restrict the switchable element set.
# 4. Warm-start the ACP solve from an LPAC solution via `prepare_starting_value_dict`.
# 5. Tighten the big-M constants.
#
# ### Memory blow-up on large cases
#
# Splitting all busbars in a 3120-bus network generates tens of thousands of switches.
# Split one busbar at a time.
#
# Making the "ratings never bind" gotcha concrete — switches default to 100 p.u., which
# on a 100 MVA base is 10 GVA:

for (id, sw) in data_bus["switch"]
    sw["thermal_rating"] = 3.0
    sw["psw"] = 3.0
    sw["qsw"] = 3.0
end

result_bounded = _PMTP.run_acdc_BuS_AC(data_bus, LPACCPowerModel, gurobi)
println("with realistic switch ratings — coupler: ",
        result_bounded["solution"]["switch"]["1"]["status"],
        "  objective: ", result_bounded["objective"])

# ---
# # 9. Limitations, roadmap, Q&A *(5 min)*
#
# Be straight about these. They are also the research agenda, and they generate the best
# questions.
#
# - **Double-busbar configurations only.** Breaker-and-a-half and double-bus-double-breaker
#   are not modelled.
# - **All switching units treated identically.** Real substations distinguish circuit
#   breakers, disconnectors and load-break switches, with different capabilities. The
#   model gives them one rating and one behaviour.
# - **No N-1 security constraints.** The optimized topology is not tested against
#   contingencies — and splitting a busbar *reduces* redundancy. This is a material
#   limitation for operational use, and the first thing a TSO will ask about.
# - **No protection coordination.** The model tells you a topology is cheaper. It does not
#   tell you it is safe to operate. That verification is a separate, planning-stage exercise.
# - **Deterministic operating point only.** One snapshot, one forecast. Real renewable
#   injections are uncertain. The extension to day-ahead topology optimization under RES
#   uncertainty is published — [doi:10.1016/j.ijepes.2025.111527](https://doi.org/10.1016/j.ijepes.2025.111527)
#   — and that code is planned for the repository.
# - **Ranking, not enumeration.** See Section 6.
#
# ### Contributions explicitly wanted
#
# Docstrings on the exported problem specifications · configurable big-M values · an API
# for restricting the switchable element set · N-1 constraints · and — genuinely useful
# and currently missing — **a plotting tool that shows the optimized substation topology
# and switch states**. If your audience has one frontend-inclined person, this is the ask.
#
# > **Suggested closing question:** *in your control room, which is the harder sell — the
# > model's recommendation, or the absence of an N-1 check behind it?*

# ---
# # Appendix — Bundled test cases and further reading
#
# Test networks live in `test/data_sources/`.
#
# | File | Description |
# |---|---|
# | `case5_acdc.m` | 5 AC buses, 3 DC buses, 3 converters — the worked example |
# | `case39_acdc.m` | 39 AC buses, 10 DC buses, 10 converters |
# | `case67.m` | 67 AC buses, 9 DC buses, 9 converters |
# | `pglib_opf_case588_sdet_acdc.m` | 588 AC buses, 7 DC buses |
# | `case3120sp_mcdc.m` | 3120 AC buses, 5 DC buses |
# | `cigre_b4_dc_grid.m` | CIGRE B4 DC grid |
# | AC-only | `case5.m`, `case14.m`, `case24.m`, `case30_ieee.m`, `case57_ieee.m`, `case118_ieee.m`, `case793_goc.m`, `case3375wp_k.m` |
#
# **Further reading**
#
# - **Main reference.** G. Bastianel, M. Vanin, D. Van Hertem, H. Ergun, "Optimal
#   transmission switching and busbar splitting in hybrid AC/DC grids," *SEGAN* 46 (2026)
#   102182. [doi:10.1016/j.segan.2026.102182](https://doi.org/10.1016/j.segan.2026.102182)
# - **Candidate selection.** "Identifying Best Candidates for Busbar Splitting," *EPSR*
#   263 (2027). [doi:10.1016/j.epsr.2026.113611](https://doi.org/10.1016/j.epsr.2026.113611)
# - **Under uncertainty.** "Day-ahead transmission grid topology optimization considering
#   renewable energy sources' uncertainty," *IJEPES* 174 (2026) 111527.
#   [doi:10.1016/j.ijepes.2025.111527](https://doi.org/10.1016/j.ijepes.2025.111527)
# - **Upstream packages.** [PowerModels.jl](https://github.com/lanl-ansi/PowerModels.jl) ·
#   [PowerModelsACDC.jl](https://github.com/Electa-Git/PowerModelsACDC.jl)
# - **Package documentation.**
#   [electa-git.github.io/PowerModelsTopologicalActions.jl/dev](https://electa-git.github.io/PowerModelsTopologicalActions.jl/dev/)
#
# ---
#
# Developed in WP1 of the ETF DIRECTIONS project (FOD Economie, Belgian Government), with
# Etch — Energy Transmission Competence Hub, EnergyVille, KU Leuven and Elia Group.
# Primary developer: Giacomo Bastianel.
