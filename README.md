# PowerModelsTopologicalActions.jl

PowerModelsTopologicalActions.jl is a Julia/JuMP package to model AC and DC topological actions (Optimal Transmission Switching (OTS) and Busbar Splitting (BS)) for Steady-State Power Network Optimization. It is based on PowerModels.jl and PowerModelsACDC.jl. This consists of the first package being able to perform both OTS and BS on either part of AC/DC grids. 
While the OTS is a well-established problem in the literature, the BS is still not explored widely, especially for the DC part of AC/DC grids.

## Quick Start

```julia
# Install the package
using Pkg
Pkg.add("PowerModelsTopologicalActionsII")

# Load required packages
using PowerModelsTopologicalActionsII
using PowerModels, PowerModelsACDC
using Ipopt, Juniper

# Setup solvers
ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, 
    "nl_solver" => ipopt, 
    "mip_solver" => ipopt,
    "time_limit" => 3600)

# Load network data
data = PowerModels.parse_file("case5_acdc.m")
PowerModelsACDC.process_additional_data!(data)

# Solve different problem types
result_ac_ots = run_acdcots_AC(data, ACPPowerModel, juniper)
result_dc_ots = run_acdcots_DC(data, ACPPowerModel, juniper)
result_combined = run_acdcots_AC_DC(data, ACPPowerModel, juniper)
```

**Core Problem Specifications**
* AC Optimal Transmission Switching
* DC Optimal Transmission Switching
* AC/DC Optimal Transmission Switching
* AC Busbar Splitting
* DC Busbar Splitting
* AC/DC Busbar Splitting

**Core network formulations**
* AC (polar coordinates)
* SOC Relaxation (W-space) 
* QC Relaxation (W+L-space)
* LPAC Approximation (Cold Start)

## Problem Types

### Optimal Transmission Switching (OTS)

Optimal Transmission Switching allows transmission lines to be temporarily taken out of service to improve system economics and/or security.

#### AC Transmission Switching
```julia
# Switch only AC transmission lines
result = run_acdcots_AC(data, ACPPowerModel, juniper)

# Check which AC lines were switched off
for (i, branch) in result["solution"]["branch"]
    if branch["br_status"] < 0.5
        println("AC Branch $i was switched OFF")
    end
end
```

#### DC Transmission Switching  
```julia
# Switch only DC transmission lines and converters
result = run_acdcots_DC(data, ACPPowerModel, juniper)

# Check DC switching decisions
for (i, branchdc) in result["solution"]["branchdc"]
    if haskey(branchdc, "br_status") && branchdc["br_status"] < 0.5
        println("DC Branch $i was switched OFF") 
    end
end
```

#### Combined AC/DC Switching
```julia
# Allow switching of both AC and DC elements
result = run_acdcots_AC_DC(data, ACPPowerModel, juniper)
```

### Busbar Splitting

Busbar splitting divides a single bus into multiple bus sections with controllable switches between them. This provides operational flexibility and can improve system security.

#### AC Busbar Splitting
```julia
# Requires data with switch definitions
result = run_acdcsw_AC(data_with_switches, ACPPowerModel, juniper)

# Check switch positions
for (i, switch) in result["solution"]["switch"]
    if switch["status"] > 0.5
        println("Switch $i is CLOSED")
    else
        println("Switch $i is OPEN")
    end
end
```

## Mathematical Formulations

The package supports multiple mathematical formulations for different computational trade-offs:

* **ACPPowerModel**: Full AC power flow (nonlinear, exact)
* **SOCWRPowerModel**: Second-order cone relaxation (convex approximation)
* **QCWRPowerModel**: Quadratic convex relaxation
* **LPACCPowerModel**: Linear approximation (fast, approximate)
* **DCPPowerModel**: DC power flow approximation

Example with different formulations:
```julia
# Exact AC formulation (slower, exact)
result_acp = run_acdcots_AC(data, ACPPowerModel, juniper)

# Convex relaxation (faster, approximate)  
result_soc = run_acdcots_AC(data, SOCWRPowerModel, mosek)

# Linear approximation (fastest, approximate)
result_lpac = run_acdcots_AC(data, LPACCPowerModel, gurobi)
```

## Data Requirements

### Basic AC/DC Networks
The package works with standard MATPOWER case files extended with DC grid data:
```julia
data = PowerModels.parse_file("case.m")
PowerModelsACDC.process_additional_data!(data)
```

### Networks with Switches
For busbar splitting problems, the network data must include switch definitions:
```julia
data["switch"] = Dict(
    "1" => Dict(
        "f_bus" => 1,
        "t_bus" => 2, 
        "status" => 1,
        "switching_cost" => 100.0
    )
)
```

## API Reference

### Main Problem Functions

| Function | Description |
|----------|-------------|
| `run_acdcots_AC(data, model, solver)` | AC-only transmission switching |
| `run_acdcots_DC(data, model, solver)` | DC-only transmission switching |
| `run_acdcots_AC_DC(data, model, solver)` | Combined AC/DC transmission switching |
| `run_acdcsw_AC(data, model, solver)` | AC busbar splitting |
| `run_acdcsw_DC(data, model, solver)` | DC busbar splitting |
| `run_acdcsw_AC_DC(data, model, solver)` | Combined AC/DC busbar splitting |

### Formulation Support

| Model Type | AC OTS | DC OTS | AC BS | DC BS | Combined |
|------------|--------|--------|-------|-------|----------|
| ACPPowerModel | ✓ | ✓ | ✓ | ✓ | ✓ |
| SOCWRPowerModel | ✓ | ✓ | ✓ | ✓ | ✓ |
| QCWRPowerModel | ✓ | ✓ | ✓ | ✓ | ✓ |
| LPACCPowerModel | ✓ | ✓ | ✓ | ✓ | ✓ |
| DCPPowerModel | ✓ | - | ✓ | - | - |

### Solver Requirements

- **AC problems**: Nonlinear solver (e.g., Ipopt)
- **Mixed-integer problems**: MINLP solver (e.g., Juniper, Alpine)
- **Relaxations**: Conic solvers (e.g., Mosek) or linear solvers (e.g., Gurobi)

## Advanced Usage

### Custom Objective Functions
```julia
# Include switching costs in the objective
settings = Dict("output" => Dict("branch_flows" => true))
result = run_acdcots_AC(data, ACPPowerModel, juniper; setting = settings)
```

### Multi-period Problems
```julia
# For time-series analysis
multinetwork_data = PowerModels.replicate(data, 24)  # 24 hours
result = run_acdcots_AC(multinetwork_data, ACPPowerModel, juniper)
```

### Feasibility Checking
The package includes utilities for checking the feasibility of switching decisions:
```julia
# Check if a switching solution is feasible
feasible = check_switching_feasibility(data, switching_solution)
```

## Performance Tips

1. **Use relaxations for large systems**: SOC relaxations provide good approximations with better computational performance
2. **Warm-start with OPF solutions**: Solve a standard OPF first to get good initial points
3. **Limit switching options**: Restrict which elements can be switched to reduce problem complexity
4. **Use time limits**: Set appropriate time limits for mixed-integer solvers

# Acknowledgements

This code has been developed as part of WP1 of the ETF DIRECTIONS project from the FOD Economie of the Belgian Government. The primary developer is Giacomo Bastianel (@GiacomoBastianel) with support from the following contributors:

Marta Vanin (@MartaVanin)


# Citing PowerModelsTopologicalActions.jl

```bibtex
@misc{PowerModelsTopologicalActions.jl,
  title = {PowerModelsTopologicalActions.jl: A Julia Package for AC/DC Optimal Transmission Switching and Busbar Splitting},
  author = {Bastianel, Giacomo and Vanin, Marta},
  year = {2024},
  url = {https://github.com/Electa-Git/PowerModelsTopologicalActions.jl}
}
```



