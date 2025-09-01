isdefined(Base, :__precompile__) && __precompile__()

"""
    PowerModelsTopologicalActionsII

A Julia package for modeling AC and DC topological actions in power systems, including 
Optimal Transmission Switching (OTS) and Busbar Splitting (BS) for steady-state power 
network optimization.

This package extends PowerModels.jl and PowerModelsACDC.jl to provide comprehensive
support for both AC and DC grid topological actions, including:

## Core Problem Specifications
- AC Optimal Transmission Switching (OTS)
- DC Optimal Transmission Switching (OTS) 
- AC/DC Optimal Transmission Switching
- AC Busbar Splitting (BS)
- DC Busbar Splitting (BS)
- AC/DC Busbar Splitting

## Mathematical Formulations
- AC formulations (polar coordinates)
- SOC Relaxation (W-space)
- QC Relaxation (W+L-space) 
- LPAC Approximation (Cold Start)
- DC approximations

## Key Features
- Mixed-integer optimization formulations for topological actions
- Support for hybrid AC/DC power systems
- Multiple relaxation schemes for computational efficiency
- Comprehensive constraint and variable libraries
- Integration with JuMP optimization framework

## Usage
```julia
using PowerModelsTopologicalActionsII
using PowerModels, PowerModelsACDC
using Ipopt, Juniper

# Load network data
data = PowerModels.parse_file("case.m")
PowerModelsACDC.process_additional_data!(data)

# Solve AC OTS problem
result = run_acdcots_AC(data, ACPPowerModel, optimizer)
```
"""
module PowerModelsTopologicalActionsII

# import Compat

import Memento
import PowerModelsACDC
const _PMACDC = PowerModelsACDC
import PowerModels
const _PM = PowerModels
import InfrastructureModels
const _IM = InfrastructureModels

import JuMP

#import Plots
#import PlotlyJS
# Create our module level logger (this will get precompiled)
const _LOGGER = Memento.getlogger(@__MODULE__)

# Register the module level logger at runtime so that folks can access the logger via `getlogger(PowerModels)`
# NOTE: If this line is not included then the precompiled `_PM._LOGGER` won't be registered at runtime.

__init__() = Memento.register(_LOGGER)

include("core/constraint.jl")
include("core/constraint_template.jl")
include("core/variableots.jl")
include("core/variable.jl")
include("core/busbar_splitting.jl")
include("core/busbar_splitting_fixed.jl")
include("core/base.jl")
include("core/objective.jl")
include("core/relaxation_scheme.jl")
include("core/feasibility_check.jl")

include("prob/acdcots_AC.jl")
include("prob/acdcots_DC.jl")
include("prob/acdcots_AC_DC.jl")
include("prob/acdcsw_opf.jl")
include("prob/acdcsw_AC.jl")
include("prob/acdcsw_AC_big_M.jl")
include("prob/acdcsw_AC_fixed.jl")
include("prob/acdcsw_AC_current.jl")
include("prob/acdcsw_DC.jl")
include("prob/acdcsw_DC_big_M.jl")
include("prob/acdcsw_DC_fixed.jl")
include("prob/acdcsw_DC_current.jl")
include("prob/acdcsw_AC_DC.jl")
include("prob/acdcsw_AC_DC_big_M.jl")
include("prob/acdcsw_AC_DC_fixed.jl")
include("prob/acdcsw_AC_DC_current.jl")
include("prob/ots_AC.jl")

include("formconv/acp.jl")
include("formconv/wr.jl")
include("formconv/wrm.jl")
include("formconv/dcp.jl")
include("formconv/shared.jl")
include("formconv/lpac.jl")

include("formdcgrid/acp.jl")
include("formdcgrid/wr.jl")
include("formdcgrid/dcp.jl")
include("formdcgrid/shared.jl")
include("formdcgrid/bf.jl")
include("formdcgrid/lpac.jl")



end # module PowerModelsTopologicalActionsII
