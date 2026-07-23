# Data model

This page documents what the busbar-splitting preparation functions do to a PowerModels
network dictionary. You need it when debugging a model that solves but gives a nonsensical
topology, when fixing elements to one half of a busbar, or when writing your own
post-processing.

## Starting point

The input is a standard PowerModels dictionary extended by
`PowerModelsACDC.process_additional_data!`, so alongside `"bus"`, `"branch"`, `"gen"`,
`"load"` you have `"busdc"`, `"branchdc"`, and `"convdc"`.

Two prerequisites:

- `data["switch"]` must exist. PowerModels creates it when parsing a Matpower file, and the
  preparation functions index into it directly rather than creating it.
- The case must contain **no pre-existing switches**. `AC_busbars_split` assumes it owns the
  `"switch"` dictionary and numbers its own entries from 1.

`data["dcswitch"]` does not need to exist — `DC_busbars_split` creates it, overwriting
anything already there.

## The transformation, by example

Take busbar 2 of `case5_acdc.m`, with a generator, a load, and two branches attached.

**Before:**

```
        gen 1     load 1
           │         │
    ───────┴────┬────┴───────      busbar 2
                │
          branch 3   branch 5
```

**After `AC_busbars_split(data, 2)`:**

```
     gen 1        load 1      branch 3     branch 5
        │            │            │            │
    aux bus 9    aux bus 10   aux bus 11   aux bus 12
       ╱ ╲          ╱ ╲          ╱ ╲          ╱ ╲
    sw4   sw5    sw6   sw7    sw8   sw9   sw10  sw11
     │     │      │     │      │     │      │     │
 ────┴─────┼──────┴─────┼──────┴─────┼──────┴─────┼────
   busbar 2│           ／            │  busbar 2′ │
           └────── sw1 (ZIL) ────────┘
```

Every element now has its own auxiliary bus and a pair of switches, one to each half. The
coupler `sw1` decides whether the two halves are one node or two.

## Buses

Three kinds of bus coexist after the transformation, distinguished by flags.

| Key | Type | Meaning |
|---|---|---|
| `split` | `Bool` | `true` on the original busbar nominated for splitting |
| `ZIL` | `Bool` | `true` on both halves — they are joined by a coupler |
| `bus_split` | `Int` | index of the original busbar this bus derives from |
| `auxiliary_bus` | `Bool` | `true` on buses created to host a detached element |
| `auxiliary` | `String` | element type on this auxiliary bus: `"gen"`, `"load"`, `"branch"`, `"convdc"` |
| `original` | `Int` | index of that element in its own dictionary, **before** the split |

So:

```julia
# the original busbar
data["bus"]["2"]["split"]          # true
data["bus"]["2"]["ZIL"]            # true

# its second half, appended after the highest original bus index
data["bus"]["6"]["split"]          # false
data["bus"]["6"]["ZIL"]            # true
data["bus"]["6"]["bus_split"]      # 2
data["bus"]["6"]["bus_type"]       # 1 (PQ)

# an auxiliary bus hosting generator 1
data["bus"]["9"]["auxiliary_bus"]  # true
data["bus"]["9"]["auxiliary"]      # "gen"
data["bus"]["9"]["original"]       # 1
data["bus"]["9"]["bus_split"]      # 2
data["bus"]["9"]["split"]          # false
```

The `auxiliary` / `original` pair is how the feasibility check knows which element to
reconnect where. Nothing else recovers that mapping.

Element references are rewritten to point at the auxiliary buses: `gen["gen_bus"]`,
`load["load_bus"]`, `branch["f_bus"]` / `["t_bus"]`, `convdc["busac_i"]`. The original index
survives only in `bus[...]["original"]`.

## AC switches

Two kinds, both in `data["switch"]`.

**Busbar coupler (ZIL)** — one per split busbar, created first, so it takes the low indices:

```julia
data["switch"]["1"] = Dict(
    "f_bus"          => 2,      # first half
    "t_bus"          => 6,      # second half
    "bus_split"      => 2,      # busbar being split
    "ZIL"            => true,
    "index"          => 1,
    "psw"            => 100.0,
    "qsw"            => 100.0,
    "thermal_rating" => 100.0,
    "cost"           => 1.0,    # penalty for opening — see below
    "state"          => 1,
    "status"         => 1,
    "source_id"      => ["switch", 1],
)
```

**Element switch** — two per detached element, no `ZIL` key, and free to open:

```julia
data["switch"]["4"] = Dict(
    "f_bus"     => 9,       # auxiliary bus
    "t_bus"     => 2,       # first half of the split busbar
    "bus_split" => 2,
    "auxiliary" => "gen",   # what kind of element
    "original"  => 1,       # which one
    "cost"      => 0.0,     # free
    # ... ratings inherited from switch 1
)
```

The presence or absence of the `"auxiliary"` key is the discriminator used throughout the
code — `calc_ac_switch_cost` charges only switches that lack it, and
`compute_couples_of_switches` pairs only switches that have it.

!!! warning "Default ratings are effectively infinite"
    `psw`, `qsw`, and `thermal_rating` all default to `100.0` p.u., far above anything that
    binds on the bundled cases. If you want switch ratings to constrain the solution — for
    instance to model realistic circuit-breaker capabilities — set them explicitly after
    preparation.

## DC switches

`data["dcswitch"]`, structurally identical but with DC naming and no reactive power:

| AC key | DC key |
|---|---|
| `f_bus` / `t_bus` | `f_busdc` / `t_busdc` |
| `bus_split` | `busdc_split` |
| `psw`, `qsw` | `psw` only |

Multiconductor variants add a `terminal` key to distinguish poles.

## Switch couples

The pairing that makes exclusivity constraints possible, stored both as a return value and
on `data["switch_couples"]` / `data["dcswitch_couples"]`:

```julia
switch_couples["4"] = Dict(
    "f_sw"         => 4,   # switch to the first half
    "t_sw"         => 5,   # switch to the second half
    "bus_split"    => 2,   # which busbar
    "switch_split" => 1,   # index of that busbar's coupler
)
```

One entry per element, keyed by the first switch of the pair. `compute_couples_of_switches`
builds these by matching switches that share `auxiliary`, `original`, and `bus_split` while
differing in `index`; `eliminate_duplicates_couple_of_switches` then drops the mirror-image
duplicates.

The model iterates `ids(pm, :switch_couples)` to post exclusivity, ZIL, and BuS-OTS
constraints. **If this dictionary is empty, those constraints are silently not posted** and
the model becomes meaningless while still solving happily. This is the failure mode behind
the `switch_couples` warning in [Known issues and gotchas](@ref).

## Fixing an element to one half

In a real substation some elements are permanently bolted to one busbar section. Modelling
this removes two binaries and shrinks the search space. There is no API for it, but the
data model supports it directly — delete the couple and fix the unwanted switch:

```julia
data_split, switch_couples, extremes = AC_busbars_split(data, 2)

# force generator 1 onto the first half of busbar 2
for (cid, c) in switch_couples
    sw_f = data_split["switch"]["$(c["f_sw"])"]
    if get(sw_f, "auxiliary", "") == "gen" && sw_f["original"] == 1
        # keep f_sw closed, force t_sw open
        data_split["switch"]["$(c["t_sw"])"]["status"] = 0
        delete!(data_split["switch_couples"], cid)
    end
end
```

Verify the resulting topology carefully — you are working below the level the preparation
functions guarantee.

## Reference: keys added by preparation

| Dictionary | Key | Type | Set on |
|---|---|---|---|
| `bus` | `split` | `Bool` | every bus |
| `bus` | `ZIL` | `Bool` | both halves of a split busbar |
| `bus` | `bus_split` | `Int` | halves and auxiliary buses |
| `bus` | `auxiliary_bus` | `Bool` | auxiliary buses |
| `bus` | `auxiliary` | `String` | auxiliary buses |
| `bus` | `original` | `Int` | auxiliary buses |
| `switch` | `ZIL` | `Bool` | couplers only |
| `switch` | `bus_split` | `Int` | all |
| `switch` | `auxiliary`, `original` | `String`, `Int` | element switches only |
| `switch` | `cost` | `Float64` | `1.0` couplers, `0.0` element switches |
| `busdc` | as `bus`, with `busdc_split` | | |
| `dcswitch` | as `switch`, with `busdc_split` | | |
| *(top level)* | `switch_couples` | `Dict` | after `AC_busbars_split` |
| *(top level)* | `dcswitch_couples` | `Dict` | after `DC_busbars_split` |