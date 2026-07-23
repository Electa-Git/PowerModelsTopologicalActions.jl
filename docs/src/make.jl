using Documenter
using PowerModelsTopologicalActions

makedocs(
    modules  = [PowerModelsTopologicalActions],
    sitename = "PowerModelsTopologicalActions.jl",
    authors  = "Giacomo Bastianel, Marta Vanin, Dirk Van Hertem, Hakan Ergun",
    format   = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true",
        canonical  = "https://electa-git.github.io/PowerModelsTopologicalActions.jl/stable/",
        assets     = String[],
    ),
    pages = [
        "Home" => "index.md",
        "Getting started" => [
            "Installation" => "installation.md",
            "Quick start"  => "quickstart.md",
        ],
        "Manual" => [
            "Optimal transmission switching" => "ots.md",
            "Busbar splitting"               => "busbar_splitting.md",
            "AC feasibility check"           => "feasibility_check.md",
            "Formulations"                   => "formulations.md",
        ],
        "Reference" => [
            "Data model"    => "data_model.md",
            "API reference" => "api.md",
        ],
        "Known issues and gotchas" => "known_issues.md",
    ],
    # The package currently has almost no docstrings, so doctests and the
    # missing-docs check are disabled. Re-enable `checkdocs = :exports` once
    # docstrings have been added to the exported problem specifications.
    checkdocs = :none,
    doctest   = false,
    warnonly  = [:missing_docs],
)

deploydocs(
    repo      = "github.com/Electa-Git/PowerModelsTopologicalActions.jl.git",
    devbranch = "main",
    push_preview = true,
)