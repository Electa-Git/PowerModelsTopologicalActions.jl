#=
Build script for the busbar-splitting webinar material.

    julia --project=docs make_webinar.jl

Generates, from the single source `webinar_busbar_splitting.jl`:

  build/webinar_busbar_splitting.md     — markdown (Documenter flavor, for docs/)
  build/webinar_busbar_splitting.ipynb  — Jupyter notebook (participants run this)
  build/webinar_busbar_splitting.jl     — clean script, prose stripped to comments

Set EXECUTE=true to run the notebook at build time, which bakes the outputs in.
That needs a working Gurobi licence on the build machine — leave it false for CI.
=#

using Literate

const SRC     = joinpath(@__DIR__, "webinar_busbar_splitting.jl")
const OUT     = joinpath(@__DIR__, "build")
const EXECUTE = false

mkpath(OUT)

# --- 1. Markdown ------------------------------------------------------------
# DocumenterFlavor renders the `!!! warning` / `!!! danger` blocks as admonitions
# and `` ```math `` blocks via KaTeX. Use CommonMarkFlavor() if you are publishing
# somewhere other than Documenter (GitHub README, MkDocs, a static site).
Literate.markdown(SRC, OUT;
    flavor  = Literate.DocumenterFlavor(),
    execute = EXECUTE,
    credit  = false,
)

# --- 2. Notebook ------------------------------------------------------------
# `execute = false` ships an unexecuted notebook: participants run it themselves,
# which is what you want for a live session. Flip it to true if you need a
# reference copy with outputs for people who cannot install a solver.
Literate.notebook(SRC, OUT;
    execute = EXECUTE,
    credit  = false,
)

# --- 3. Plain script --------------------------------------------------------
# Prose becomes comments. This is the "here is everything we ran" handout.
Literate.script(SRC, OUT;
    credit     = false,
    keep_comments = true,
)

@info "Generated" readdir(OUT)

#=
--- Hooking this into the package docs -------------------------------------

`PowerModelsTopologicalActions.jl` already has a `docs/` folder with `make.jl`.
To publish the webinar as a docs page, generate into `docs/src/` before the
`makedocs` call and add it to the page list:

    using Literate
    Literate.markdown(
        joinpath(@__DIR__, "..", "webinar_busbar_splitting.jl"),
        joinpath(@__DIR__, "src");
        flavor = Literate.DocumenterFlavor(), credit = false,
    )

    makedocs(
        sitename = "PowerModelsTopologicalActions.jl",
        pages = [
            "Home"                 => "index.md",
            "Installation"         => "installation.md",
            "Quick start"          => "quickstart.md",
            "Busbar splitting"     => "busbar_splitting.md",
            "Webinar tutorial"     => "webinar_busbar_splitting.md",   # ← new
            ...
        ],
    )

Add `build/` and the generated `docs/src/webinar_busbar_splitting.md` to
`.gitignore` — they are artifacts, not sources.

--- Literate syntax cheat sheet used in the source --------------------------

  # text          markdown line
  ## text         a real code comment, survives into the outputs as `#`
  #-              force a chunk (= cell) break
  #md ...         include only in the markdown output
  #nb ...         include only in the notebook output
  #jl ...         include only in the plain-script output
  #src ...        keep in the source only, drop from every output
  #!nb ...        include everywhere except the notebook
  ```math ... ``` display math (KaTeX in Documenter, MathJax in the notebook)
  ``x``           inline math

--- Why a `!!! warning` renders differently per output ---------------------

Admonitions are a Documenter construct. In markdown they become styled callouts;
in the notebook they degrade to plain text. Where the callout genuinely carries
the message — the "how to use this page" box at the top — the source uses `#md`
and `#nb` variants so each output gets an idiomatic version. Elsewhere plain
blockquotes (`> ...`) are used, which render acceptably everywhere.
=#
