using Documenter

include("../src/XIMC.jl")
using .XIMC

# makedocs(
#     sitename="KeyVNA Documentation",
#     pages = [
#         "setup_file.md",
#         "Setup File" => "setup_file.md",
#         "Subsection" => []
#     ]
# )


format = Documenter.HTML(edit_link = "master",
                         prettyurls = get(ENV, "CI", nothing) == "true"
)

pdf = Documenter.LaTeX(platform = "tectonic")

About = "About" => "index.md"

GettingStarted = "Getting Started" => "getting_started.md"

UserGuide = "User guide" => "user.md"

# License = "License" => "license.md"

PAGES = [
    About,
    UserGuide,
    GettingStarted
    ]

makedocs(
    modules = [XIMC],
    sitename = "XIMC.jl",
    format = format,
    checkdocs = :exports,
    pages = PAGES,
    repo = Remotes.GitLab("https://git.rwth-aachen.de/", "nick1", "XIMC-jl")
)

# operations_cb()

# deploydocs(repo = "github.com/da-boi/KeyVNA.jl")