# Getting Started

## Installation

To install the package run one of the following commands in the Julia REPL.

Using SSH:
```julia
using Pkg
Pkg.add(url="git@git.rwth-aachen.de:nick1/XIMC-jl.git")
```

Using HTTPS:
```julia
using Pkg
Pkg.add(url="https://git.rwth-aachen.de/nick1/XIMC-jl.git")
```

### Dependencies

For the package to work the XIMC library provided by Standa [https://libximc.xisupport.com/doc-en/index.html] needs to be installed on your machine.
Make sure that the installation directory is added to the enviroment variable, such that the linker can find the library (E.g. for windows the path has to be added to the `PATH` variable).

## Example usage

A simple example on how to use the Package.
Connecting to the motor and moving it.

```julia
using XIMC

# Connect to motor with URI "Big Chungus"
D = requestDevices("Big Chungus")

# Move the motor to position: 1000 steps and 0 micro steps
# and wait for it stop
commandMove(D, 1000, 0)
commandWaitForStop(D)

# Close the connection
closeDevice(D)
```