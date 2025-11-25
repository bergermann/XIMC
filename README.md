# XIMC (WIP)
Julia wrapper for XIMC motor control package by Standa (www.standa.lt).  
The standard wrapper has been expanded for ease of use and to conform more to Julian standards.  

This module was created as part of the MADMAX project at RWTH Aachen.
The functionality is suited to the needs of one project and therefore
offers not yet the full functionality possible.

## Installation

To install the package run one of the following commands in the Julia REPL.

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

# Bind key file (this is not yet automated inside of the package)
setBindyKey("keyfile.sqlite")

# Connect to motor with URI "Big Chungus", at specified IP port
D = requestDevices("123.45.67.890","Big Chungus")

# Move the motor to position: 1000 steps and 0 micro steps
# and wait for it stop
commandMove(D, 1000, 0)
commandWaitForStop(D)

# Close the connection
closeDevices(D)
```

## Documenation

The full documentation can be found [here](https://ximc-jl-nick1-7d57c72cdff051bfc51526e4f437e4b729e4ce3f351731c3f.pages.rwth-aachen.de/).

## Contact
- [Dominik Bergermann](mailto:dominik.bergermann@rwth-aachen.de)
