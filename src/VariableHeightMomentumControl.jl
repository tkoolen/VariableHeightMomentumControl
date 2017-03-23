__precompile__()

module VariableHeightMomentumControl

using RigidBodyDynamics
using RigidBodyDynamics.OdeIntegrators
using JuMP
using Gurobi
using StaticArrays

export
    VariableHeightMomentumController,
    control,
    initialize_state!

include("util.jl")
include("initialization.jl")
include("control.jl")

end # module
