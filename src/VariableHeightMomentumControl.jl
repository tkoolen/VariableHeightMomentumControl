__precompile__()

module VariableHeightMomentumControl

using RigidBodyDynamics
using RigidBodyDynamics.TreeDataStructure
using RigidBodyDynamics.OdeIntegrators
using JuMP
using Gurobi
using StaticArrays
import RigidBodyDynamics: @framecheck

export
    VariableHeightMomentumController,
    DrakeVisualizerSink,
    control,
    initialize_state!


include("visualization.jl")
include("util.jl")
include("initialization.jl")
include("control.jl")

end # module
