__precompile__()

module VariableHeightMomentumControl

using RigidBodyDynamics
using RigidBodyDynamics.OdeIntegrators
using JuMP
using Gurobi
using StaticArrays

import RigidBodyDynamics: GenericJoint

export
    VariableHeightMomentumController,
    control,
    initialize_state!,
    set_desired_configuration!

include("util.jl")
include("initialization.jl")
include("control.jl")

end # module
