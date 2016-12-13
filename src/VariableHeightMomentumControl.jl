__precompile__()

module VariableHeightMomentumControl

using RigidBodyDynamics
using RigidBodyDynamics.TreeDataStructure
using RigidBodyDynamics.OdeIntegrators

# TODO: move to RigidBodyTreeInspector?
using RigidBodyTreeInspector
using DrakeVisualizer
export DrakeVisualizerSink
include("visualization.jl")

end # module
