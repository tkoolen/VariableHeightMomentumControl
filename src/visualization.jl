using RigidBodyTreeInspector
using DrakeVisualizer

import DrakeVisualizer: Visualizer

using RigidBodyDynamics.OdeIntegrators
import RigidBodyDynamics.OdeIntegrators: initialize, process

type VisualizerOdeResultsSink <: OdeResultsSink
    vis::Visualizer
    min_wall_Δt::Float64
    last_update_wall_time::Float64

    function VisualizerOdeResultsSink(vis::Visualizer; max_fps::Float64 = 60.)
        new(vis, 1 / max_fps, -Inf)
    end
end

function initialize(sink::VisualizerOdeResultsSink, t, state)
    sink.last_update_wall_time = -Inf
    process(sink, t, state)
end

function process(sink::VisualizerOdeResultsSink, t, state)
    wall_Δt = time() - sink.last_update_wall_time
    if wall_Δt > sink.min_wall_Δt
        settransform!(sink.vis, state)
        sink.last_update_wall_time = time()
    end
    nothing
end
