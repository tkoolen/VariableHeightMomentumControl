type DrakeVisualizerSink <: OdeResultsSink
    vis::DrakeVisualizer.Visualizer
    Δt::Float64
    lastUpdateTime::Float64

    DrakeVisualizerSink(vis::DrakeVisualizer.Visualizer, Δt::Float64) = new(vis, Δt, -Inf)
end

function OdeIntegrators.initialize(sink::DrakeVisualizerSink, t, state)
    DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()
    sink.lastUpdateTime = -Inf
end

function OdeIntegrators.process(sink::DrakeVisualizerSink, t, state)
    if t > sink.lastUpdateTime + sink.Δt
        draw(sink.vis, state)
        sink.lastUpdateTime = t
    end
end
