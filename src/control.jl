# See https://github.com/tkoolen/VariableHeightInvertedPendulum
function cubic_orbital_energy_control_law(g, zf, x, z, xd, zd)
    a = xd / x
    b = zd - a * z
    u = -7 * a^2 + (3 * zf * a^3 - g * a) / b - (10 * a^3 * b) / g
end

mutable struct VariableHeightMomentumController
    τ::Vector{Float64}
    dynamicsResult::DynamicsResult{Float64}
    positionControlledJointReferences::Dict{GenericJoint{Float64}, Vector{Float64}}
    centroidalFrame::CartesianFrame3D
    soleFrame::CartesianFrame3D

    function VariableHeightMomentumController(
            mechanism::Mechanism, positionControlledJoints::Vector{GenericJoint{Float64}},
            centroidalFrame::CartesianFrame3D, soleFrame::CartesianFrame3D)
        τ = zeros(num_velocities(mechanism))
        positionControlledJointReferences = Dict(j => zeros(num_positions(j)) for j in positionControlledJoints)
        new(τ, DynamicsResult{Float64}(mechanism), positionControlledJointReferences, centroidalFrame, soleFrame)
    end
end

function set_desired_configuration!(controller::VariableHeightMomentumController, joint::Joint, qdes::AbstractVector)
    controller.positionControlledJointReferences[joint] .= qdes
end

function control(controller::VariableHeightMomentumController, t, state)
    # create model
    solver = Gurobi.GurobiSolver() # currently overkill; it's just a linear solve right now.
    Gurobi.setparameters!(solver, Silent = true)
    model = Model(solver = solver)
    nv = num_velocities(state)
    @variable(model, v̇[1 : nv])

    # Compute desired rate of change of linear momentum
    centroidalFrame = controller.centroidalFrame
    soleFrame = controller.soleFrame
    mechanism = state.mechanism
    com = center_of_mass(state) - transform(state, Point3D(Float64, soleFrame), root_frame(mechanism))
    v = velocity(state)
    A = momentum_matrix(state)
    m = mass(state.mechanism)
    comdot = FreeVector3D(A.frame, A.linear * v) / m
    g = -RigidBodyDynamics.gravitational_spatial_acceleration(mechanism).linear[3]
    zf = 0.95
    x, z = com.v[1], com.v[3]
    xd, zd = comdot.v[1], comdot.v[3]
    u = cubic_orbital_energy_control_law(g, zf, x, z, xd, zd)
    u = max(u, zero(u))
    fx = m * x * u
    fz = m * (z * u - g)
    ḣdes = Wrench{Float64}(centroidalFrame, zero(SVector{3, eltype(u)}), SVector(fx, zero(u), fz))
    centroidalToWorld = Transform3D(centroidalFrame, root_frame(mechanism), com.v)
    ḣdes = transform(ḣdes, centroidalToWorld)

    # Set up QP
    Ȧv = momentum_rate_bias(state)
    @framecheck A.frame ḣdes.frame
    @framecheck A.frame Ȧv.frame
    @constraint(model, Array(A.linear) * v̇ + Array(Ȧv.linear) .== Array(ḣdes.linear))
    @constraint(model, Array(A.angular) * v̇ + Array(Ȧv.angular) .== Array(ḣdes.angular))

    # PD control for upper body and free leg
    kp = 100.
    kd = 20.
    for (joint, q_desired) in controller.positionControlledJointReferences
        v̇pd = kp .* (q_desired .- configuration(state, joint)) - kd .* velocity(state, joint)
        @constraint(model, v̇[velocity_range(state, joint)] .== v̇pd)
    end

    # minimize squared joint accelerations
    @objective(model, Min, sum(v̇[i]^2 for i = 1 : nv))

    # solve
    status = solve(model)

    # Compute CoP
    gravitationalWrench = transform(Wrench(centroidalFrame, zero(SVector{3}), SVector(0, 0, -m * g)), centroidalToWorld)
    groundReactionWrench = ḣdes - gravitationalWrench
    groundReactionWrench = transform(state, groundReactionWrench, soleFrame)
    cop = center_of_pressure(groundReactionWrench)
    cop = transform(state, cop, root_frame(mechanism))
    @assert all(isnan(cop.v)) || isapprox(cop.v[1], 0., atol = 1e-8)

    # inverse dynamics
    result = controller.dynamicsResult
    result.v̇ = getvalue(v̇)
    inverse_dynamics!(controller.τ, result.jointwrenches, result.accelerations, state, result.v̇)
    controller.τ
end
