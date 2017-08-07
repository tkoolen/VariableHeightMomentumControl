function solve_for_velocities!(state::MechanismState, momentum::Momentum, fixedJoints::Vector{GenericJoint{Float64}})
    solver = Gurobi.GurobiSolver()
    Gurobi.setparameters!(solver, Silent = true)
    model = Model(solver = solver)
    nv = num_velocities(state)
    A = momentum_matrix(state)

    momentum = transform(state, momentum, A.frame)

    @variable(model, v[1 : nv])
    @constraint(model, Array(A.linear) * v .== Array(momentum.linear))
    @constraint(model, Array(A.angular) * v .== Array(momentum.angular))
    for joint in fixedJoints
        @constraint(model, v[velocity_range(state, joint)] .== 0)
    end
    @objective(model, Min, sum(v[i]^2 for i = 1 : nv))

    status = solve(model)
    set_velocity!(state, getvalue(v))
end

function initialize_state!(state::MechanismState, xd::Float64, positionControlledJoints)
    setdirty!(state)
    com = center_of_mass(state)
    centroidalFrame = CartesianFrame3D()
    comdot = FreeVector3D(centroidalFrame, SVector(xd, 0.0, 0.0))
    linearMomentum = mass(state.mechanism) * comdot
    initialMomentum = Momentum{Float64}(centroidalFrame, zero(SVector{3}), linearMomentum.v)
    centroidalToWorld = Transform3D(centroidalFrame, root_frame(state.mechanism), com.v)
    initialMomentum = transform(initialMomentum, centroidalToWorld)
    solve_for_velocities!(state, initialMomentum, positionControlledJoints)
end
