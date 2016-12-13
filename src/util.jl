# assumes that the plane in which the CoP lies is the x-y plane of the frame in which the wrench is expressed
function center_of_pressure(wrench::Wrench)
    T = eltype(wrench)
    normal = StaticArrays.SVector(zero(T), zero(T), one(T))
    torque = wrench.angular
    force = wrench.linear
    Point3D(wrench.frame, normal × torque / (normal ⋅ force))
end
