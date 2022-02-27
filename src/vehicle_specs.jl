mutable struct susp # suspension properties
    ubj::Vector{Float64} # upper ball joint
    lbj::Vector{Float64} # lower ball joint
    uaapf::Vector{Float64} # upper a-arm pivot, front
    uaapr::Vector{Float64} # lower a-arm pivot, rear
    laapf::Vector{Float64} # upper a-arm pivot, front
    laapr::Vector{Float64} # lower a-arm pivot, rear
    itre::Vector{Float64} # inner tie rod end
    otre::Vector{Float64} # outer tie rod end
    sue::Vector{Float64} # spring upper end
    sle::Vector{Float64} # spring lower end
    k::Float64 # spring stiffness
    c::Float64 # damping
    kr::Float64 # roll stiffness
    mu::Float64 # unspring mass
    kt::Float64 # tire vertical stiffness
    t::Float64 # track width

end

susp() = susp(zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), 10000, 2000, 5000, 40, 150000, 1.5)

mutable struct props
    mass::Float64 # sprung mass
    I::Vector{Float64} # sprung inertia
    hg::Float64 #  mass centre height
    wb::Float64 # wheelbase

    fwf::Float64 # front weight fraction
    a::Float64
    b::Float64

    rad::Float64 # tire radius 
    front::susp
    rear::susp

    Iw::Float64 # wheel inertia

    cod::Float64 # coefficient of drag
    farea::Float64 # frontal area

    drive::String
    wtf::Int64

    # tire model
    a_mtm::Vector{Float64}
    b_mtm::Vector{Float64}

    g::Float64

    e::Vector{Float64} # gear ratios
    ex::Float64 # final drive ratio
    eff::Float64 # efficiency
    engine_type::Int64
    revs::StepRangeLen{Float64}
    torque::Vector{Float64}
    redline::Float64
    launchrpm::Float64
    vmax::Vector{Float64}

    fbf::Float64 # front brake fraction, from 0 to 1

    acc_max::Float64 # [g's] maximum lateral acceleration
    maxv::Float64 # Driver model max allowed speed (m/s)

    course::Array{Float64,2}
    rads::Vector{Vector{Float64}}
    preload::Vector{Float64}

    r_orth::Array{Float64,2}
    g_mtx::Array{Float64,2}
    f_mtx::Array{Float64,2}

    k_mtx::Array{Float64,2}
    l_mtx::Array{Float64,2}
    m_mtx::Array{Float64,2}

    chassnum::Int64  # body number of chassis, should always be 1
    wheelnum::Vector{Int64}
    tirenum::Vector{Int64}

    lat_sen_num::Vector{Int64}
    long_sen_num::Vector{Int64}

    torque_act_num::Vector{Int64}
    brake_act_num::Vector{Int64}
    lat_act_num::Vector{Int64}
    long_act_num::Vector{Int64}
    vert_act_num::Vector{Int64}
    aero_act_num::Int64
    inertial_act_num::Int64

    bodys::Vector{Any}

end


props(; mass = 1000, Ix = 1000, Iy = 1000, Iz = 1000, hg = 0.1, wb = 1.0, fwf = 0.5, cod = 0.1, farea = 1.0, front = susp(), rear = susp(), drive = "RearWheelDrive", engine_type = 1, e = [5, 4, 3, 2, 1], ex = 3.5, fbf = 0.65, acc_max = 0.3) = props(
    mass, # sprung mass
    [Ix, Iy, Iz], # sprung inertia
    hg, #  mass centre height
    wb, # wheelbase
    fwf, # front weight fraction
    0, # a
    0, #b
    17 / 2 * 0.0254 + 205 * 50 / 100000, # tire radius 
    front,
    rear,
    1.3, # wheel inertia
    cod, # coefficient of drag
    farea, # frontal area
    drive,
    1, # rear wheel drive
    [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / pi, 6494.6, 4.7966E-3 * 180 / pi, -0.3875E-3, 1.0],
    [1.65, -7.6118E-6, 1.1226, -7.36E-7, 14.482, -7.6614E-5, -3.86E-9, 8.5055E-5, 7.5719E-2],
    9.81,
    e, # gear ratios
    ex, # final drive ratio
    0.9, # efficiency
    engine_type,
    1:1:1,
    [0],
    0, # redline 
    0, # launchrpm
    [0, 0, 0, 0, 0], # vmax
    fbf, # front brake fraction, from 0 to 1
    acc_max, # [g's] maximum lateral acceleration
    100.0, # Driver model max allowed speed (m/s)
    [0 0; 0 0],
    [[]],
    [0, 0, 0, 0], # preload
    [0 0; 0 0],
    [0 0; 0 0],
    [0 0; 0 0],
    [0 0; 0 0],
    [0 0; 0 0],
    [0 0; 0 0],
    1,  # body number of chassis, should always be 1
    [],
    [],
    [],
    [],
    [],
    [],
    [],
    [],
    [],
    0,
    0,
    []
)
