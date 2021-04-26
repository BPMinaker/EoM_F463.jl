function eqn_of_motion(zdot, z, params, t; flag = false)

    pp = false
    pp && println("t: ", t)

    frm = z[1:13]

    pp && println("frm: ", frm)

    x = z[1]  # Unload reference frame location
    y = z[2]
    psi = z[3]

    s = z[4]  # Distance travelled by reference frame
    delta = z[5]  # Steer angle
    throttle = z[6]

    pp && println("s: ", s)
    pp && println("delta: ", delta)
    pp && println("throttle: ", throttle)

    cp = cos(psi)
    sp = sin(psi)
    rot_mtx = [cp -sp; sp cp]  # Find rotation matrix

    z = z[7:end]  # Unload the system state from the rest of the state vector
    pmin = z[1:14]  # Positions, minimal coordinates
    wmin = z[15:end]  # Velocities, minimal coordindates

    p = params.r_orth * pmin  # Convert minimal coordinates to physical coordinates to compute nonlinear inertial acceleration terms
    w = params.r_orth * wmin

    vel = [w[6*params.chassnum-5], w[6*params.chassnum-4], 0]  # Record chassis speed
    ang_vel = [0, 0, w[6*params.chassnum]]

    glbl_vel = rot_mtx * vel[1:2]

    pp && display(p[1:6])
    pp && display(w[1:6])

    path_error, heading_error, u_ref, curv = track_offset([x, y], psi, s, vel[1], params)

    pp && println("chass u: ", vel[1])
    pp && println("chass yaw rate: ", w[6*params.chassnum])
    pp && println("path_error: ", path_error)
    pp && println("offset: ", offset)

    # driver model
    k = [10, 10, 6, 2, 0.8, 0.16, 0.04, 0.01] # weights of preview points
    k /= sum(k) # normalized
    path_error = k' * path_error
    heading_error = k' * heading_error

    # calculate the steering angle based on road curvature, plus error
    err = params.wb * curv + 1.1 * heading_error + 0.1 * path_error
    # find delta by integrating, effectively adds low pass filter

    delta_dot = 10 * (err - delta)

    # if steer is very large and increasing
    if abs(delta) > 1 && delta * delta_dot > 0
        delta_dot = 0
    end

    # nonlinear inertial forces and moments to each body and kinematics
    n = size(p)  # Number of coordinates
    fi = zeros(n)  # Preallocate space, faster
    pdot = zeros(n)

    for i = 1:17  # Loop over bodies
        lin = 6 * i .+ (-5:-3)
        rot = 6 * i .+ (-2:0)

        pdot[lin] = w[lin] - vel - skew(ang_vel) * (p[lin] + params.rads[i])
        pdot[rot] = w[rot] - ang_vel

    end
    pmindot = params.r_orth' * pdot  # Convert velocities relative to frame to reduced coordinates


    wheel_vel = zeros(3, length(params.wheelnum))  # Calculate slip angles from wheel speeds
    wheel_angvel = zeros(3, length(params.wheelnum))  # Calculate slip ratios from wheel ang speeds
    camber = zeros(4)
    steer = zeros(4)
    wheel_bounce = zeros(4)

    for i = 1:length(params.wheelnum)
        lin = 6 * params.wheelnum[i] .+ (-5:-3)
        rot = 6 * params.wheelnum[i] .+ (-2:0)

        wheel_vel[:, i] = w[lin]
        wheel_angvel[:, i] = w[rot]
        wheel_bounce[i] = p[lin[3]]
        camber[i] = p[rot[1]]
        steer[i] = p[rot[3]]

    end

    camber[1:2] .*= -1  # how to know 1,2 are always left side? fix me...
    wheelspin = wheel_angvel[2, :]

    pp && println("wheelspin: ", wheelspin)

    nrm_frc =
        params.preload -
        [params.front.kt, params.rear.kt, params.front.kt, params.rear.kt] .* wheel_bounce  # Compute normal forces

    # display(nrm_frc)

    v_lat = params.g_mtx[params.lat_sen_num, :] * wmin  # Calculate slip angles from lateral speed sensors
    v_long = params.g_mtx[params.long_sen_num, :] * wmin  # Calculate slip ratios from long speed sensors

    pp && println("v: ", v_lat)
    pp && println("u: ", v_long)


    v_lat -= wheel_vel[1, :] .* sin.([delta, 0, delta, 0] .+ steer)   # Add steer to slip  # how to know 1,3 are front wheels???  fix me...

    v_slip = (v_lat .^ 2 + v_long .^ 2) .^ 0.5 .+ eps(1.0)  # Find magnitude of slip

    slip = v_slip ./ (wheel_vel[1, :] .+ 0.1) # Add small delta to prevent inf result - huge slip ratio at low speeds - fix me...
    ratio = (v_lat ./ v_slip) .^ 2  # Compute ratio of lateral to long, 1=lateral, 0=long

    pp && println("slip: ", slip)
    pp && println("ratio: ", ratio)

    tire_frc = tire_comb(slip, nrm_frc, ratio, camber, params)  # Tire model
    tire_frc_pus = tire_frc ./ v_slip

    lat_frc = -v_lat .* tire_frc_pus
    long_frc = -v_long .* tire_frc_pus

    pp && println("Y: ", lat_frc)
    pp && println("X: ", long_frc)

    faero = 0
    froll = zeros(4)
    if vel[1] > 0
        faero = 0.5 * 1.23 * params.cod * params.farea * vel[1]^2 # Aero drag force
        froll = nrm_frc * (0.0136 + 5.2 * 10^-7 * vel[1]^2) # Rolling resistance force
    end

    long_frc -= froll

    # longitudinal speed control, tanh function smooths out on/off behavour of throttle, gentle slope around zero, much steeper away from zero, smoothly limited to +/- 1
    throttle_dot = 20 * (tanh(0.2 * (u_ref - vel[1])^3) - throttle)
    brake = -throttle
    # limit min throttle, brake
    throttle < 0 && (throttle = 0)
    brake < 0 && (brake = 0)

    # find correct gear
    gear = findnext(vel[1] .< params.vmax, 1)
    # if going above redline in top gear, use top gear (it happens sometimes, not sure quite how, slip ratio?)
    if isnothing(gear)
        gear = length(params.e)
    end

    # average wheel speed
    wf = (wheelspin[1] + wheelspin[3]) / 2
    wr = (wheelspin[2] + wheelspin[4]) / 2
    # convert to rpm, find engine speed
    # if FWD, use front wheel speed, if RWD, use rear wheel speed
    if params.wtf == 1
        we = wr * params.ex * params.e[gear] * 30 / pi
    elseif params.wtf == -1
        we = wf * params.ex * params.e[gear] * 30 / pi
    else
        we = (wf + wr) * params.ex * params.e[gear] * 15 / pi
    end

    # add clutch torque, for low speed starts, launch control
    ct = 0
    if gear == 1 && we < params.launchrpm
        ct = (params.launchrpm - we) * 0.05
        we = params.launchrpm
    end
    # limit engine speed, close throttle at 500 rpm above redline
    if we > params.redline
        throttle -= (we - params.redline) / 500
        if throttle < 0
            throttle = 0
        end
        we = params.redline
    end

    # fit torque curve to data
    te = CubicSplineInterpolation(params.revs, params.torque)
    # engine torque from curve fit, plus clutch torque
    tq = ct + throttle * te(we)

    # convert engine torque to axle torque, -ve of right side to accoubnt for mirror of axle
    if params.wtf == 1
        ax_tq = tq * params.eff * params.ex * params.e[gear] * [0, 0.5, 0, 0.5]
    elseif params.wtf == -1
        ax_tq = tq * params.eff * params.ex * params.e[gear] * [0.5, 0, 0.5, 0]
    else
        ax_tq = tq * params.eff * params.ex * params.e[gear] * [0.25, 0.25, 0.25, 0.25]
    end

    # compute brake torques
    brake_m = 1.5 * params.mass * params.g * params.rad * brake
    # divide brake torque front/rear by weighting
    brake_tq =
        0.5 * [params.fbf, (1 - params.fbf), params.fbf, (1 - params.fbf)] .*
        tanh.(3 * wheelspin) * brake_m

    # reduce all the forces to minimal coordinates
    fnlmin = -params.f_mtx[:, params.inertial_act_num] * params.mass * vel[1] * ang_vel[3] # Ficticious lateral force  

    femin = params.f_mtx[:, params.lat_act_num] * lat_frc # Treat tire forces as external

    ftracmin = params.f_mtx[:, params.long_act_num] * long_frc

    fimin = -params.k_mtx * pmin - params.l_mtx * pmindot  # Internal stiffness and damping

    fdmin = params.f_mtx[:, params.torque_act_num] * ax_tq  # Apply torque to drive axles

    fbrakemin = -params.f_mtx[:, params.brake_act_num] * brake_tq   # Apply torque to brakes

    fres = params.f_mtx[:, params.aero_act_num] * -faero # Aero drag

    # Find accelerations in reduced coordinates
    wmindot = params.m_mtx \ (fnlmin + femin + fimin + fres + fdmin + ftracmin + fbrakemin)

    if pp
        temp = params.r_orth * wmindot
        println("wdot: ", temp[1:6])
    end

    # return slopes to ODE solver
    # return extra data after ODE solved
    if ~flag
        zdot .= [
            glbl_vel
            ang_vel[3]
            vel[1]
            delta_dot
            throttle_dot
            pmindot
            wmindot
        ]
    else
        zdot .= [
            u_ref
            path_error
            heading_error
            we
            gear
            camber
            long_frc
            lat_frc
            nrm_frc
            slip
        ] # extra outputs
    end

end # Leave
