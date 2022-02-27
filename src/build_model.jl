function build_model!(params::EoM_F463.props)

    # Generate equations of motion from EoM
    my_sys = input_F463(u = 0, params)
    EoM.sort_system!(my_sys)
    my_data = EoM.generate_eom(my_sys)

    params.bodys = my_sys.bodys

    names = lowercase.(getfield.(my_sys.bodys, :name))  ## Search for the chassis and wheels
    params.wheelnum = []

    for i in 1:length(names)    # Loop over each body
        if contains(names[i], "chassis")
            params.chassnum = i
        end
        if contains(names[i], "wheel")
            push!(params.wheelnum, i)
        end
    end
    #params.wheelnum
    ##params.chassnum
    if params.chassnum == 0
        error("Missing chassis.")
    end

    params.tirenum = []
    names = lowercase.(getfield.(my_sys.flex_points, :name))  ## Search for the tire vert spring
    for i in 1:length(names)    ## Loop over each body
        if contains(names[i], "tire")  # && contains(names[i],"vertical")
            push!(params.tirenum, i)
        end
    end

    params.preload = []
    for i in params.tirenum
        append!(params.preload, my_sys.flex_points[i].preload)
    end

    if any(params.preload .< 0)
        error("Negative tire preload!")
    end

    params.lat_sen_num = []
    #params.vert_sen_num = []
    params.long_sen_num = []

    names = lowercase.(getfield.(my_sys.sensors, :name))  ## Search for tire speed and vert disp sensors
    for i in 1:length(names)    ## Loop over each body
        if contains(names[i], "tire") && contains(names[i], " v")
            push!(params.lat_sen_num, i)
        end
        if contains(names[i], "tire") && contains(names[i], " u")
            push!(params.long_sen_num, i)
        end
    end
    # display("lat_sen")
    # display(params.lat_sen_num)
    # display("vert_sen")
    # display(params.vert_sen_num)
    # display("long_sen")
    # display(params.long_sen_num)


    params.lat_act_num = []
    params.long_act_num = []
    params.vert_act_num = []
    params.aero_act_num = 0
    params.inertial_act_num = 0
    params.torque_act_num = []
    params.brake_act_num = []

    names = lowercase.(getfield.(my_sys.actuators, :name))  ## Search for aero, torque, and tire force actuators

    for i in 1:length(names)    ## Loop over each actuator
        if contains(names[i], "tire") && contains(names[i], " y")
            push!(params.lat_act_num, i)
        end
        if contains(names[i], "tire") && contains(names[i], " x")
            push!(params.long_act_num, i)
        end
        if contains(names[i], "tire") && contains(names[i], " z")
            push!(params.vert_act_num, i)
        end
        if contains(names[i], "chassis x")
            params.aero_act_num = i
        end
        if contains(names[i], "chassis y")
            params.inertial_act_num = i
        end
        if contains(names[i], "torque")
            push!(params.torque_act_num, i)
        end
        if contains(names[i], "brake")
            push!(params.brake_act_num, i)
        end
    end


    # display("lat_act")
    # display(params.lat_act_num)
    # display("long_act")
    # display(params.long_act_num)
    # # display("aero_act");
    # # params.aero_act_num
    # # disp("torque_act");
    # # params.torque_act_num

    params.r_orth = nullspace(my_data.constraint)
    params.g_mtx = my_data.output * params.r_orth  ## Sensors matrix
    params.f_mtx = params.r_orth' * my_data.input  ## Actuators matrix


    params.m_mtx = params.r_orth' * my_data.mass * params.r_orth
    params.l_mtx = params.r_orth' * my_data.damping * params.r_orth
    params.k_mtx =
        params.r_orth' *
        (my_data.stiffness + my_data.tangent_stiffness + my_data.load_stiffness) *
        params.r_orth


    params.rads = getfield.(my_sys.bodys, :location)
    #params.mass=[syst.data.bodys.mass];
    #params.inertia=[syst.data.bodys.inertia];

end ## Leave


