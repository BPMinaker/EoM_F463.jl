function build_model!(params)

    # Generate equations of motion from EoM

    temp(x) = input_F463(u = x, params)
    my_sys, my_eqns, my_data = run_eom(temp, vpts = 0, :diagnose)
    my_sys = my_sys[1]

    params.bodys = my_sys.bodys

    names = lowercase.(name.(my_sys.bodys))  ## Search for the chassis and wheels
    params.wheelnum = []

    for i = 1:length(names)    # Loop over each body
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
    names = lowercase.(name.(my_sys.flex_points))  ## Search for the tire vert spring
    for i = 1:length(names)    ## Loop over each body
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

    names = lowercase.(name.(my_sys.sensors))  ## Search for tire speed and vert disp sensors
    for i = 1:length(names)    ## Loop over each body
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

    names = lowercase.(name.(my_sys.actuators))  ## Search for aero, torque, and tire force actuators

    for i = 1:length(names)    ## Loop over each actuator
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

    params.r_orth = nullspace(my_data[1].constraint)
    params.g_mtx = my_data[1].output * params.r_orth  ## Sensors matrix
    params.f_mtx = params.r_orth' * my_data[1].input  ## Actuators matrix


    params.m_mtx = params.r_orth' * my_data[1].mass * params.r_orth
    params.l_mtx = params.r_orth' * my_data[1].damping * params.r_orth
    params.k_mtx =
        params.r_orth' *
        (my_data[1].stiffness + my_data[1].tangent_stiffness + my_data[1].load_stiffness) *
        params.r_orth


    params.rads = location.(my_sys.bodys)
    #params.mass=[syst.data.bodys.mass];
    #params.inertia=[syst.data.bodys.inertia];

end ## Leave


    # #error("stop");

    # [q,r]=size(syst.eom.rigid.cnsrt_mtx);  ## q = the number of rows in the constraint matrix
    # params.nbods=r/6;

    # if(q>0)
    # 	params.r_orth=null(syst.eom.rigid.cnsrt_mtx);
    # else
    # 	params.r_orth=eye(r);
    # end
    # params.l_orth=params.r_orth';  ## Build the coordinate reduction matrices

    # params.m_mtx=params.l_orth*syst.eom.mass.mtx*params.r_orth;  ## Reduced mass matrix
    # params.c_mtx=params.l_orth*syst.eom.elastic.dmpng_mtx*params.r_orth;  ## Reduced damping
    # params.k_mtx=params.l_orth*syst.eom.eqn_of_mtn.stiff_mtx*params.r_orth;  ## Reduced stiffness
    # params.g_mtx=syst.eom.outputs.sensor_mtx*params.r_orth;  ## Sensors matrix
    # params.f_mtx=params.l_orth*syst.eom.forced.f_mtx;  ## Actuators matrix

    # #params.m_mtx
    # #params.c_mtx
    # #params.k_mtx
    # #error("stopped");

    # init_vel=[syst.data.bodys.velocity];  ## Set initial conditions, bodies and ref frame
    # init_angvel=[syst.data.bodys.angular_velocity];
    # vfi=[syst.data.bodys(params.chassnum).velocity(1); 0;0;0;0;syst.data.bodys(params.chassnum).angular_velocity(3)];

    # #error("stop")

    # v=zeros(r,1);  ## Preallocate , r is number of velocities, full set 
    # for i=1:params.nbods
    # 	v(6*i+(-5:-3))=init_vel(:,i);
    # 	v(6*i+(-2:0))=init_angvel(:,i);
    # end

    # n=size(params.l_orth,1);  ## Should be r-q, but get it here just in case
    # zmin0=[zeros(n,1);params.l_orth*v];  ## Two first order ODE for each DOF, disp zeros, vel from model

    # z0=[0;0;0;1;0;0;0;vfi;0;0;0;zmin0];  ## Set initial conditions for frame and system
    # # initial position of ref frame (0 0 0)
    # # initial orientation of ref frame (0 0 0 1) (Euler parameters)
    # # initial velocities of reference frame vx,vy,vz,wx,wy,wz, from input file to match chassis
    # # initial distance along track
    # # initial orientation
    # # initial steer angle
    # # positions and velocities of vehicle, from input file


