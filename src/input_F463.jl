function input_F463(params;u=0)

## Copyright (C) 2017, Bruce Minaker
## full_car_a_arm_pushrod.jl is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2, or (at your option)
## any later version.
##
## full_car_a_arm_pushrod.jl is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
## General Public License for more details at www.gnu.org/copyleft/gpl.html.
##
##--------------------------------------------------------------------


    params.a=params.wb*(1-params.fwf)
    params.b=params.wb*params.fwf

    half_weight = params.mass * params.g / 2
    Zf = half_weight * params.fwf
    Zr = half_weight - Zf

    cf = params.a_mtm[4] * sin(2 * atan(Zf / params.a_mtm[5]))
    cr = params.a_mtm[4] * sin(2 * atan(Zr / params.a_mtm[5]))

    df = (params.b_mtm[4] * Zf ^ 2 + params.b_mtm[5] * Zf) * exp(-params.b_mtm[6] * Zf)
    dr = (params.b_mtm[4] * Zr ^ 2 + params.b_mtm[5] * Zr) * exp(-params.b_mtm[6] * Zr)

    the_system = mbd_system("Full Car F463")

    item = body("Chassis")
    item.mass = params.mass
    item.moments_of_inertia = params.I
    item.location = [0,0,params.hg]
    item.velocity = [u,0,0]
    add_item!(item,the_system)
    add_item!(weight(item,params.g),the_system)


    item=actuator("Chassis X")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [-0.1,0,params.hg]
    add_item!(item,the_system)

    item = actuator("Chassis Y")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [0,-0.1,params.hg]
    add_item!(item,the_system)

    item = sensor("Chassis x")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [-0.1,0,params.hg]
    add_item!(item,the_system)

    item = sensor("Chassis y")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [0,-0.1,params.hg]
    add_item!(item,the_system)

    item = sensor("Chassis z")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [0,0,params.hg-0.1]
    add_item!(item,the_system)

    item=sensor("Chassis ϕ")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [-0.1,0,params.hg]
    item.twist = 1
    item.gain = 180/pi
    add_item!(item,the_system)

    item=sensor("Chassis θ")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [0,-0.1,params.hg]
    item.twist = 1
    item.gain = 180/pi
    add_item!(item,the_system)

    item=sensor("Chassis ψ")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1] = [0,0,params.hg]
    item.location[2] = [0,0,params.hg-0.1]
    item.twist = 1
    item.gain = 180/pi
    add_item!(item,the_system)

    item = body("LF Wheel+hub")
    item.moments_of_inertia = [0,params.Iw,0]
    item.location = [params.a,params.front.t/2,params.rad]
    item.velocity = [u,0,0]
    item.angular_velocity = [0,u/params.rad,0]
    add_item!(item,the_system)

    item = body("LF Upright")
    item.mass = params.front.mu
    item.location = [params.a,params.front.t/2-0.1,params.rad]
    item.velocity = [u,0,0]
    add_item!(item,the_system)
    add_item!(weight(item,params.g),the_system)


    item = rigid_point("LF Wheel bearing")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "LF Upright"
    item.location = [params.a,params.front.t/2,params.rad]
    item.forces = 3
    item.moments = 2
    item.axis = [0,1,0]
    add_item!(item,the_system)

    item = flex_point("LF Tire")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location = [params.a,params.front.t/2,0]
    item.stiffness = [params.front.kt,0]
    item.forces = 1
    item.moments = 0
    item.axis = [0,0,1]
    item.rolling_axis = [0,1,0]
    add_item!(item,the_system)

    if u > 0
        item = flex_point("LF Tire")
        item.body[1] = "LF Wheel+hub"
        item.body[2] = "ground"
        item.location = [params.a,params.front.t/2,0]
        item.damping = [cf/u,0]
        item.forces = 1
        item.moments = 0
        item.axis = [0,1,0]
        add_item!(item,the_system)

        item = flex_point("LF Tire")
        item.body[1] = "LF Wheel+hub"
        item.body[2] = "ground"
        item.location = [params.a,params.front.t/2,0]
        item.damping = [df/u,0]
        item.forces = 1
        item.moments = 0
        item.axis = [1,0,0]
        add_item!(item,the_system)
    end

    item = link("LF Tie-rod")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = params.front.itre + [params.a,params.front.t/2,0]
    item.location[2] = params.front.otre + [params.a,params.front.t/2,0]
    add_item!(item,the_system)

    item = link("LF Upper A-arm, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = params.front.uaapf + [params.a,params.front.t/2,0]
    item.location[2] = params.front.ubj + [params.a,params.front.t/2,0]
    add_item!(item,the_system)

    item = link("LF Upper A-arm, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = params.front.uaapr + [params.a,params.front.t/2,0]
    item.location[2] = params.front.ubj + [params.a,params.front.t/2,0]
    add_item!(item,the_system)

    item = body("LF Lower A-arm")
    item.location = params.front.sle + [0,-0.1,0] + [params.a,params.front.t/2,0]
    add_item!(item,the_system)

    item = rigid_point("LF Lower ball joint")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "LF Upright"
    item.location = params.front.lbj + [params.a,params.front.t/2,0]
    item.forces = 3
    item.moments = 0
    add_item!(item,the_system)

    item = rigid_point("LF Lower A-arm pivot, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Lower A-arm"
    item.location = params.front.laapf + [params.a,params.front.t/2,0]
    item.forces = 3
    item.moments = 0
    add_item!(item,the_system)

    item = rigid_point("LF Lower A-arm pivot, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Lower A-arm"
    item.location = params.front.laapr + [params.a,params.front.t/2,0]
    item.forces = 2
    item.moments = 0
    item.axis = [1,0,0]
    add_item!(item,the_system)

    item = body("LF Anti-roll arm")
    item.location = [params.a-0.25,0.5,params.front.sle[3]-0.05]
    add_item!(item,the_system)

    item = rigid_point("LF Anti-roll arm pivot")
    item.body[1] = "Chassis"
    item.body[2] = "LF Anti-roll arm"
    item.location = [params.a-0.25,0.5,params.front.sle[3]-0.05]
    item.forces = 3
    item.moments = 2
    item.axis = [0,1,0]
    add_item!(item,the_system)

    item = link("LF Drop link")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "LF Anti-roll arm"
    item.location[1] = params.front.sle + [params.a,params.front.t/2,0]
    item.location[2] = params.front.sle + [0,0,-0.05]  + [params.a,params.front.t/2,0]
    add_item!(item,the_system)

    item = spring("LF Suspension spring")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "Chassis"
    item.location[1] = params.front.sle + [params.a,params.front.t/2,0]
    item.location[2] = params.front.sue + [params.a,params.front.t/2,0]
    item.stiffness = params.front.k
    item.damping = params.front.c
    add_item!(item,the_system)

    item=spring("Front anti-roll bar")
    item.body[1]="LF Anti-roll arm"
    item.body[2]="RF Anti-roll arm"
    item.location[1] = [params.a-0.25,0.45,params.front.sle[3]-0.05]
    item.location[2] = item.location[1] .* [1,-1,1] 
    item.stiffness = params.front.kr
    item.twist = 1
    add_item!(item,the_system)

    item=actuator("LF Tire X")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [params.a,params.front.t/2,0]
    item.location[2] = [params.a-0.1,params.front.t/2,0]
    add_item!(item,the_system)

    item=actuator("LF Tire Y")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [params.a,params.front.t/2,0]
    item.location[2] = [params.a,params.front.t/2-0.1,0]
    add_item!(item,the_system)

    item=actuator("LF Tire Z")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [params.a,params.front.t/2,0]
    item.location[2] = [params.a,params.front.t/2,-0.1]
    add_item!(item,the_system)

    item = actuator("LF brake")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "LF Upright"
    item.location[1] = [params.a,params.front.t/2,params.rad]
    item.location[2] = [params.a,params.front.t/2-0.1,params.rad]
    item.twist = 1
    add_item!(item,the_system)

    item = actuator("LF axle torque")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "Chassis"
    item.location[1] = [params.a,params.front.t/2,params.rad]
    item.location[2] = [params.a,params.front.t/2-0.1,params.rad]
    item.twist = 1
    add_item!(item,the_system)

    item = sensor("LF Tire u")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [params.a,params.front.t/2,0]
    item.location[2] = [params.a-0.1,params.front.t/2,0]
    item.order = 2
    add_item!(item,the_system)

    item = sensor("LF Tire v")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [params.a,params.front.t/2,0]
    item.location[2] = [params.a,params.front.t/2-0.1,0]
    item.order = 2
    add_item!(item,the_system)

    item = sensor("LF Tire z")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [params.a,params.front.t/2,0]
    item.location[2] = [params.a,params.front.t/2,-0.1]
    add_item!(item,the_system)

    item = body("LR Wheel+hub")
    item.moments_of_inertia = [0,params.Iw,0]
    item.location = [-params.b,params.rear.t/2,params.rad]
    item.velocity = [u,0,0]
    item.angular_velocity = [0,u/params.rad,0]
    add_item!(item,the_system)

    item = body("LR Upright")
    item.mass = params.rear.mu
    item.location = [-params.b,params.rear.t/2-0.1,params.rad]
    item.velocity = [u,0,0]
    add_item!(item,the_system)
    add_item!(weight(item,params.g),the_system)

    item = rigid_point("LR Wheel bearing")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "LR Upright"
    item.location = [-params.b,params.rear.t/2,params.rad]
    item.forces = 3
    item.moments = 2
    item.axis = [0,1,0]
    add_item!(item,the_system)

    item = flex_point("LR Tire")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location = [-params.b,params.rear.t/2,0]
    item.stiffness = [params.rear.kt,0]
    item.forces = 1
    item.moments = 0
    item.axis = [0,0,1]
    item.rolling_axis = [0,1,0]
    add_item!(item,the_system)

    if u > 0
        item = flex_point("LR Tire")
        item.body[1] = "LR Wheel+hub"
        item.body[2] = "ground"
        item.location = [-params.b,params.rear.t/2,0]
        item.damping = [cr/u,0]
        item.forces = 1
        item.moments = 0
        item.axis = [0,1,0]
        add_item!(item,the_system)

        item = flex_point("LR Tire")
        item.body[1] = "LR Wheel+hub"
        item.body[2] = "ground"
        item.location = [-params.b,params.rear.t/2,0]
        item.damping = [dr/u,0]
        item.forces = 1
        item.moments = 0
        item.axis = [1,0,0]
        add_item!(item,the_system)
    end

    item = link("LR Tie-rod")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = params.rear.itre + [-params.b,params.rear.t/2,0]
    item.location[2] = params.rear.otre + [-params.b,params.rear.t/2,0]
    add_item!(item,the_system)

    item = link("LR Upper A-arm, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = params.rear.uaapf + [-params.b,params.rear.t/2,0]
    item.location[2] = params.rear.ubj + [-params.b,params.rear.t/2,0]
    add_item!(item,the_system)

    item = link("LR Upper A-arm, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = params.rear.uaapr + [-params.b,params.rear.t/2,0]
    item.location[2] = params.rear.ubj + [-params.b,params.rear.t/2,0]
    add_item!(item,the_system)

    item = body("LR Lower A-arm")
    item.location = params.rear.sle + [0,-0.1,0] + [-params.b,params.rear.t/2,0]
    add_item!(item,the_system)

    item = rigid_point("LR Lower ball joint")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "LR Upright"
    item.location = params.rear.lbj + [-params.b,params.rear.t/2,0]
    item.forces = 3
    item.moments = 0
    add_item!(item,the_system)

    item = rigid_point("LR Lower A-arm pivot, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Lower A-arm"
    item.location = params.rear.laapf + [-params.b,params.rear.t/2,0]
    item.forces = 3
    item.moments = 0
    add_item!(item,the_system)

    item = rigid_point("LR Lower A-arm pivot, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Lower A-arm"
    item.location = params.rear.laapr + [-params.b,params.rear.t/2,0]
    item.forces = 2
    item.moments = 0
    item.axis = [1,0,0]
    add_item!(item,the_system)

    item = body("LR Anti-roll arm")
    item.location = [-params.b+0.25,0.5,params.rear.sle[3]-0.05]
    add_item!(item,the_system)

    item = rigid_point("LR Anti-roll arm pivot")
    item.body[1] = "Chassis"
    item.body[2] = "LR Anti-roll arm"
    item.location = [-params.b+0.25,0.5,params.rear.sle[3]-0.05]
    item.forces = 3
    item.moments = 2
    item.axis = [0,1,0]
    add_item!(item,the_system)

    item = link("LR Drop link")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "LR Anti-roll arm"
    item.location[1] = params.rear.sle + [-params.b,params.rear.t/2,0]
    item.location[2] = params.rear.sle + [0,0,-0.05]  + [-params.b,params.rear.t/2,0]
    add_item!(item,the_system)

    item = spring("LR Suspension spring")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "Chassis"
    item.location[1] = params.rear.sle + [-params.b,params.rear.t/2,0]
    item.location[2] = params.rear.sue + [-params.b,params.rear.t/2,0]
    item.stiffness = params.rear.k
    item.damping = params.rear.c
    add_item!(item,the_system)

    item=spring("Rear anti-roll bar")
    item.body[1] = "LR Anti-roll arm"
    item.body[2] = "RR Anti-roll arm"
    item.location[1] = [-params.b+0.25,0.45,params.rear.sle[3]-0.05]
    item.location[2] = item.location[1] .* [1,-1,1] 
    item.stiffness = params.rear.kr
    item.twist = 1
    add_item!(item,the_system)

    item=actuator("LR Tire X")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [-params.b,params.rear.t/2,0]
    item.location[2] = [-params.b-0.1,params.rear.t/2,0]
    item.units = "N"
    add_item!(item,the_system)

    item=actuator("LR Tire Y")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [-params.b,params.rear.t/2,0]
    item.location[2] = [-params.b,params.rear.t/2-0.1,0]
    item.units = "N"
    add_item!(item,the_system)

    item=actuator("LR Tire Z")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [-params.b,params.rear.t/2,0]
    item.location[2] = [-params.b,params.rear.t/2,-0.1]
    item.units = "N"
    add_item!(item,the_system)

    item = actuator("LR brake")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "LR Upright"
    item.location[1] = [-params.b,params.rear.t/2,params.rad]
    item.location[2] = [-params.b,params.rear.t/2-0.1,params.rad]
    item.twist = 1
    item.units = "N*m"
    add_item!(item,the_system)

    item = actuator("LR axle torque")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "Chassis"
    item.location[1] = [-params.b,params.rear.t/2,params.rad]
    item.location[2] = [-params.b,params.rear.t/2-0.1,params.rad]
    item.twist = 1
    item.units = "N*m"
    add_item!(item,the_system)

    item = sensor("LR Tire u")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-params.b,params.rear.t/2,0]
    item.location[2] = [-params.b-0.1,params.rear.t/2,0]
    item.order = 2
    item.units = "m/s"
    add_item!(item,the_system)

    item = sensor("LR Tire v")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [-params.b,params.rear.t/2,0]
    item.location[2] = [-params.b,params.rear.t/2-0.1,0]
    item.order = 2
    item.units = "m/s"
    add_item!(item,the_system)

    item = sensor("LR Tire z")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location[1] = [-params.b,params.rear.t/2,0]
    item.location[2] = [-params.b,params.rear.t/2,-0.1]
    item.units = "m"
    add_item!(item,the_system)

    # reflect all LF or LR items in y axis
    mirror!(the_system)

    the_system

end
