function F463_animate(lptime, nums; filename = "history.html")

    ##
    ## This program is free software; you can redistribute it and/or modify it
    ## under the terms of the GNU General Public License as published by the
    ## Free Software Foundation; either version 2, or (at your option) any
    ## later version.
    ##
    ## This is distributed in the hope that it will be useful, but WITHOUT
    ## ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    ## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    ## for more details.
    ##

    println("Writing animation...")
    params = props()
    track!(params)

    # file name and track path for animation
    track = [params.course[:, 1:2] zeros(size(params.course))]

    nt = length(lptime)
    nt > 3 && (nt = 3)

    #locate first three ranks in the results
    v_win = partialsortperm(lptime, 1:nt) # sort out the three least laptime and locate the index
    nums_win = nums[v_win] # find the correspond student_ID for three winners
    laptime = lptime[v_win] # same as above for laptimes

    #    println(laptime)

    #Define parameters
    x3d_chassis = ["" "" ""] # predefine the string used to draw car bodies
    tck = "" # track path data
    tck_idx = "" #index for tracking path points
    s_tck = "" #pre-define the track path string
    n = size(track, 1)
    scal = ""
    s_total = ""

    #Draw ground (using the x3d_grnd function )
    x3d_grnd = EoM_X3D.x3d_pnt(
        [0, 0, 0],
        cubes = true,
        rad = [50, 50, 0.01],
        col = [0.25, 0.25, 0.25],
        tran = 0.5,
    )
    #vehicle
    x3d_chassis[1] = EoM_X3D.x3d_pnt(
                         [0, 0, 0],
                         cubes = true,
                         rad = [2.5, 1.5, 1.2],
                         col = [1, 0, 0],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([0, 0, 0.6], [0, 0, 2.6]),
                         rad = 0.1,
                         col = [1, 0, 0],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([1.25, 0, 0], [3.25, 0, 0]),
                         rad = 0.1,
                         col = [1, 0, 0],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([0, 0.75, 0], [0, 2.75, 0]),
                         rad = 0.1,
                         col = [1, 0, 0],
                         tran = 0.3,
                     )

    x3d_chassis[2] = EoM_X3D.x3d_pnt(
                         [0, 0, 0],
                         cubes = true,
                         rad = [2.5, 1.5, 1.2],
                         col = [0, 1, 0],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([0, 0, 0.6], [0, 0, 2.6]),
                         rad = 0.1,
                         col = [0, 1, 0],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([1.25, 0, 0], [3.25, 0, 0]),
                         rad = 0.1,
                         col = [0, 1, 0],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([0, 0.75, 0], [0, 2.75, 0]),
                         rad = 0.1,
                         col = [0, 1, 0],
                         tran = 0.3,
                     )

    x3d_chassis[3] = EoM_X3D.x3d_pnt(
                         [0, 0, 0],
                         cubes = true,
                         rad = [2.5, 1.5, 1.2],
                         col = [0, 0, 1],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([0, 0, 0.6], [0, 0, 2.6]),
                         rad = 0.1,
                         col = [0, 0, 1],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([1.25, 0, 0], [3.25, 0, 0]),
                         rad = 0.1,
                         col = [0, 0, 1],
                         tran = 0.3,
                     ) * EoM_X3D.x3d_cyl(
                         hcat([0, 0.75, 0], [0, 2.75, 0]),
                         rad = 0.1,
                         col = [0, 0, 1],
                         tran = 0.3,
                     )

    # Build string for each winner
    for i in 1:nt
        pstn = "" # position history
        rntn = "" # rotation history
        tme = ""

        # load data
        grp = nums_win[i] # student numbers for each winner
        folder = "$(grp[1])_$(grp[2])" # locate the corresponding folder according to winners' ID
        time_hstr = readdlm(joinpath("results", folder, "time_history.txt")) # read the data
        tout = time_hstr[:, 1] # real time
        lcn = time_hstr[:, 2:4]' # location
        rtn = time_hstr[:, 5:7]' # rotation

        time = tout
        time /= laptime[1] # divide the real time by the laptime to switch to key format that required in x3dom
        m = length(time) # number of points in the x3dom

        # writing the string for location history
        for i in 1:m
            pstn *= "$(lcn[1,i]) $(lcn[2,i]) $(lcn[3,i]),\n" # string for each key point (moment)

            roll = rtn[1, i]
            pitch = rtn[2, i]
            yaw = rtn[3, i]
            c1 = cos(yaw / 2)
            s1 = sin(yaw / 2)
            c2 = cos(pitch / 2)
            s2 = sin(pitch / 2)
            c3 = cos(roll / 2)
            s3 = sin(roll / 2)
            c1c2 = c1 * c2
            s1s2 = s1 * s2
            angle = 2 * acos(c1c2 * c3 + s1s2 * s3)
            x = c1c2 * s3 - s1s2 * c3
            y = c1 * s2 * c3 + s1 * c2 * s3
            z = s1 * c2 * c3 - c1 * s2 * s3

            norm = x * x + y * y + z * z
            if norm < 1e-5
                x = 0
                y = 1
                z = 0
            else
                norm = sqrt(norm)
                x /= norm
                y /= norm
                z /= norm
            end

            rntn *= "$x $y $z $angle,\n" # final format for presenting a rotating motion in x3dom
        end

        pstn = chop(pstn, tail = 2)
        rntn = chop(rntn, tail = 2)

        # Define Key
        for i in 1:m-1
            tme *= "$(time[i])," # "Key"
        end
        tme *= "$(time[m])"

        # writing the chassis strings in the format of x3dom interpolation
        s =
            "<PositionInterpolator DEF='chassis_psn_$(i)' keyValue='\n" *
            pstn *
            "'\nkey='" *
            tme *
            "'></PositionInterpolator>\n"
        s *=
            "<OrientationInterpolator DEF='chassis_rtn_$(i)' keyValue='\n" *
            rntn *
            "'\nkey='" *
            tme *
            "'></OrientationInterpolator>\n"

        s *= "<Transform DEF='chassis_$(i)' >\n"
        s *= x3d_chassis[i]
        s *= "</Transform>\n"

        s *= "<ROUTE fromNode='IDtimer' fromField='fraction_changed' toNode='chassis_psn_$(i)' toField='set_fraction'></ROUTE>\n"
        s *= "<ROUTE fromNode='IDtimer' fromField='fraction_changed' toNode='chassis_rtn_$(i)' toField='set_fraction'></ROUTE>\n"
        s *= "<ROUTE fromNode='chassis_psn_$(i)' fromField='value_changed' toNode='chassis_$(i)' toField='set_translation'></ROUTE>\n"
        s *= "<ROUTE fromNode='chassis_rtn_$(i)' fromField='value_changed' toNode='chassis_$(i)' toField='set_rotation'></ROUTE>\n"

        s_total *= s
    end

    # writing the ground strings in the format of x3dom interpolation
    s_total = x3d_grnd * s_total

    # writing the track map strings in the format of x3dom interpolation
    for i in 1:n
        tck *= "$(track[i,1]) $(track[i,3]) $(-track[i,2]),\n"
    end
    idx = hcat(0:1:n-1)'
    for i in 1:n
        tck_idx *= "$(idx[i]), "
    end

    s_tck *= "<Transform rotation = '1 0 0 1.57'>\n"
    s_tck *= " <Shape>\n"
    s_tck *= "<IndexedLineSet coordIndex='\n" * tck_idx * "'>\n"
    s_tck *= "<Coordinate point='\n" * tck * "'>\n"
    s_tck *= "</IndexedLineSet>\n"
    s_tck *= " </Shape>\n"
    s_tck *= "</Transform>\n"

    s_total = s_tck * s_total

    # make the folder to save the animation
    dir = joinpath("results", "competition_x3d")
    ~isdir(dir) && (mkdir(dir))

    # call x3d_save function to finalize the file generation
    EoM_X3D.x3d_save(s_total, joinpath(dir, filename), laptime[end])

    println("Animation written.")

end


# s_mtx[i] = s # Save the sttring to the string matrix
# s_mtx = ["" "" ""]
# s_mtx[3] *= x3d_grnd
# x3d_save(s_mtx, s_tck, joinpath(dir, filename), laptime)


# # Roll Pitch Yaw
# roll = rtn[1, i]
# pitch = rtn[2, i]
# yaw = rtn[3, i]
# s_roll = sin(roll)
# c_roll = cos(roll)
# s_pitch = sin(pitch)
# c_pitch = cos(pitch)
# s_yaw = sin(yaw)
# c_yaw = cos(yaw)

# # Rotation Matrix that required to find euler's parameters
# S_ao = [
#     c_pitch*c_yaw c_roll*s_yaw+s_roll*s_yaw*s_pitch s_yaw*s_roll-c_roll*s_pitch*c_yaw
#     -c_pitch*s_yaw c_roll*c_yaw-s_roll*s_yaw*s_pitch s_roll*c_yaw+c_roll*s_pitch*s_yaw
#     s_pitch -s_roll*c_pitch c_roll*c_pitch
# ]
# trS = S_ao[1, 1] + S_ao[2, 2] + S_ao[3, 3]

# # Euler's parameters
# eu4 = sqrt((trS + 1) / 4)
# eu1 = (S_ao[3, 2] - S_ao[2, 3]) / (4 * eu4)
# eu2 = (S_ao[1, 3] - S_ao[3, 1]) / (4 * eu4)
# eu3 = (S_ao[2, 1] - S_ao[1, 2]) / (4 * eu4)

# # Absolute 3D rotation angle and its axis unit vector, this is the format to represent a 3D rotation in x3dom
# theta = 2 * (acos(eu4)) # absolute angle
# lam1 = eu1 / (sin(theta / 2)) # unit vector of rotation axis
# if abs(lam1) == 0 # delete the negative values
#     lam1 = 0
# end
# lam2 = eu2 / (sin(theta / 2)) # unit vector of rotation axis
# if abs(lam2) == 0
#     lam2 = 0
# end
# lam3 = -eu3 / (sin(theta / 2)) # unit vector of rotation axis
# if abs(lam3) == 0
#     lam3 = 0
# end





