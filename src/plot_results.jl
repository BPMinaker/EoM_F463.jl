function plot_results(sol, yout, p, group_ID, params; disp = true)

    plotly()

    println("Ran simulation. Plotting results...")

    out = "results"
    ~isdir(out) && (mkdir(out))

    tmstr = "$(group_ID[1])_$(group_ID[2])"
    out = joinpath("results", tmstr)
    ~isdir(out) && (mkdir(out))

    # combine the results into a matrix
    res = hcat(sol.u...)
    res2 = hcat(yout...)

    x = params.r_orth * res[7:20, :]
    w = params.r_orth * res[21:end, :]

    x[3, :] .+= params.hg

    writedlm(joinpath(out, "time_history.txt"), [sol.t res[1:2, :]' x[3:5, :]' res[3, :]])

    savefig(p, joinpath(out, "perf.html"))
    disp && display(p)

    p = plot(
        sol.t,
        x[1:3, :]',
        xlabel = "Time [s]",
        ylabel = "Chassis motion [m]",
        xlims = (0, Inf),
        label = ["x" "y" "z"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "chassis loc.html"))
    disp && display(p)

    p = plot(
        sol.t,
        180 / pi * x[4:5, :]',
        xlabel = "Time [s]",
        ylabel = "Chassis motion [deg]",
        xlims = (0, Inf),
        label = ["ϕ" "θ"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "chassis ang.html"))
    disp && display(p)

    p = plot(
        sol.t,
        180 / pi * atan.(w[2, :], w[1, :]),
        xlabel = "Time [s]",
        ylabel = "β [deg]",
        xlims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "body slip.html"))
    disp && display(p)

    p = plot(
        sol.t,
        w[4:6, :]',
        xlabel = "Time [s]",
        ylabel = "Chassis angular velocity [rad/s]",
        xlims = (0, Inf),
        label = ["p" "q" "r"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "chassis ang vel.html"))
    disp && display(p)

    plot(
        params.course[:, 1],
        params.course[:, 2],
        label = "Target path",
        aspect_ratio = :equal,
        size = (900, 600),
    )
    p = plot!(
        res[1, :],
        res[2, :],
        xlabel = "x [m]",
        ylabel = "y [m]",
        label = "Frame Path",
    )
    savefig(p, joinpath(out, "path.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res[4, :],
        xlabel = "Time [s]",
        ylabel = "Distance [m]",
        xlims = (0, Inf),
        ylims = (0, 2500),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "distance.html"))
    disp && display(p)

    p = plot(
        sol.t,
        180 / pi * res[3, :],
        xlabel = "Time [s]",
        ylabel = "ψ [deg]",
        xlims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "yaw.html"))
    disp && display(p)

    p = plot(
        sol.t,
        180 / pi * res[5, :],
        xlabel = "Time [s]",
        ylabel = "δ [deg]",
        xlims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "steer.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res[6, :],
        xlabel = "Time [s]",
        ylabel = "Throttle/Brake",
        xlims = (0, Inf),
        ylims = (-1, 1),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "throttle.html"))
    disp && display(p)

    p = plot(
        sol.t,
        3.6 * [w[1, :] res2[1, :]],
        xlabel = "Time [s]",
        ylabel = "Forward, Target speeds [km/h]",
        xlims = (0, Inf),
        ylims = (0, Inf),
        label = ["u" "u Target"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "speed.html"))
    disp && display(p)

    p = plot(
        sol.t,
        w[6*params.wheelnum.-1, :]',
        xlabel = "Time [s]",
        ylabel = "Wheel speeds [rad/s]",
        xlims = (0, Inf),
        ylims = (0, Inf),
        label = ["LF" "LR" "RF" "RR"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "wheel_speed.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[4, :],
        xlabel = "Time [s]",
        ylabel = "Engine speed [rpm]",
        xlims = (0, Inf),
        ylims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "rpm.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[5, :],
        xlabel = "Time [s]",
        ylabel = "Gear",
        xlims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "gear.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[2, :],
        xlabel = "Time [s]",
        ylabel = "Path error [m]",
        xlims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "path_err.html"))
    disp && display(p)

    p = plot(
        sol.t,
        180 / pi * res2[3, :],
        xlabel = "Time [s]",
        ylabel = "Heading error [deg]",
        xlims = (0, Inf),
        label = "",
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "heading_err.html"))
    disp && display(p)

    p = plot(
        sol.t,
        180 / pi * res2[6:9, :]',
        xlabel = "Time [s]",
        ylabel = "Camber [deg]",
        xlims = (0, Inf),
        label = ["LF" "LR" "RF" "RR"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "camber.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[10:13, :]',
        xlabel = "Time [s]",
        ylabel = "Long forces [N]",
        xlims = (0, Inf),
        label = ["LF" "LR" "RF" "RR"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "long_frc.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[14:17, :]',
        xlabel = "Time [s]",
        ylabel = "Lateral forces [N]",
        xlims = (0, Inf),
        label = ["LF" "LR" "RF" "RR"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "lat_frc.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[18:21, :]',
        xlabel = "Time [s]",
        ylabel = "Vertical forces [N]",
        xlims = (0, Inf),
        ylims = (0, Inf),
        label = ["LF" "LR" "RF" "RR"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "vert_frc.html"))
    disp && display(p)

    p = plot(
        sol.t,
        res2[22:25, :]',
        xlabel = "Time [s]",
        ylabel = "Tire slip []",
        xlims = (0, Inf),
        ylims = (0, 0.1),
        label = ["LF" "LR" "RF" "RR"],
        xticks = 0:5:sol.t[end],
        size = (900, 600),
    )
    savefig(p, joinpath(out, "tire_slip.html"))
    disp && display(p)

    out
end


#=
p=plot(sol.t,(res2[2,:].+res[7,:].*res[12,:])/9.81,xlabel="Time [s]",ylabel="Lateral accl'n [g]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"lat_acc.html"))
disp && display(p)

p=plot(sol.t,(res2[1,:].-res[8,:].*res[12,:])/9.81,xlabel="Time [s]",ylabel="Long accl'n [g]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"long_acc.html"))
disp && display(p)

=#
