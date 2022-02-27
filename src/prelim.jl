function prelim!(params::EoM_F463.props, verbose::Bool = false)

    # which engine did the student specify?
    if params.engine_type == 1
        params.revs = (2000.0/3.0):(2000.0/3.0):(8000.0+2000.0/3.0)
        params.torque = [98.0, 124.0, 146.0, 195.0, 220.0, 225.0, 220.0, 248.0, 254.0, 244.0, 230.0, 209.0, 186.0]
    elseif params.engine_type == 2
        params.revs = 500.0:500.0:6500.0
        params.torque = [156.0, 176.0, 215.0, 299.0, 319.0, 345.0, 332.0, 312.0, 312.0, 299.0, 280.0, 234.0, 169.0]
    else
        error("Invalid engine type")
    end
    params.redline = params.revs[end]

    te = CubicSplineInterpolation(params.revs, params.torque)

    verbose && display("Computing shift speeds...")

    if params.drive == "RearWheelDrive"
        params.wtf = 1
    elseif params.drive == "FrontWheelDrive"
        params.wtf = -1
    else
        params.wtf = 0
    end

    params.launchrpm = 0.5 * params.redline

    if length(params.e) == 5
        params.eff = 0.95
    else
        params.eff = 0.90
    end

    params.vmax = 0.95 * (params.redline * pi / 30 * (params.rad) / params.ex) ./ params.e #find redline speeds

    verbose && display("Speeds in each gear at redline (5% tire slip) [kph]:")
    verbose && display(round.(params.vmax * 3.6, digits = 2))
    verbose && display("Speeds range of each gear [kph]:")
    verbose && display(round.(diff(params.vmax * 3.6), digits = 2))

    for i in 1:length(params.e)-1
        at_bs = params.torque[end] * params.e[i] # axle torque before shift at redline
        at_as = te(params.revs[end] * params.e[i+1] / params.e[i]) * params.e[i+1] # axle torque after shift at new engine speed

        if at_as > at_bs
            verbose && display("Short shift needed in gear $i.  Calculating shift speed...") # change shift speed
            dat(ws) = te(ws) * params.e[i] - te(ws * params.e[i+1] / params.e[i]) * params.e[i+1]
            shift_rpm = find_zero(dat, params.revs[end-1])
            params.vmax[i] = shift_rpm * pi / 30 * params.rad / params.ex / params.e[i] / 1.05
        end
    end

    verbose && display("Shift speeds for maximum acceleration [kph]:")
    verbose && display(round.(params.vmax * 3.6, digits = 2))
    verbose && display("Speeds range of each gear [kph]:")
    verbose && display(round.(diff(params.vmax * 3.6), digits = 2))

    w = params.revs[1]:10.0:params.redline
    temp = zeros(length(w), 2 * length(params.e))  # empty space

    for n in 1:length(params.e)
        u = w * 2pi * 60 * params.rad / 1.05 / params.ex / params.e[n] / 1000  # find speed for each engine speed
        xt = te.(w) * params.e[n] * params.ex * params.eff / params.rad  # find traction for each engine speed
        temp[:, 2n-1] = u  #stor it
        temp[:, 2n] = xt
    end

    plot(temp[:, 1], temp[:, 2], label = "1", size = (900, 600))
    for n in 2:length(params.e)-1
        plot!(temp[:, 2*n-1], temp[:, 2*n], label = "$n")
    end
    p = plot!(temp[:, 2*length(params.e)-1], temp[:, 2*length(params.e)], xlabel = "Speed [km/h]", ylabel = "Traction force [N]", label = "$(length(params.e))")

    p
end
