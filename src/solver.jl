# define function to decide when to stop simulation, stops when value crosses zero
function dist_to_end(z, t, integrator)

    # if the distance travelled is too short, stop when travelling backward, when close to end, use x coordinate (finish line)

    #println("t: ",t)
    #println("z[1]: ", z[1])
    #println("z[4]: ", z[4])

    if z[4] < 2200
        value = -z[4]
    else
        value = z[1]
    end

    value

end

# function to launch ode solver
function solver(params::EoM_F463.props)

    println("Gathered data. Simulating...")

    # set max end time and plot interval
    tend = 175
    tspan = (0.0, tend)
    x0 = zeros(6 + 28)

    # define function that contains the equations to solve, and solve them, but don't return extra data until after solution complete
    cb = ContinuousCallback(dist_to_end, terminate!, interp_points = 1000)
    prob = ODEProblem(eqn_of_motion, x0, tspan, params)

    sol = solve(
        prob,
        Tsit5(),
        saveat = 0.1,
        dt = 1.e-3,
        dtmax = 0.01,
        callback = cb,
        progress = true,
    )

    # recalculate extra derivative data (accelerations)
    # define empty vector of vectors, call ode function
    yout = [zeros(25) for i in sol.t]
    for i in 1:length(sol.t)
        eqn_of_motion(yout[i], sol.u[i], params, sol.t[i], flag = true)
    end

    sol, yout
end
