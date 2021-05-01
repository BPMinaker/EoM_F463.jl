function tire_comb(slip, load, ratio, camber, params)

    one_minus_ratio = 1.0 .- ratio

    C = params.a_mtm[1] * ratio + params.b_mtm[1] * one_minus_ratio
    mu_p =
        (params.a_mtm[2] * ratio + params.b_mtm[2] * one_minus_ratio) .* load +
        (params.a_mtm[3] * ratio + params.b_mtm[3] * one_minus_ratio)
    D = mu_p .* load

    Ea = params.a_mtm[7] * load .+ params.a_mtm[8]
    Eb = params.b_mtm[7] * load .^ 2 .+ params.b_mtm[8] * load .+ params.b_mtm[9]
    E = ratio .* Ea + one_minus_ratio .* Eb

    Ba = params.a_mtm[4] * sin.(2 * atan.(load / params.a_mtm[5])) .* (1.0 .- params.a_mtm[6] * abs.(camber)) ./ C ./ D
    Bb = (params.b_mtm[4] * load .^ 2 + params.b_mtm[5] * load) .* exp.(-params.b_mtm[6] * load) ./ C ./ D
    B = ratio .* Ba + one_minus_ratio .* Bb

    trac = D .* sin.(C .* atan.(B .* (1.0 .- E) .* slip .+ E .* atan.(B .* slip)))

    trac

end  ## Leave

#Sh=params.b_mtm(10)*load/1000+params.b_mtm(11);
