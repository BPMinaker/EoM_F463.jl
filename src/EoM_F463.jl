module EoM_F463

using OrdinaryDiffEq
using DelimitedFiles
using Dates
using Interpolations
using Roots
using Polynomials
using LinearAlgebra
using Plots

using EoM
using EoM_X3D

export prelim!
export track!
export build_model!
export solver
export plot_results
export props
export susp
export input_F463
export F463_animate

include("solver.jl") # high level solver
include("prelim.jl") # compute shift points
include("eqn_of_motion.jl") # equations of motion, driver model
include("tire_comb.jl") # tire model
include("track.jl")  # define the track
include("track_offset.jl") # track location
include("plot_results.jl") # plot results
include("F463_animate.jl") # the function to save the animation of time history

include("vehicle_specs.jl") # this will define the custom variable type called 'props' that holds all the default values of the vehicle, driver, and track parameters

include("input_F463.jl")
include("build_model.jl")
end # module
