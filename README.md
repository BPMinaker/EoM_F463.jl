# EoM_F463.jl

The F463 laptime simulator, integrated into EoM eqn of motion generator.  Uses a 14 degree of freedom model (6 chassis motions, 4 suspension motions, 4 wheel rotations).
The vehicle model includes a double A-arm suspension with massless arms, but the upright has mass to include the wheel hop effect, and the wheel has rotary inertia.
The suspension uses a spring between the lower A-arm and the chassis, and a torsional anti-roll bar.
The kinematics are linearized, but allows inclusion of roll centre/jacking force effects, bump/roll steer, and camber change.
The vehicle model uses a nonlinear tire model with coupling between vertical, lateral, and longitudinal forces.
The engine model is nonlinear, and so is the aero load.
The driver is automated and chooses a target speed based on requested lateral acceleration, and steers based on road curvature, heading error, and offset error.
Both inputs use a low pass filter before acting on the vehicle model.


