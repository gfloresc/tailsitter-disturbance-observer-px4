# Aerodynamic wrench on the vehicle measured in Gazebo (IjcAeroSystem): all external forces and
# moments applied in the physics step except gravity, contacts and the rotor actuation (thrust
# and rotor drag torque). Ground truth for the disturbance observers of ijc_control.

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # simulation time of the physics step (microseconds)
float32[3] force_ned      # force, NED frame [N]
float32[3] torque_frd     # moment about the CoM, body FRD frame [N m]
