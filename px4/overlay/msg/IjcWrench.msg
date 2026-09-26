# External wrench requested by ijc_control for disturbance injection in simulation.
# Forwarded by gz_bridge to the IjcWrenchSystem Gazebo plugin (applied at the CoM).

uint64 timestamp          # time since system start (microseconds)
float32[3] force_ned      # force in the inertial NED frame [N]
float32[3] torque_frd     # torque in the body FRD frame [N m]
