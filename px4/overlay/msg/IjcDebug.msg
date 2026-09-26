# Debug/log topic of the ijc_control module (observer-based tail-sitter controller).
# Frames: NED inertial, FRD body. Disturbances are expressed as accelerations:
# F in m/s^2 (F = F_ext/m, inertial), T in rad/s^2 (T = J^-1 tau_ext, body).

uint64 timestamp               # time since system start (microseconds)
uint64 timestamp_sample        # timestamp of the angular velocity sample (microseconds)

uint8 STATE_IDLE = 0
uint8 STATE_TAKEOFF = 1
uint8 STATE_HOVER = 2
uint8 STATE_LAND = 3
uint8 STATE_TOUCHDOWN = 4
uint8 state                    # module state machine

bool comp                      # disturbance compensation active in the control laws (IJC_COMP)
bool observers_active          # observers running (false: held in reset)
float32 t_scen                 # scenario time [s] (-1 if not started)

float32[3] p                   # position used by the controller [m]
float32[3] v                   # velocity used by the controller [m/s]
float32[3] p_d                 # desired position [m]
float32[3] v_d                 # desired velocity [m/s]
float32[3] a_d                 # desired (feedforward) acceleration [m/s^2]
float32[3] e_p                 # p - p_d [m]
float32[3] e_v                 # v - v_d [m/s]

float32[3] v_hat               # translational observer state [m/s]
float32[3] f_hat               # estimated force disturbance F_hat [m/s^2]
float32[3] omega               # body angular velocity used by the controller [rad/s]
float32[3] omega_hat           # rotational observer state [rad/s]
float32[3] t_hat               # estimated torque disturbance T_hat [rad/s^2]

float32[4] q                   # attitude quaternion (w,x,y,z), body FRD -> NED
float32[4] q_d                 # desired attitude quaternion
float32[4] q_bar               # attitude error Q_d^-1 * Q (w >= 0)

float32[3] mu                  # position control vector [N] (after safeguards)
float32 f                      # commanded collective thrust [N]
float32[3] tau                 # commanded torque [N m]
float32 f_applied              # applied thrust (after allocation saturation) [N]
float32[3] tau_applied         # applied torque (after allocation saturation) [N m]
float32 f_model                # applied thrust through the actuator model (used by the observers) [N]
float32[3] tau_model           # applied torque through the actuator model (used by the observers) [N m]
float32 thrust_norm            # normalized thrust sent to the allocator [0,1]
float32[3] torque_norm         # normalized torque sent to the allocator [-1,1]

float32[3] f_lat_ff            # canted-rotor lateral force feedforward added to mu [N] (NED)
uint8 mode                     # IJC_MODE: 0 ijc_control in the loop, 1 standard PX4 controllers

float32[3] f_applied_dist      # injected force disturbance F [m/s^2] (NED)
float32[3] t_applied_dist      # injected torque disturbance T [rad/s^2] (FRD)

float32 dt_att                 # attitude/rotational observer step [s]
float32 dt_pos                 # position/translational observer step [s]
