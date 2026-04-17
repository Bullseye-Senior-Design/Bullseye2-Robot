import numpy as np
import casadi as ca
from scipy.interpolate import interp1d
import time
import threading
import logging
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from structure.Subsystem import Subsystem
from Robot.Constants import Constants
import rsplan
import math
from enum import Enum

logger = logging.getLogger(f"{__name__}.PathFollowing")
logger.setLevel(logging.DEBUG)

class DriveDirection(Enum):
    FORWARD = "forward"
    REVERSE = "reverse"

class PathFollowing(Subsystem):
    """Model Predictive Control Navigator for path following.
    
    This class wraps the MPC solver and provides a threaded interface for
    continuous path following using feedback from the KalmanStateEstimator.
    """
    
    _instance = None

    def __new__(cls):
        """Singleton pattern: return the same instance every time."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.start()
        return cls._instance
    
    @classmethod
    def reset_instance(cls):
        """Reset the singleton instance and return a fresh one. Call before restart."""
        if cls._instance is not None:
            cls._instance.stop_path_following()
            cls._instance = None
            logger.info("PathFollowing singleton reset")
        # Return new instance
        return cls()
    
    def start(self):
        """Initialize MPC Navigator with default parameters."""
        super().__init__()
        # ────────────────────────────────────────────────
        # Parameters & Constants
        # ────────────────────────────────────────────────
        self.Ts = 0.3  # MPC sampling time (seconds)
        self.p = 25 # 12 may be better for computation capacity
        self.L = Constants.wheel_base_width  # Wheelbase of the robot (meters) - must be set in Constants.py
        self.clutch_disengage_threshold = Constants.inside_clutch_angle_threshold_rads
        # crusing speed for reference trajectory generation, can be adjusted via set_nominal_speed() method
        self.v_nom = Constants.rear_motor_top_speed*0.4
        
        # Track direction as enum (not bool/string)
        self._drive_direction = DriveDirection.FORWARD
        
        # CHANGED: Use abs() to ensure positive arc-length steps even if driving in reverse
        self.ds = abs(self.v_nom) * self.Ts
        self.ds_ref = abs(self.v_nom) * self.Ts  
        
        # Weights (Q for state, R for input, Rd for rate of change, V for speed tracking)
        # CHANGED: Lowered lateral weight slightly so it doesn't fight the forward progress as violently
        self.Q_diag = np.array([10.0, 15.0]) # Cross Track and heading
        self.Q_terminal_diag = np.array([0.0, 0.0, 0.0]) # [lateral, heading, longitudinal] terminal (last point) weights
        self.R_diag = np.array([0.1, 1.0]) # Discourages large values
        self.Rd_diag = np.array([3.0, 15.0]) # Smoothness 
        self.V_weight = 20.0  
        
        # Constraints
        # CHANGED: Default to forward-only to prevent backwards/forwards oscillation traps
        self.lower_speed_limit = 0.5
        self.upper_speed_limit = 0.6
        self.v_bounds = [Constants.rear_motor_top_speed*self.lower_speed_limit, Constants.rear_motor_top_speed*self.upper_speed_limit]
        self.delta_bounds = [-Constants.steering_angle_limit_rads, Constants.steering_angle_limit_rads]

        # State bounds
        self.lbx = np.array([-np.inf, -np.inf, -np.inf] * (self.p + 1) + 
                            [self.v_bounds[0], self.delta_bounds[0]] * self.p)
        self.ubx = np.array([np.inf, np.inf, np.inf] * (self.p + 1) + 
                            [self.v_bounds[1], self.delta_bounds[1]] * self.p)
        
        # Setup MPC solver
        self.solver, self.n_states, self.n_controls = self._setup_mpc()
        
        # Constraint bounds (g=0 for initial state and dynamics)
        self.lbg = np.zeros((self.p + 1) * self.n_states)
        self.ubg = np.zeros((self.p + 1) * self.n_states)
        
        # Path matrix
        self.path_matrix = None
        
        # Thread control
        self._running = False
        self._thread = None
        self._lock = threading.RLock()
        
        # Current commands (output)
        self._v_cmd = 0.0
        self._delta_cmd = 0.0
        
        # MPC state
        self._last_u = np.array([0.0, 0.0])
        self._x_prev = None  # For warm starting
        
        # Path completion tracking
        self.closest_point_idx = None  # To track closest point on path for determining if the path is finished
        
        # Get reference to state estimator
        self.state_estimator = KalmanStateEstimator()
    
    def _setup_mpc(self):
        """Setup the MPC solver using CasADi with Frenet Frame cost function."""
        # Symbolic states
        x = ca.SX.sym('x') # type: ignore
        y = ca.SX.sym('y') # type: ignore
        theta = ca.SX.sym('theta') # type: ignore
        states = ca.vertcat(x, y, theta)
        n_states = states.size1()
        
        # Symbolic inputs
        v = ca.SX.sym('v') # type: ignore
        delta = ca.SX.sym('delta') # type: ignore
        controls = ca.vertcat(v, delta)
        n_controls = controls.size1()
        
        # Hybrid kinematic model:
        # - Bicycle model when both rear clutches are engaged (small steering)
        # - Tricycle model when one rear clutch is disengaged (larger steering)
        omega_bicycle = (v / self.L) * ca.tan(delta)
        omega_tricycle = (v / self.L) * ca.sin(delta) # simplified tricycle model for larger steering angles
        use_tricycle = ca.fabs(delta) > self.clutch_disengage_threshold
        omega = ca.if_else(use_tricycle, omega_tricycle, omega_bicycle)
        rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)
        f = ca.Function('f', [states, controls], [rhs])
        
        # Optimization variables
        U = ca.SX.sym('U', n_controls, self.p) # type: ignore
        X = ca.SX.sym('X', n_states, self.p + 1) # type: ignore
        # Parameters: initial state (3) + prev control (2) + pose refs (3*(p+1)) + speed refs (p+1)
        P = ca.SX.sym('P', n_states + n_controls + (self.p+1)*3 + (self.p+1)) # type: ignore
        
        cost_fn = 0
        g = []
        
        # Unpack parameters
        x_init = P[0:3]
        u_prev = P[3:5]
        pose_ref_end = 5 + (self.p+1)*3
        ref_traj = ca.reshape(P[5:pose_ref_end], 3, self.p+1)
        v_ref = P[pose_ref_end:pose_ref_end + self.p + 1]
        
        # Initial state constraint
        g.append(X[:, 0] - x_init)
        
        for k in range(self.p):
            st = X[:, k]
            con = U[:, k]
            ref_pose = ref_traj[:, k]
            
            # Compute cross-track error (Frenet Frame)
            dx = st[0] - ref_pose[0]
            dy = st[1] - ref_pose[1]
            ref_theta = ref_pose[2]
            
            e_lateral = -dx * ca.sin(ref_theta) + dy * ca.cos(ref_theta)
            e_heading = st[2] - ref_pose[2]
            e_heading = ca.atan2(ca.sin(e_heading), ca.cos(e_heading))
            
            cost_fn += self.Q_diag[0] * e_lateral**2
            cost_fn += self.Q_diag[1] * e_heading**2
            
            # Input effort cost
            cost_fn += ca.mtimes([con.T, np.diag(self.R_diag), con])
            
            # Speed tracking cost (track desired speed reference)
            cost_fn += self.V_weight * (con[0] - v_ref[k])**2
            
            # Smoothness cost
            u_compare = u_prev if k == 0 else U[:, k-1]
            cost_fn += ca.mtimes([(con - u_compare).T, np.diag(self.Rd_diag), 
                                  (con - u_compare)])
            
            # Dynamics constraint
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + (self.Ts * f_value) # type: ignore
            g.append(st_next - st_next_euler)
        
        # Terminal cost (Frenet Frame) - uses higher weights for goal emphasis
        dx_term = X[0, self.p] - ref_traj[0, self.p]
        dy_term = X[1, self.p] - ref_traj[1, self.p]
        ref_theta_term = ref_traj[2, self.p]
        
        # CHANGED: Added Longitudinal Error Calculation to pull the robot to the end of the horizon
        e_lateral_term = -dx_term * ca.sin(ref_theta_term) + dy_term * ca.cos(ref_theta_term)
        e_longitudinal_term = dx_term * ca.cos(ref_theta_term) + dy_term * ca.sin(ref_theta_term) 
        
        e_heading_term = X[2, self.p] - ref_traj[2, self.p]
        e_heading_term = ca.atan2(ca.sin(e_heading_term), ca.cos(e_heading_term))
        
        cost_fn += self.Q_terminal_diag[0] * e_lateral_term**2
        cost_fn += self.Q_terminal_diag[1] * e_heading_term**2
        cost_fn += self.Q_terminal_diag[2] * e_longitudinal_term**2 # CHANGED: Now utilizing index 2!
        
        # Reshape for solver
        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        
        nlp_prob = {
            'f': cost_fn,
            'x': opt_vars,
            'g': ca.vertcat(*g),
            'p': P
        }
        
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.warm_start_init_point': 'yes', # Tells the solver to actively use that previous state to initialize its next guess. 
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.max_iter': 100
        }
        
        return ca.nlpsol('solver', 'ipopt', nlp_prob, opts), n_states, n_controls
    
    def _generate_reference(self, cur_state):
        """Generate reference trajectory from path matrix using fixed arc-length spacing."""
        if self.path_matrix is None:
            return np.zeros((self.p + 1, 3))
        
        x_wp = self.path_matrix[:, 0]
        y_wp = self.path_matrix[:, 1]
        theta_wp = self.path_matrix[:, 2]
        if len(x_wp) < 2:
            # Not enough points to form a path segment; hold current waypoint pose.
            ref_pose = np.array([x_wp[0], y_wp[0], theta_wp[0]], dtype=float) if len(x_wp) == 1 else np.zeros(3)
            return np.tile(ref_pose, (self.p + 1, 1))
        
        # Compute cumulative arc-length of waypoints
        dx, dy = np.diff(x_wp), np.diff(y_wp)
        s_wp = np.cumsum(np.sqrt(dx**2 + dy**2))
        s_wp = np.insert(s_wp, 0, 0.0)

        # Remove non-increasing arc-length samples caused by duplicate/near-duplicate waypoints.
        keep = np.concatenate(([True], np.diff(s_wp) > 1e-9))
        s_wp = s_wp[keep]
        x_wp = x_wp[keep]
        y_wp = y_wp[keep]
        theta_wp = theta_wp[keep]

        if len(s_wp) < 2:
            ref_pose = np.array([x_wp[0], y_wp[0], theta_wp[0]], dtype=float)
            return np.tile(ref_pose, (self.p + 1, 1))

        # Cubic requires >=4 points; linear is robust for short segments.
        interp_kind = 'cubic' if len(s_wp) >= 4 else 'linear'
        
        # Create interpolators for x, y, and theta as functions of arc-length
        interp_x = interp1d(s_wp, x_wp, kind=interp_kind, fill_value='extrapolate') # type: ignore
        interp_y = interp1d(s_wp, y_wp, kind=interp_kind, fill_value='extrapolate') # type: ignore
        interp_theta = interp1d(s_wp, theta_wp, kind=interp_kind, fill_value='extrapolate') # type: ignore
        
        # Find closest point on path to current state
        distances = np.sqrt((x_wp - cur_state[0])**2 + (y_wp - cur_state[1])**2)
        closest_idx = np.argmin(distances)
        logger.debug(f"Closest waypoint: x={x_wp[closest_idx]:.2f}, y={y_wp[closest_idx]:.2f}, theta={theta_wp[closest_idx]:.2f} rad")
        self.closest_point_idx = closest_idx  # Store closest point index for potential use in determining path completion
        
        # Interpolate arc-length at robot's position for better accuracy
        if closest_idx == len(x_wp) - 1:
            s_cur = s_wp[-1]
        else:
            # Linear interpolation between two closest waypoints
            sum_distances = distances[closest_idx] + distances[closest_idx + 1]
            alpha = distances[closest_idx] / sum_distances if sum_distances > 0 else 0
            s_cur = s_wp[closest_idx] * (1 - alpha) + s_wp[closest_idx + 1] * alpha
        
        # Fixed arc-length spacing (independent of speed changes)
        ref = np.zeros((self.p + 1, 3))
        for i in range(self.p + 1):
            s_f = min(s_cur + i * self.ds_ref, s_wp[-1])
            ref[i, :] = [interp_x(s_f), interp_y(s_f), interp_theta(s_f)]
        return ref
    
    def set_path(self, path_matrix: np.ndarray):
        """Set the path to follow."""
        with self._lock:
            self.path_matrix = np.asarray(path_matrix, dtype=float)
            
    def _update_solver_bounds(self):
        """Rebuild optimization bounds using current velocity and steering limits."""
        self.lbx = np.array(
            [-np.inf, -np.inf, -np.inf] * (self.p + 1) +
            [self.v_bounds[0], self.delta_bounds[0]] * self.p
        )
        self.ubx = np.array(
            [np.inf, np.inf, np.inf] * (self.p + 1) +
            [self.v_bounds[1], self.delta_bounds[1]] * self.p
        )
            
    def set_drive_direction(self, direction: DriveDirection) -> bool:
        """Set motion direction constraints using DriveDirection enum."""
        with self._lock:
            if not isinstance(direction, DriveDirection):
                logger.warning("Invalid direction '%s'. Use DriveDirection.FORWARD or DriveDirection.REVERSE.", direction)
                return False

            self._drive_direction = direction

            # Keep speed magnitude, flip sign to match direction
            speed_mag = abs(self.v_nom)
            self.v_nom = speed_mag if direction == DriveDirection.FORWARD else -speed_mag

            if direction == DriveDirection.FORWARD:
                self.v_bounds = [Constants.rear_motor_top_speed*self.lower_speed_limit, Constants.rear_motor_top_speed*self.upper_speed_limit]
            else:
                self.v_bounds = [-Constants.rear_motor_top_speed*self.upper_speed_limit, -Constants.rear_motor_top_speed*self.lower_speed_limit]

            self.ds = abs(self.v_nom) * self.Ts
            self.ds_ref = abs(self.v_nom) * self.Ts
            self._update_solver_bounds()

            logger.debug("Drive direction set to %s (v_nom=%.3f)", self._drive_direction.value, self.v_nom)
            return True
    
    def set_nominal_speed(self, speed_percent):
        """Set the desired nominal speed for path following.
        
        Args:
            speed_percent: Speed as a percentage of maximum (0-100).
                          Negative values indicate reverse direction.
        """
        with self._lock:
            clamped_percent = np.clip(speed_percent, -100, 100)
            self.v_nom = (clamped_percent / 100.0) * Constants.rear_motor_top_speed
            
            # Keep enum direction synchronized with speed sign
            self._drive_direction = (
                DriveDirection.FORWARD if self.v_nom >= 0 else DriveDirection.REVERSE
            )
            
            # CHANGED: Absolute values to maintain proper spatial lookahead
            self.ds = abs(self.v_nom) * self.Ts
            self.ds_ref = abs(self.v_nom) * self.Ts  
            
            if self.v_nom >= 0:
                self.v_bounds = [Constants.rear_motor_top_speed*self.lower_speed_limit, Constants.rear_motor_top_speed*self.upper_speed_limit]
            else:
                self.v_bounds = [-Constants.rear_motor_top_speed*self.upper_speed_limit, -Constants.rear_motor_top_speed*self.lower_speed_limit]

            self._update_solver_bounds()
            
            logger.debug(
                "Set nominal speed: %s%% -> %.3f m/s (%s)",
                speed_percent,
                self.v_nom,
                self._drive_direction.value,
            )
    
    def get_nominal_speed(self):
        """Get the current nominal speed setting."""
        with self._lock:
            speed_percent = (self.v_nom / Constants.rear_motor_top_speed) * 100.0
            return self.v_nom, speed_percent
    
    def set_speed_tracking_weight(self, weight):
        """Set the weight for speed tracking in the MPC cost function."""
        if weight <= 0:
            logger.warning(f"Speed tracking weight must be positive. Got {weight}, ignoring.")
            return
        
        with self._lock:
            self.V_weight = weight
            logger.debug(f"Set speed tracking weight: {weight}")
    
    def get_path(self):
        """Get the current reference path."""
        with self._lock:
            return self.path_matrix.copy() if self.path_matrix is not None else None
    
    def start_path_following(self):
        """Start the MPC path following in a separate thread."""
        with self._lock:
            if self._running:
                logger.debug("Path following already running")
                return

            if self._thread is not None and self._thread.is_alive():
                logger.warning("Path following thread is still shutting down; refusing to start a second control loop")
                return
            
            if self.path_matrix is None:
                logger.error("No path set. Call set_path() first.")
                return
            
            self._running = True
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
            logger.info("MPC path following started")
    
    def stop_path_following(self):
        """Stop the MPC path following."""
        thread_to_join = None
        with self._lock:
            if not self._running:
                thread_to_join = self._thread
            else:
                self._running = False
                logger.info("MPC path following stopped")
                thread_to_join = self._thread
        
        # Wait for thread to fully exit before returning
        if thread_to_join is not None:
            thread_to_join.join(timeout=2.0)
            if thread_to_join.is_alive():
                logger.warning("MPC control loop thread did not terminate within timeout")
            else:
                with self._lock:
                    if self._thread is thread_to_join:
                        self._thread = None
        
        # Reset warm start data to prevent incompatibility on restart
        with self._lock:
            self._x_prev = None
            self._last_u = np.array([0.0, 0.0])
            logger.info("MPC state reset for clean restart")
    
    def get_current_commands(self):
        """Get the current velocity and steering commands."""
        with self._lock:
            return self._v_cmd, self._delta_cmd
    
    def is_running(self):
        """Check if path following is active."""
        with self._lock:
            return self._running
    
    def is_at_goal(self, tolerance : float) -> bool:
        """Check if the robot has reached the end of the path."""
        with self._lock:
            if self.path_matrix is None:
                logger.warning("is_at_goal called but no path set")
                return False
            
            # Get current position and compute distance to goal
            current_state = self.state_estimator.get_state()
            distance_to_goal = self._get_distance_to_goal(current_state.pos)
            is_close_enough = distance_to_goal is not None and distance_to_goal <= tolerance
            
            # Also consider the path complete if our current point is near the end of the path, to prevent stalling at the end if the robot overshoots slightly.
            is_nearest_point_end_of_path = self.closest_point_idx is not None and self.closest_point_idx >= len(self.path_matrix) - 2
            return is_close_enough or is_nearest_point_end_of_path # type: ignore
    
    def get_distance_to_goal(self):
        """Get the current distance to the end of the path."""
        with self._lock:
            return self._get_distance_to_goal(self.state_estimator.get_state().pos)
        
    def generate_path(self, start_pose, goal_pose, step_size=0.1) -> np.ndarray:
        """
        Generates a Dubins path that strictly obeys the robot's physical turning limit.
        
        Args:
            start_pose: [x, y, theta]
            goal_pose: [x, y, theta]
            turning_radius: Minimum turning radius in meters (L / tan(max_steer))
            step_size: Distance between waypoints in meters
            
        Returns:
            np.array of shape (N, 3) containing [x, y, theta] waypoints.
        """
        # Poses must be tuples of (x, y, yaw)
        q0 = tuple(start_pose)
        q1 = tuple(goal_pose)
        
        wheelbase = Constants.wheel_base_width
        max_steer_angle = Constants.steering_angle_limit_rads

        # Kinematic formula for Ackermann turning radius
        turning_radius = wheelbase / math.tan(max_steer_angle)
        
        # Generate the Reeds-Shepp path
        # It automatically checks all geometric combinations to find the shortest
        path = rsplan.path( # type: ignore
            start_pose=q0, 
            end_pose=q1, 
            turn_radius=turning_radius, 
            runway_length=0.0,       # If you want to pull straight in at the end set 0.5 
            step_size=step_size,
        )
        
        # Extract the waypoints into a numpy array
        waypoints = path.waypoints()
            
        return np.array(waypoints)
    
    def _get_distance_to_goal(self, current_state) -> float | None:
        """Helper function to compute distance to goal from a given state."""
        with self._lock:
            if self.path_matrix is None:
                return None
            
            current_x = current_state[0]
            current_y = current_state[1]
            
            goal_x = self.path_matrix[-1, 0]
            goal_y = self.path_matrix[-1, 1]
            
            distance_to_goal = np.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
            return distance_to_goal
    
    def _control_loop(self):
        """Main control loop running in separate thread."""
        logger.info("MPC control loop started")
        next_time = time.time()
        iteration = 0

        try:
            while True:
                with self._lock:
                    if not self._running:
                        break
                
                next_time += self.Ts
                start_time = time.time()
                iteration += 1
                
                try:
                    # Get current state from Kalman filter
                    state = self.state_estimator.get_state()
                    cur_state = np.array([state.pos[0], state.pos[1], 
                                         self.state_estimator.euler[2]])  # x, y, yaw
                    
                    # Generate reference trajectory
                    refs = self._generate_reference(cur_state)
                    distance_to_goal = self._get_distance_to_goal(cur_state)
                    
                    # Generate speed reference trajectory
                    with self._lock:
                        v_nom_current = self.v_nom
                    
                    v_ref = np.zeros(self.p + 1)
                    for i in range(self.p + 1):
                        if distance_to_goal is not None:
                            # Estimate distance to goal for this point in the horizon
                            dist_i = max(0.0, distance_to_goal - (i * self.ds_ref))
                            
                            # CHANGED: Smooth deceleration profile within 1.5 meters of the goal
                            if dist_i < 1.5:
                                speed_scale = max(0.0, dist_i / 1.5)
                                # Ensure complete stop by dropping velocity to 0 at extreme proximity
                                v_ref[i] = v_nom_current * speed_scale if dist_i > 0.05 else 0.0
                            else:
                                v_ref[i] = v_nom_current
                        else:
                            v_ref[i] = v_nom_current
                    
                    # Prepare parameters
                    params = np.concatenate([cur_state, self._last_u, refs.flatten(), v_ref])
                    
                    # Solve MPC
                    solver_args = {
                        'lbx': self.lbx, 
                        'ubx': self.ubx, 
                        'lbg': self.lbg, 
                        'ubg': self.ubg, 
                        'p': params
                    }
                    # Only use warm start after first few iterations to avoid stale solver state issues
                    if self._x_prev is not None and iteration > 2:
                        solver_args['x0'] = ca.DM(self._x_prev)
                    
                    res = self.solver(**solver_args)
                    
                    # Verify solver result is valid before using
                    if res is None or 'x' not in res:
                        logger.error("Solver returned invalid result, zeroing commands")
                        with self._lock:
                            self._v_cmd = 0.0
                            self._delta_cmd = 0.0
                        continue
                    
                    # Extract control commands
                    u_offset = self.n_states * (self.p + 1)
                    u_opt = res['x'][u_offset : u_offset + self.n_controls]
                    v_cmd = float(u_opt[0])
                    delta_cmd = float(u_opt[1])
                    
                    # Sanity check before applying commands
                    if not (np.isfinite(v_cmd) and np.isfinite(delta_cmd)):
                        logger.error("Solver produced non-finite commands: v=%.3f, δ=%.3f, zeroing", v_cmd, delta_cmd)
                        with self._lock:
                            self._v_cmd = 0.0
                            self._delta_cmd = 0.0
                        continue
                    
                    # Update stored values
                    with self._lock:
                        self._v_cmd = v_cmd
                        self._delta_cmd = delta_cmd
                        self._last_u = np.array([v_cmd, delta_cmd])
                        self._x_prev = res['x']
                    
                    if abs(v_cmd) < 0.01 and distance_to_goal is not None and distance_to_goal > 0.25:
                        logger.debug(
                            "MPC commanding zero velocity but not at goal! Distance to goal: %.3f m (tolerance: %.3f m)",
                            distance_to_goal,
                            0.25
                        )
                    
                    elapsed = time.time() - start_time
                    logger.debug(
                        "MPC: V=%.2f m/s | δ=%.1f° | Pos=(%.2f, %.2f) | Time=%.3fs",
                        v_cmd,
                        np.degrees(delta_cmd),
                        cur_state[0],
                        cur_state[1],
                        elapsed,
                    )
                    
                except Exception:
                    logger.exception("MPC error")
                    with self._lock:
                        self._v_cmd = 0.0
                        self._delta_cmd = 0.0
                
                # Timing control
                sleep_duration = next_time - time.time()
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
        finally:
            with self._lock:
                if self._thread is threading.current_thread():
                    self._thread = None
            logger.info("MPC control loop stopped")
