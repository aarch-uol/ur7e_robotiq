"""
Mathematical functions for F/T sensor calibration.

Theory:
-------
When a payload with mass m and center of gravity (CoG) is attached to an F/T sensor,
the measured wrench in the sensor frame consists of:

1. Force: F = R_world_to_sensor @ [0, 0, -m*g]^T
   Where R_world_to_sensor is the rotation matrix from world frame to sensor frame
   
2. Torque: T = CoG × F
   The torque is the cross product of the CoG position vector and the gravity force

By measuring the wrench in multiple orientations, we can solve for m and CoG using
least-squares optimization.

The minimum number of poses is theoretically 2 (with different orientations), but
3-4 poses with significant orientation differences provide more accurate results.
"""

import numpy as np
from scipy.optimize import least_squares
from typing import Any, Dict, Tuple, List, Optional


# Gravity constant (m/s^2)
GRAVITY = 9.80665


def rotation_matrix_from_quaternion(quat: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [x, y, z, w] to 3x3 rotation matrix.
    
    Args:
        quat: Quaternion as [x, y, z, w]
        
    Returns:
        3x3 rotation matrix
    """
    x, y, z, w = quat
    
    # Normalize quaternion
    n = np.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-10:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    
    return R


def compute_expected_wrench(
    R_sensor_to_world: np.ndarray,
    mass: float,
    cog: np.ndarray,
    gravity: float = GRAVITY
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute expected force and torque given sensor orientation, mass and CoG.
    
    The sensor frame has gravity vector transformed from world frame.
    Assuming world frame has Z-up convention, gravity is [0, 0, -g] in world frame.
    
    ATI F/T sensor convention: positive force = force pushing INTO the sensor
    (compression positive). When payload hangs below sensor, it pulls down,
    which is measured as positive Fz when sensor Z points down.
    
    Args:
        R_sensor_to_world: 3x3 rotation matrix from sensor frame to world frame
                          (this is what TF provides)
        mass: Payload mass in kg
        cog: Center of gravity [x, y, z] in sensor frame, in meters
        gravity: Gravity acceleration (default 9.80665 m/s^2)
        
    Returns:
        force: Expected force [Fx, Fy, Fz] in sensor frame (N)
        torque: Expected torque [Tx, Ty, Tz] in sensor frame (Nm)
    """
    # Gravity vector in world frame (Z-up, pointing down)
    g_world = np.array([0, 0, -gravity])
    
    # Transform gravity to sensor frame using R^T (world->sensor = inverse of sensor->world)
    R_world_to_sensor = R_sensor_to_world.T
    g_sensor = R_world_to_sensor @ g_world
    
    # F/T sensor convention: measures force that payload exerts on sensor
    # When payload hangs down, it pulls sensor down = positive force in sensor Z direction
    # Since g_sensor points in same direction as payload weight when sensor Z points down,
    # the measured force is: F = m * g_sensor
    force = mass * g_sensor
    
    # Torque = CoG × Force
    torque = np.cross(cog, force)
    
    return force, torque


def residual_function(
    params: np.ndarray,
    rotations: List[np.ndarray],
    measured_wrenches: List[np.ndarray],
    weights: Optional[np.ndarray] = None,
    estimate_bias: bool = False
) -> np.ndarray:
    """
    Compute residuals for least-squares optimization.
    
    Args:
        params: [mass, cog_x, cog_y, cog_z] or [mass, cog_x, cog_y, cog_z, bias_fx, bias_fy, bias_fz, bias_tx, bias_ty, bias_tz]
        rotations: List of 3x3 rotation matrices (world to sensor)
        measured_wrenches: List of measured wrenches [Fx, Fy, Fz, Tx, Ty, Tz]
        weights: Optional weights for each measurement component
        estimate_bias: If True, params includes 6 bias terms
        
    Returns:
        Residuals as 1D array
    """
    mass = params[0]
    cog = params[1:4]
    
    if estimate_bias:
        bias = params[4:10]
    else:
        bias = np.zeros(6)
    
    residuals = []
    
    for R, wrench_meas in zip(rotations, measured_wrenches):
        force_exp, torque_exp = compute_expected_wrench(R, mass, cog)
        wrench_exp = np.concatenate([force_exp, torque_exp]) + bias
        
        diff = wrench_meas - wrench_exp
        if weights is not None:
            diff = diff * weights
        residuals.extend(diff)
    
    return np.array(residuals)


def estimate_mass_and_cog(
    orientations: List[np.ndarray],
    measured_wrenches: List[np.ndarray],
    initial_mass: float = 1.0,
    initial_cog: Optional[np.ndarray] = None,
    use_forces_only: bool = False,
    estimate_bias: bool = True,
    verbose: bool = True
) -> Tuple[float, np.ndarray, dict]:
    """
    Estimate payload mass and center of gravity from F/T measurements.
    
    Args:
        orientations: List of quaternions [x, y, z, w] representing sensor orientation
                     in world frame for each measurement pose
        measured_wrenches: List of measured wrenches [Fx, Fy, Fz, Tx, Ty, Tz]
                          Forces in N, Torques in Nm
        initial_mass: Initial guess for mass (kg)
        initial_cog: Initial guess for CoG [x, y, z] in meters (default [0, 0, 0.05])
        use_forces_only: If True, only use force measurements (simpler but less accurate)
        estimate_bias: If True, also estimate sensor bias (recommended for real sensors)
        verbose: Print optimization progress
        
    Returns:
        mass: Estimated mass in kg
        cog: Estimated center of gravity [x, y, z] in sensor frame (meters)
        info: Dictionary with additional information (residuals, covariance, bias, etc.)
    """
    if initial_cog is None:
        initial_cog = np.array([0.0, 0.0, 0.05])
    
    # Convert quaternions to rotation matrices
    rotations = [rotation_matrix_from_quaternion(q) for q in orientations]
    
    # Set up weights (can emphasize forces vs torques)
    if use_forces_only:
        weights = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    else:
        # Weight forces and torques differently since they have different units
        # Typical force ~10N, typical torque ~1Nm, so scale torques up
        weights = np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0])
    
    # Initial parameters
    if estimate_bias:
        # [mass, cog_x, cog_y, cog_z, bias_fx, bias_fy, bias_fz, bias_tx, bias_ty, bias_tz]
        x0 = np.array([initial_mass, initial_cog[0], initial_cog[1], initial_cog[2],
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Bounds
        bounds = (
            [0.01, -0.2, -0.2, -0.2, -5.0, -5.0, -5.0, -1.0, -1.0, -1.0],  # Lower
            [10.0, 0.2, 0.2, 0.5, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0]            # Upper
        )
    else:
        x0 = np.array([initial_mass, initial_cog[0], initial_cog[1], initial_cog[2]])
        bounds = (
            [0.01, -0.2, -0.2, -0.2],
            [10.0, 0.2, 0.2, 0.5]
        )
    
    # Run optimization
    result = least_squares(
        residual_function,
        x0,
        args=(rotations, measured_wrenches, weights, estimate_bias),
        bounds=bounds,
        verbose=2 if verbose else 0,
        method='trf'  # Trust Region Reflective algorithm
    )
    
    mass = result.x[0]
    cog = result.x[1:4]
    bias = result.x[4:10] if estimate_bias else np.zeros(6)
    
    # Compute statistics
    residuals = result.fun
    n_measurements = len(measured_wrenches)
    n_params = 10 if estimate_bias else 4
    dof = n_measurements * 6 - n_params  # degrees of freedom
    
    # RMS error
    rms_error = np.sqrt(np.mean(residuals**2))
    
    # Compute expected vs measured for each pose
    pose_errors = []
    for i, (R, wrench_meas) in enumerate(zip(rotations, measured_wrenches)):
        force_exp, torque_exp = compute_expected_wrench(R, mass, cog)
        wrench_exp = np.concatenate([force_exp, torque_exp]) + bias
        error = wrench_meas - wrench_exp
        pose_errors.append({
            'pose_idx': i,
            'measured': wrench_meas,
            'expected': wrench_exp,
            'error': error,
            'force_error_norm': np.linalg.norm(error[:3]),
            'torque_error_norm': np.linalg.norm(error[3:])
        })
    
    info = {
        'success': result.success,
        'message': result.message,
        'rms_error': rms_error,
        'residuals': residuals,
        'pose_errors': pose_errors,
        'n_iterations': result.nfev,
        'optimality': result.optimality,
        'bias': bias,
        'bias_estimated': estimate_bias
    }
    
    return mass, cog, info


def compute_calibration_observability(
    orientations: List[np.ndarray],
) -> Dict[str, Any]:
    """Measure whether the selected orientations identify all ten parameters.

    The linearized payload model uses mass, first moment (mass times CoG), and
    six constant bias components. Column normalization makes the reported
    condition number describe pose geometry rather than mixed physical units.
    """
    design_blocks = []
    gravity_directions = []
    for quaternion in orientations:
        quaternion = np.asarray(quaternion, dtype=float)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("each orientation must contain four finite quaternion values")
        rotation = rotation_matrix_from_quaternion(quaternion)
        gravity_sensor = rotation.T @ np.array([0.0, 0.0, -GRAVITY])
        gx, gy, gz = gravity_sensor
        skew_gravity = np.array([
            [0.0, -gz, gy],
            [gz, 0.0, -gx],
            [-gy, gx, 0.0],
        ])

        block = np.zeros((6, 10))
        block[:3, 0] = gravity_sensor
        block[:3, 4:7] = np.eye(3)
        block[3:, 1:4] = -skew_gravity
        block[3:, 7:10] = np.eye(3)
        block[3:, :] *= 10.0
        design_blocks.append(block)
        gravity_directions.append(gravity_sensor / np.linalg.norm(gravity_sensor))

    design = np.vstack(design_blocks)
    column_norms = np.linalg.norm(design, axis=0)
    safe_column_norms = np.where(
        column_norms > np.finfo(float).eps, column_norms, 1.0
    )
    normalized_design = design / safe_column_norms
    singular_values = np.linalg.svd(normalized_design, compute_uv=False)
    rank = int(np.linalg.matrix_rank(normalized_design))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > np.finfo(float).eps
        else float('inf')
    )

    max_angle = 0.0
    for i, first in enumerate(gravity_directions):
        for second in gravity_directions[i + 1:]:
            cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
            max_angle = max(max_angle, float(np.degrees(np.arccos(cosine))))

    return {
        'rank': rank,
        'required_rank': 10,
        'condition_number': condition,
        'max_gravity_angle_deg': max_angle,
        'singular_values': singular_values,
    }


def analyze_payload_measurements(
    orientations: List[np.ndarray],
    measured_wrenches: List[np.ndarray],
    baseline_bias: Optional[np.ndarray] = None,
    estimate_residual_bias: bool = True,
    verbose: bool = False,
) -> Tuple[float, np.ndarray, dict]:
    """Estimate payload parameters with consistent baseline/residual bias handling.

    ``baseline_bias`` is a separately measured unloaded-sensor bias.  It is
    subtracted before fitting.  A residual constant bias is still estimated by
    default so that drift between the unloaded and payload phases is not folded
    into the payload mass or CoG.

    The returned ``info`` dictionary contains ``baseline_bias``,
    ``residual_bias`` and ``total_bias``.  The total is the constant offset that
    must already be removed by hardware tare or by a software bias stage before
    the wrench reaches an admittance controller.
    """
    if len(orientations) != len(measured_wrenches):
        raise ValueError("orientations and measured_wrenches must have the same length")
    if len(orientations) < 3:
        raise ValueError("at least three distinct calibration poses are required")

    for wrench in measured_wrenches:
        wrench = np.asarray(wrench, dtype=float)
        if wrench.shape != (6,) or not np.all(np.isfinite(wrench)):
            raise ValueError("each measured wrench must contain six finite values")

    observability = compute_calibration_observability(orientations)

    baseline = (
        np.zeros(6, dtype=float)
        if baseline_bias is None
        else np.asarray(baseline_bias, dtype=float)
    )
    if baseline.shape != (6,) or not np.all(np.isfinite(baseline)):
        raise ValueError("baseline_bias must contain six finite values")

    corrected_wrenches = [
        np.asarray(wrench, dtype=float) - baseline for wrench in measured_wrenches
    ]
    mass, cog, info = estimate_mass_and_cog(
        orientations=orientations,
        measured_wrenches=corrected_wrenches,
        estimate_bias=estimate_residual_bias,
        verbose=verbose,
    )

    residual = np.asarray(info.get("bias", np.zeros(6)), dtype=float)
    info["baseline_bias"] = baseline
    info["residual_bias"] = residual
    info["total_bias"] = baseline + residual
    info["baseline_bias_applied"] = baseline_bias is not None
    info["observability"] = observability
    return mass, cog, info


def evaluate_calibration_quality(
    mass: float,
    cog: np.ndarray,
    info: Dict[str, Any],
    measurement_stds: Optional[List[np.ndarray]] = None,
    *,
    min_mass: float = 0.02,
    max_mass: float = 5.0,
    max_cog_distance: float = 0.30,
    max_force_error: float = 0.75,
    max_torque_error: float = 0.05,
    max_force_std: float = 0.15,
    max_torque_std: float = 0.01,
    max_total_force_bias: float = 0.75,
    max_total_torque_bias: float = 0.05,
    max_observability_condition: float = 100.0,
) -> Dict[str, Any]:
    """Apply conservative quality gates before publishing controller values.

    Optimizer convergence alone does not establish that a calibration is
    physically usable.  These gates check fit errors, stationary sensor noise,
    parameter plausibility and the constant bias that would remain at runtime.
    """
    issues: List[str] = []
    warnings: List[str] = []
    cog = np.asarray(cog, dtype=float)
    pose_errors = info.get("pose_errors", [])
    total_bias = np.asarray(info.get("total_bias", np.zeros(6)), dtype=float)

    force_errors = [float(p["force_error_norm"]) for p in pose_errors]
    torque_errors = [float(p["torque_error_norm"]) for p in pose_errors]
    max_observed_force_error = max(force_errors, default=float("inf"))
    max_observed_torque_error = max(torque_errors, default=float("inf"))
    cog_distance = float(np.linalg.norm(cog))
    total_force_bias = float(np.linalg.norm(total_bias[:3]))
    total_torque_bias = float(np.linalg.norm(total_bias[3:]))
    observability = info.get("observability")

    if not info.get("success", False):
        issues.append("least-squares optimizer did not converge")
    if observability is None:
        warnings.append("pose observability metrics were unavailable")
    else:
        if observability['rank'] < observability['required_rank']:
            issues.append(
                f"pose design rank {observability['rank']} is below the required "
                f"{observability['required_rank']}; use more varied sensor orientations"
            )
        if observability['condition_number'] > max_observability_condition:
            issues.append(
                f"pose design condition number {observability['condition_number']:.1f} exceeds "
                f"{max_observability_condition:.1f}; use more varied sensor orientations"
            )
    if not np.isfinite(mass) or not np.all(np.isfinite(cog)):
        issues.append("estimated mass or CoG is not finite")
    elif not min_mass <= mass <= max_mass:
        issues.append(
            f"estimated mass {mass:.4f} kg is outside [{min_mass:.3f}, {max_mass:.3f}] kg"
        )
    if np.isfinite(cog_distance) and cog_distance > max_cog_distance:
        issues.append(
            f"CoG distance {cog_distance:.4f} m exceeds {max_cog_distance:.4f} m"
        )
    if max_observed_force_error > max_force_error:
        issues.append(
            f"maximum force fit error {max_observed_force_error:.3f} N exceeds "
            f"{max_force_error:.3f} N"
        )
    if max_observed_torque_error > max_torque_error:
        issues.append(
            f"maximum torque fit error {max_observed_torque_error:.4f} Nm exceeds "
            f"{max_torque_error:.4f} Nm"
        )
    if total_force_bias > max_total_force_bias:
        issues.append(
            f"constant force bias {total_force_bias:.3f} N exceeds "
            f"{max_total_force_bias:.3f} N; re-tare unloaded or add runtime bias correction"
        )
    if total_torque_bias > max_total_torque_bias:
        issues.append(
            f"constant torque bias {total_torque_bias:.4f} Nm exceeds "
            f"{max_total_torque_bias:.4f} Nm; re-tare unloaded or add runtime bias correction"
        )

    max_observed_force_std = 0.0
    max_observed_torque_std = 0.0
    if measurement_stds is not None and len(measurement_stds) > 0:
        stds = [np.asarray(std, dtype=float) for std in measurement_stds]
        if any(std.shape != (6,) or not np.all(np.isfinite(std)) for std in stds):
            issues.append(
                "one or more measurement standard deviations are malformed or non-finite"
            )
        else:
            max_observed_force_std = max(float(np.max(std[:3])) for std in stds)
            max_observed_torque_std = max(float(np.max(std[3:])) for std in stds)
            if max_observed_force_std > max_force_std:
                issues.append(
                    f"stationary force noise {max_observed_force_std:.3f} N exceeds "
                    f"{max_force_std:.3f} N"
                )
            if max_observed_torque_std > max_torque_std:
                issues.append(
                    f"stationary torque noise {max_observed_torque_std:.4f} Nm exceeds "
                    f"{max_torque_std:.4f} Nm"
                )
    else:
        warnings.append("measurement standard deviations were unavailable")

    return {
        "passed": not issues,
        "issues": issues,
        "warnings": warnings,
        "metrics": {
            "mass_kg": float(mass),
            "cog_distance_m": cog_distance,
            "max_force_fit_error_N": max_observed_force_error,
            "max_torque_fit_error_Nm": max_observed_torque_error,
            "max_force_std_N": max_observed_force_std,
            "max_torque_std_Nm": max_observed_torque_std,
            "total_force_bias_N": total_force_bias,
            "total_torque_bias_Nm": total_torque_bias,
            "observability_rank": (
                observability['rank'] if observability is not None else None
            ),
            "observability_condition": (
                observability['condition_number'] if observability is not None else None
            ),
            "max_gravity_angle_deg": (
                observability['max_gravity_angle_deg']
                if observability is not None else None
            ),
        },
    }


def compute_bias_from_measurement(
    orientation: np.ndarray,
    measured_wrench: np.ndarray,
    mass: float,
    cog: np.ndarray
) -> np.ndarray:
    """
    Compute sensor bias given a measurement and known mass/CoG.
    
    Bias = Measured - Expected
    
    Args:
        orientation: Quaternion [x, y, z, w]
        measured_wrench: Measured wrench [Fx, Fy, Fz, Tx, Ty, Tz]
        mass: Known mass (kg)
        cog: Known CoG [x, y, z] (meters)
        
    Returns:
        bias: Sensor bias [Fx, Fy, Fz, Tx, Ty, Tz]
    """
    R = rotation_matrix_from_quaternion(orientation)
    force_exp, torque_exp = compute_expected_wrench(R, mass, cog)
    wrench_exp = np.concatenate([force_exp, torque_exp])
    
    bias = measured_wrench - wrench_exp
    return bias
