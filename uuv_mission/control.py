# This is a code by Alexander Bergman, Hertford College, University of Oxford.
# The purpose of this code is to build a controller for a UUV (Unmanned Underwater Vehicle) that can navigate to a target position using PD control.
# Code started on 15/10/2026

def PD_Controller(current_position, target_position, current_velocity, Kp, Kd):
    """
    PD Controller for UUV navigation.

    Parameters:
    current_position (tuple): Current position of the UUV (x, y, z).
    target_position (tuple): Target position for the UUV (x, y, z).
    current_velocity (tuple): Current velocity of the UUV (vx, vy, vz).
    Kp (float): Proportional gain.
    Kd (float): Derivative gain.

    Returns:
    tuple: Control output (control_x, control_y, control_z).
    """
    
    # Calculate the error between the target and current position
    error = [target - current for target, current in zip(target_position, current_position)]
    
    # Calculate the derivative of the error (change in error)
    derivative = [-velocity for velocity in current_velocity]
    
    # Calculate control output using PD formula
    control_output = [Kp * e + Kd * d for e, d in zip(error, derivative)]
    
    return tuple(control_output)