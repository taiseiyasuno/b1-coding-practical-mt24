def PD_Controller(current_position, target_position, Kp, Kd, previous_error):
    """
    This is a Proportional-derivative controller designed to allow a UUV to navigate a cave.
    It should model the following equation accurately:

            u[t] = Kp * e[t] + Kd * (e[t] - e[t-1]), where e[t] = r[t] - y[t]
            For this controller to work, we assume that e[-1] = 0

    Inputs:
    current_position: The current position of the submarine (y)
    target_position: The position where the submarine should be in that timestep (y)
    Kp: The coefficient of the proportional term within the governing control equation
    Kd: The coefficient of the derivative term within the governing control equation
    previous_error: the error from the iteration completed before the current one (should start at 0)

    Outputs:
    control_signal: The signal that will be output which should control the submarine's movement to reduce error between current position and target position.
    error: The error from this timestep, which can be used in the next iteration 

    Written by Alexander Bergman, Hertford College, Oxford University
    """
    # Firstly we need to calculate the error between the current and target position:
    error = [target - current for target, current in zip(target_position, current_position)]

    # Now we calculate the derivative term, which is e[t] - e[t-1].
    derivative_error = [
        current_error - previous
        for current_error, previous in zip(error,previous_error)
        ]
    Output = (Kp * error + Kd * derivative_error)

    return Output, error

# Now this code can be implemented
# Note that the current error must become the previous error for the next timestep
# Therefore implement something like previous_error = error at end of loop.