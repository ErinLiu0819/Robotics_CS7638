######################################################################
# This file copyright the Georgia Institute of Technology
#
# Author: xliu3019
######################################################################


def pid_thrust(target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
    Student code for Thrust PID control. Drone's starting x, y position is (0, 0).

    Args:
    target_elevation: The target elevation that the drone has to achieve
    drone_elevation: The drone's elevation at the current time step
    tau_p: Proportional gain
    tau_i: Integral gain
    tau_d: Differential gain
    data: Dictionary that you can use to pass values across calls.
        Reserved keys:
            max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.

    Returns:
        Tuple of thrust, data
        thrust - The calculated change in thrust using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.
            Reserved keys:
                max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    '''

    if 'prev_err' not in data:
        data['prev_err'] = target_elevation - drone_elevation
        data['int_err'] = 0

    err = target_elevation - drone_elevation
    diff_err = err - data['prev_err']
    data['int_err'] += err

    thrust = tau_p * err + tau_d * diff_err + tau_i * data['int_err']
    data['prev_err'] = err




    return thrust, data


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data:dict() = {}):
    '''
    Student code for PD control for roll. Drone's starting x,y position is 0, 0.

    Args:
    target_x: The target horizontal displacement that the drone has to achieve
    drone_x: The drone's x position at this time step
    tau_p: Proportional gain, supplied by the test suite
    tau_i: Integral gain, supplied by the test suite
    tau_d: Differential gain, supplied by the test suite
    data: Dictionary that you can use to pass values across calls.

    Returns:
        Tuple of roll, data
        roll - The calculated change in roll using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.

    '''

    if 'prev_err' not in data:
        data['prev_err'] = target_x - drone_x
        data['int_err'] = 0

    err = target_x - drone_x
    data['int_err'] += err
    diff_err = err - data['prev_err']

    roll = tau_p * err + tau_d * diff_err + tau_i * data['int_err']

    data['prev_err'] = err

    return roll, data


def find_parameters_thrust(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test cases only.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune
    params = [0.59, 22, 0]  # [0.54, 21.5, 0.00001]
    dparams = [0.01, 0.5, 0]   # [0.01, 0.01, 0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
    osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
    best_error = hover_error * 0.2 + vel_error * 0.7 + osc_error * 0.2

    # Stopping criteria
    tolerance = 0.001
    acceptable_err = 0.3

    # Tuning until criteria are met
    # Twiddle code are learned from the lecture
    while sum(dparams) > tolerance and best_error > acceptable_err:
        for i in range(len(params)-1):
            params[i] += dparams[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}  # Update params

            # Run simulator with updated params
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

            # Recalculate the error
            vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
            osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
            err = hover_error * 0.2 + vel_error * 0.7 + osc_error * 0.2

            # If the error improves, update best_error and increase dparams
            if err < best_error:
                best_error = err
                dparams[i] *= 1.1
            else:
                # If no improvement, try the opposite direction
                params[i] -= 2.0 * dparams[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}  # Update params

                # Run simulator again after reversing parameter change abd recalculate the error
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
                vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
                osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
                err = hover_error * 0.2 + vel_error * 0.7 + osc_error * 0.2

                # If error improves after reversing, update best_error and increase dparams
                if err < best_error:
                    best_error = err
                    dparams[i] *= 1.1
                else:
                    # If no improvement, revert parameter and reduce dparams
                    params[i] += dparams[i]
                    dparams[i] *= 0.9

    # Return the dict of gain values that give the best error.
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
    return thrust_params, roll_params

def find_parameters_with_int(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test case with Integral error

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune, e.g.,
    params = [0.4,10,0.001]
    dparams = [0.01, 0.1, 0.0001]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
    osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
    best_error = hover_error * 0.2 + vel_error * 0.7 + osc_error * 0.2

    # Stop criteria
    tolerance = 0.001
    acceptable_err = 0.4

    # Tuning until criteria are met
    # Twiddle code are learned from the lecture
    while sum(dparams) > tolerance and best_error > acceptable_err:
        for i in range(2, len(params)):
            params[i] += dparams[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}  # Update params

            # Run simulator with updated params
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            # Recalculate the error
            vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
            osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
            err = hover_error * 0.2 + vel_error * 0.7 + osc_error * 0.2

            # If the error improves, update best_error and increase dparams
            if err < best_error:
                best_error = err
                dparams[i] *= 1.1
            else:
                # If no improvement, try the opposite direction
                params[i] -= 2.0 * dparams[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}  # Update params
                # Run simulator again after reversing parameter change and recalculate the error
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
                vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
                osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
                err = hover_error * 0.2 + vel_error * 0.7 + osc_error * 0.2

                # If error improves after reversing, update best_error and increase dparams
                if err < best_error:
                    best_error = err
                    dparams[i] *= 1.1
                else:
                    # If no improvement, revert parameter and reduce dparams
                    params[i] += dparams[i]
                    dparams[i] *= 0.9

    # Return the dict of gain values that give the best error.
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    return thrust_params, roll_params

def find_parameters_with_roll(run_callback, tune='both', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you will
    find gain values for Thrust as well as Roll PID controllers.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    # Initialize a list to contain your gain values that you want to tune, e.g.,
    params_thrust = [0.5,19,0]
    dparams_thrust = [0.01, 0.05, 0.0001]

    # roll_params = [-20,10000,0.01]
    # roll_dparams = [0.1,1000,0.01]
    params_roll = [-150,-10000,0.001]
    dparams_roll = [5,1000,0.0001]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': params_roll[0], 'tau_d': params_roll[1], 'tau_i': params_roll[2]}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
    osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
    best_error = (hover_error + vel_error + osc_error) / 3

    # Implement your code to use twiddle to tune the params and find the best_error
    tolerance = 0.001
    acceptable_err = 0.4
    while sum(dparams_thrust + dparams_roll) > tolerance and best_error > acceptable_err:
        for i in range(len(params_thrust)):
            params_thrust[i] += dparams_thrust[i]
            thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}   # Update params

            # Run simulator with updated params
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            # Recalculate the =error
            vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
            osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
            err = (hover_error + vel_error + osc_error) / 3

            # If the error improves, update best_error and increase dparams
            if err < best_error:
                best_error = err
                dparams_thrust[i] *= 1.1
            else:
                # If no improvement, try the opposite direction
                params_thrust[i] -= 2.0 * dparams_thrust[i]
                thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}

                # Run simulator again after reversing parameter change
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

                # Recalculate the total error
                vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
                osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
                err = (hover_error + vel_error + osc_error) / 3

                # If error improves after reversing, update best_error and increase dparams
                if err < best_error:
                    best_error = err
                    dparams_thrust[i] *= 1.1
                else:
                    # If no improvement, revert parameter and reduce dparams
                    params_thrust[i] += dparams_thrust[i]
                    dparams_thrust[i] *= 0.9

        for i in range(len(params_roll)):
            params_roll[i] += dparams_roll[i]
            roll_params = {'tau_p': params_roll[0], 'tau_d': params_roll[1], 'tau_i': params_roll[2]} # Update params

            # Run simulator with updated params
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            # Recalculate the error
            vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
            osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
            err = (hover_error + vel_error + osc_error) / 3

            # If the error improves, update best_error and increase dparams
            if err < best_error:
                best_error = err
                dparams_roll[i] *= 1.1
            else:
                # If no improvement, try the opposite direction
                params_roll[i] -= 2.0 * dparams_roll[i]
                roll_params = {'tau_p': params_roll[0], 'tau_d': params_roll[1],'tau_i': params_roll[2]}  # Update params

                # Run simulator again after reversing parameter change
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

                # Recalculate the total error
                vel_error = max(0, (drone_max_velocity - max_allowed_velocity) / drone_max_velocity)
                osc_error = max(0, (total_oscillations - max_allowed_oscillations) / total_oscillations) if total_oscillations > 0 else 0
                err = (hover_error + vel_error + osc_error) / 3

                # If error improves after reversing, update best_error and increase dparams
                if err < best_error:
                    best_error = err
                    dparams_roll[i] *= 1.1
                else:
                    # If no improvement, revert parameter and reduce dparams
                    params_roll[i] += dparams_roll[i]
                    dparams_roll[i] *= 0.9

    # Return the dict of gain values that give the best error.
    thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}
    roll_params = {'tau_p': params_roll[0], 'tau_d': params_roll[1], 'tau_i': params_roll[2]}
    return thrust_params, roll_params
