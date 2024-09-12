"""
This is the implementation of the OGN node defined in RoverSimpleController.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import math


def map_angular_to_steering(angular_speed) -> float:
    """Map angular speed to steering angle."""
    if abs(angular_speed) < 1e-3:
        return 0.0

    # max 0.6 min -0.6
    angular_speed = min(0.6, max(angular_speed, -0.6))
    return (angular_speed / abs(angular_speed)) * (-25 * abs(angular_speed) + 17.5)


class RoverSimpleController:
    """
        Receives Linear and Angular Velocity for rover and translates to wheel and steering
    outputs
    """

    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""
        # Get the inputs
        chassis_length = db.inputs.chassis_length
        chassis_width = db.inputs.chassis_width
        linear_velocity = db.inputs.linear_velocity
        angular_velocity = db.inputs.angular_velocity

        # Compute wheel velocity
        linear_velocity_x = linear_velocity[0]
        wheel_velocity = [
            linear_velocity_x,
            linear_velocity_x * 1.5,
            linear_velocity_x,
            -linear_velocity_x,
            -linear_velocity_x * 1.5,
            -linear_velocity_x,
        ]

        # Compute steering angle
        steer_position = []
        turn_rad = map_angular_to_steering(angular_velocity[2])

        if abs(turn_rad) < 1e-3:
            steer_position = [0.0, 0.0, 0.0, 0.0]
        else:
            turning_radius = abs(turn_rad)  # R

            chassis_length = 2.08157  # L
            chassis_width = 1.53774  # T

            alpha_i = math.atan(chassis_length / (turning_radius - (chassis_width / 2)))
            alpha_o = math.atan(chassis_length / (turning_radius + (chassis_width / 2)))

            if alpha_i > 0.6:
                alpha_i = 0.6

            if alpha_o > 0.6:
                alpha_o = 0.6

            alpha_i = round(alpha_i, 2)
            alpha_o = round(alpha_o, 2)

            if turn_rad > 0.0:
                steer_position = [alpha_i, -alpha_i, alpha_o, -alpha_o]
            else:
                steer_position = [-alpha_o, alpha_o, -alpha_i, alpha_i]

        # Assign the outputs
        db.outputs.wheel_velocity = wheel_velocity
        db.outputs.steering_position = steer_position

        return True
