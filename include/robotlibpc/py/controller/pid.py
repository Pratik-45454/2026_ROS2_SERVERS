"""
@file pid.py
@brief PID library modified from Arduino PID Library - Version 1.2.1
@date 2023

This PID package is created form original CPP PID library using ChatGpt 3
- Modified by Robotics club, Pulchowk Campus
"""

from enum import Enum
import time


class Mode(Enum):
    MANUAL = 0    # Manual Tuning Mode
    AUTOMATIC = 1 # Automatic Tuning Mode


class ControlDirection(Enum):
    DIRECT = 0    # Direct Control Direction
    REVERSE = 1   # Reverse Control Direction


class ProportionalType(Enum):
    PROPORTIONAL_ON_ERROR = 0       # Proportional on Error
    PROPORTIONAL_ON_MEASUREMENT = 1 # Proportional on Measurement


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max, sample_time,
                 p_type=ProportionalType.PROPORTIONAL_ON_ERROR,
                 control_direction=ControlDirection.DIRECT,
                 mode=Mode.AUTOMATIC):
        """ @brief PID class constructor.
            @param kp Proportional gain.
            @param ki Integral gain.
            @param kd Derivative gain.
            @param out_min Minimum output limit.
            @param out_max Maximum output limit.
            @param sample_time Time between PID calculations.
            @param p_type Type of proportional output (Error/Measurement).
            @param control_direction Control direction (Direct/Reverse).
            @param mode Tuning mode (Auto/Manual).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.sample_time = sample_time
        self.p_type = p_type
        self.control_direction = control_direction
        self.mode = mode

        self.input = 0.0
        self.setpoint = 0.0
        self.output_sum = 0.0
        self.output = 0.0

        self.last_input = 0.0
        self.last_time = time.time()

        self.init()

    def init(self):
        """ @brief Initializes the PID controller. """
        sample_time_sec = self.sample_time / 1000.0
        self.kp_scaled = self.kp
        self.ki_scaled = self.ki * sample_time_sec
        self.kd_scaled = self.kd / sample_time_sec

        if self.control_direction == ControlDirection.REVERSE:
            self.kp_scaled = -self.kp_scaled
            self.ki_scaled = -self.ki_scaled
            self.kd_scaled = -self.kd_scaled

    def compute(self):
        """ @brief Performs the PID computation.
            @return True if successful PID computation.
            @details It should be called every time loop() cycles. ON/OFF and
                     calculation frequency can be set using SetMode and SetSampleTime respectively.
        """
        if self.mode != Mode.AUTOMATIC:
            return False

        now = time.time()
        time_change = (now - self.last_time) * 1000.0

        if time_change < self.sample_time:
            return False

        error = self.setpoint - self.input
        d_input = self.input - self.last_input

        self.output_sum += (self.ki_scaled * error)

        if self.p_type == ProportionalType.PROPORTIONAL_ON_MEASUREMENT:
            self.output_sum -= self.kp_scaled * d_input

        self.output_sum = self.clamp(self.output_sum, self.out_min, self.out_max)

        new_output = self.kp_scaled * error if self.p_type == ProportionalType.PROPORTIONAL_ON_ERROR else 0.0
        new_output += self.output_sum - self.kd_scaled * d_input

        self.output = self.clamp(new_output, self.out_min, self.out_max)

        self.last_input = self.input
        self.last_time = now

        return True

    def set_tunings(self, kp, ki, kd):
        """ @brief Sets new tuning parameters.
            @param kp New proportional gain.
            @param ki New integral gain.
            @param kd New derivative gain.
            @details This function allows the controller's dynamic performance to be adjusted.
                     it's called automatically from the constructor, but tunings can also
                     be adjusted on the fly during normal operation.
        """
        if kp < 0 or ki < 0 or kd < 0:
            return

        self.kp = kp
        self.ki = ki
        self.kd = kd

        sample_time_sec = self.sample_time / 1000.0
        self.kp_scaled = self.kp
        self.ki_scaled = self.ki * sample_time_sec
        self.kd_scaled = self.kd / sample_time_sec

    def set_output_limits(self, out_min, out_max):
        """ @brief Sets new output limits.
            @param out_min New minimum output limit.
            @param out_max New maximum output limit.
        """
        if out_min >= out_max:
            return

        self.out_min = out_min
        self.out_max = out_max

        if self.mode == Mode.AUTOMATIC:
            self.output = self.clamp(self.output, self.out_min, self.out_max)
            self.output_sum = self.clamp(self.output_sum, self.out_min, self.out_max)

    def set_mode(self, mode):
        """ @brief Sets the tuning mode (Auto/Manual).
            @param mode New tuning mode.
            @details Allows the controller Mode to be set to manual or Automatic.
                     when the transition from manual to auto occurs, the controller is
                     automatically initialized.
        """
        if self.mode == Mode.MANUAL and mode == Mode.AUTOMATIC:
            self.output_sum = self.output
            self.last_input = self.input
            self.output_sum = self.clamp(self.output_sum, self.out_min, self.out_max)

        self.mode = mode

    def set_sample_time(self, sample_time):
        """ @brief Sets the sample time for PID calculation.
            @param sample_time New sample time in Milliseconds.
        """
        if sample_time > 0:
            ratio = sample_time / self.sample_time
            self.ki_scaled *= ratio
            self.kd_scaled /= ratio
            self.sample_time = sample_time

    def get_out_max(self):
        return self.out_max

    def get_out_min(self):
        return self.out_min

    @staticmethod
    def clamp(value, value_min, value_max):
        """ Clamp value between value_min and value_max. """
        return max(value_min, min(value, value_max))
