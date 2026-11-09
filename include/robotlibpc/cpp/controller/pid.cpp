/**********************************************************************************************
 * @file pid.cpp
 * @brief PID library source modified from Arduino PID Library - Version 1.2.1
 * @author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * @date 2023
 *
 * @section license License
 * This PID Library is licensed under the MIT License
 *
 * - Feb 6, 2024    : Modified by Robotics club, Pulchowk Campus
 * - Apr 12, 2024   : Modified for Rashberry PI
 *********************************************************************************************/

#include <stdio.h>
#include <cmath>

#include "pid.hpp"
#include "../maths/math.hpp"
#include "../common/time.hpp"

/** @brief PID class constructor.
 *  @param kp Proportional gain.
 *  @param ki Integral gain.
 *  @param kd Derivative gain.
 *  @param outMin Minimum output limit.
 *  @param outMax Maximum output limit.
 *  @param sampleTime Time between PID calculations.
 *  @param tol Tolerance for allowable error (deadband).
 *  @param controlDirection Control direction (Direct/Reverse).
 *  @param pType Type of proportional output (Error/Measurement).
 *  @param mode Tuning mode (Auto/Manual).
 *
 *  The parameters specified here are those for for which we can't set up
 *  reliable defaults, so we need to have the user set them.
 */
PID::PID(float kp, float ki, float kd,
         float outMin, float outMax, int16_t sampleTime, float tol,
         ControlDirection controlDirection,
         ProportionalType pType,
         Mode mode)
    : kp_(kp), ki_(ki), kd_(kd),
      outMin_(outMin), outMax_(outMax), sampleTime_(sampleTime), tol_(tol),
      controlDirection_(controlDirection), pType_(pType), mode_(mode)
{
}

/**
 * @brief Initializes the PID controller.
 */
void PID::init()
{
    /*Calculate sample time in second*/
    float sampleTimeSec = (float)sampleTime_ / 1000;

    /*Initialize scaler of each term of PID using gains and sample time in second*/
    kpScaled_ = kp_;
    kiScaled_ = ki_ * sampleTimeSec;
    kdScaled_ = kd_ / sampleTimeSec;

    /*Change sign of scaled gains if control direction is reverse*/
    if (controlDirection_ == REVERSE)
    {
        kpScaled_ = (-kpScaled_);
        kiScaled_ = (-kiScaled_);
        kdScaled_ = (-kdScaled_);
    }

    lastTime_ = get_tick_ms();
}

/** @brief Performs the PID computation.
 *  @return True if successful PID computation.
 *  @details It should be called every time loop() cycles. ON/OFF and
 *  calculation frequency can be set using SetMode and SetSampleTime respectively.
 */
bool PID::compute()
{
    if (mode_ != AUTOMATIC)
        return false;

    /*Compute time change*/
    uint32_t now = get_tick_ms();
    uint16_t timeChange = (now - lastTime_);

    /*Re-scale gains if tunings were changed since last compute*/
    if (kd_ != prev_kd_ || ki_ != prev_ki_ || kp_ != prev_kp_)
    {
        setTunings(kp_, ki_, kd_);
        outputSum = 0.0f;
    }

    if (timeChange < sampleTime_)
        return false;

    /*Compute all the working error variables*/
    float error = setpoint - input;
    float dInput = input - lastInput_;
    float newOutputSum = outputSum;

    /*Check if error is within tolerance (deadband), with a 50ms settling window)*/
    is_allowable_error = std::fabs(error) < tol_;

    if (is_allowable_error)
    {
        if (allow_error_count < 5)
        {
            allow_error_count++;
            is_allowable_error = false;
        }
    }
    else
    {
        allow_error_count = 0;
    }

    newOutputSum += (kiScaled_ * error);

    /*Add Proportional on Measurement, if Proportion on Measurement is specified*/
    if (pType_ == PROPORTIONAL_ON_MEASUREMENT)
        newOutputSum -= kpScaled_ * dInput;

    newOutputSum = clamp<float>(newOutputSum, outMin_, outMax_);

    /*We should create new output variable. Directly assigning to output create misleading
      curve of output if it is observed on run time from graph like stm32 cube monitor.*/
    float newOutput;

    /*Add Proportional on Error, if Proportional on Error is specified*/
    if (pType_ == PROPORTIONAL_ON_ERROR)
        newOutput = kpScaled_ * error;
    else
        newOutput = 0.0f;

    /*Compute Rest of PID Output*/
    newOutput += newOutputSum - kdScaled_ * dInput;

    newOutput = clamp<float>(newOutput, outMin_, outMax_);

    /*Update output and outputSum*/
    output = newOutput;
    outputSum = newOutputSum;

    /*Remember some variables for next time*/
    lastInput_ = input;
    lastTime_ = now;

    return true;
}

/** @brief Sets new tuning parameters.
 *  @param kp New proportional gain.
 *  @param ki New integral gain.
 *  @param kd New derivative gain.
 *
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 */
void PID::setTunings(float kp, float ki, float kd)
{
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    float sampleTimeSec = (float)sampleTime_ / 1000.0f;
    kpScaled_ = kp_;
    kiScaled_ = ki_ * sampleTimeSec;
    kdScaled_ = kd_ / sampleTimeSec;

    /*Track previous gains to detect runtime changes in compute()*/
    prev_kp_ = kp_;
    prev_ki_ = ki_;
    prev_kd_ = kd_;
}

/** @brief Sets new output limits.
 *  @param newOutMin New minimum output limit.
 *  @param newOutMax New maximum output limit.
 */
void PID::setOutputLimits(float newOutMin, float newOutMax)
{
    if (newOutMin >= newOutMax)
        return;

    outMin_ = newOutMin;
    outMax_ = newOutMax;

    if (mode_ == AUTOMATIC)
    {
        output = clamp<float>(output, outMin_, outMax_);
        outputSum = clamp<float>(outputSum, outMin_, outMax_);
    }
}

/** @brief Sets the tuning mode (Auto/Manual).
 *  @param newMode New tuning mode.
 *
 * Allows the controller Mode to be set to manual or Automatic
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 */
void PID::setMode(Mode newMode)
{
    /*Remove bump when transitioning from manual to automatic*/
    if ((mode_ == MANUAL) && (newMode == AUTOMATIC))
    {
        outputSum = output;
        lastInput_ = input;

        outputSum = clamp<float>(outputSum, outMin_, outMax_);
    }

    mode_ = newMode;
}

/** @brief Sets the sample time for PID calculation.
 *  @param newSampleTime New sample time in Milliseconds
 */
void PID::setSampleTime(uint16_t newSampleTime)
{
    if (newSampleTime > 0)
    {
        float ratio = (float)newSampleTime / (float)sampleTime_;
        kiScaled_ *= ratio;
        kdScaled_ /= ratio;
        sampleTime_ = newSampleTime;
    }
}

float PID::getOutMax()
{
    return outMax_;
}

float PID::getOutMin()
{
    return outMin_;
}

/** @brief Resets the PID internal state */
void PID::reset()
{
    outputSum = 0.0f;   
    lastInput_ = 0.0f; 
    input = 0.0f;
    output = 0.0f;
    setpoint = 0.0f;   
    is_allowable_error = false;
}