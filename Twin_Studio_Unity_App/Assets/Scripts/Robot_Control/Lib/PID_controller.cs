using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PID_controller
{
    public float Kp = 0.5f; // Proportional coefficient
    public float Ki = 0.01f; // Integral coefficient
    public float Kd = 0.2f; // Derivative coefficient
    public float Max_Output = 100.0f; // Maximum output (torque)
    public float Min_Output = -100.0f; // Minimum output (torque)

    private float Previous_Error = 0.0f; // Previous error
    private float Integral = 0.0f; // Integral sum

    public float Target_Angle = 0.0f; // Target angle

    // NOTE: 
    // previousError may cause errors if called in IF STATEMENT

    public float UpdatePID(float Target_Value, float Process_Value, float Delta_Time)
    {
        // Calculate error
        float Error = Target_Value - Process_Value;

        // Compute integral sum
        Integral += Error * Delta_Time;

        // Compute derivative
        float Derivative = (Error - Previous_Error) / Delta_Time;

        // Compute controller output
        float Output = Kp * Error + Ki * Integral + Kd * Derivative;

        // Limit output to maximum and minimum values
        if (Output > Max_Output)
        {
            Output = Max_Output;
        }
        else if (Output < Min_Output)
        {
            Output = Min_Output;
        }

        // Update previous error
        Previous_Error = Error;

        return Output; // Torque to be applied
    }
}