using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;


public class Trajectory
{
    /*
     Class containing types of ferries divided according to their design functions
        Each function returns a value depending on the initial conditions and the current time - ELAPSED TIME

    s - position (angle / coordinate)
    w - velocity

     */

    public static (float s, float w) Move_Linear_Simple(float Elapsed_Time, float Start_Angle, float Target_Time, float Target_Velocity, float Start_Time =0)
    {
        /*_________________________________________
            Source: class Linear_Function_Cls from 
            https://github.com/rparak/Trajectory_Generation/
            __________________________________________      
        */
        float s = 0;                
        float w = Target_Velocity;

        if (Target_Time > Elapsed_Time)
        {
            s = Start_Angle + Elapsed_Time * Target_Velocity;
        }
        else
        {
            s = Start_Angle + Target_Time * Target_Velocity;
            w = 0;
        }
            return (s, w);
    }

    public static (float s, float w) Move_Linear_Velocity(float Elapsed_Time, float Start_Angle, float Target_Time, float Start_Velocity, float Target_Velocity, float Start_Time = 0)
    {
        float s = 0;
        float w = Start_Velocity;
        float a = (Target_Velocity - Start_Velocity) / (Target_Time - Start_Time);

        if (Target_Time > Elapsed_Time)
        {
            w = Start_Velocity + Elapsed_Time * a;
            s = Start_Angle + Elapsed_Time * w;
        }
        else
        {
            s = Start_Angle + Target_Time * Target_Velocity;
            w = Target_Velocity;
        }

        return (s, w);
    }

    public static (float s, float w) Move_Polynomial(float Elapsed_Time, double[][] T_inv,float Start_Angle, float Target_Angle, float Target_Time = 0,float Start_Velocity = 0, float Target_Velocity = 0 )
{
        /*_________________________________________
         Source: class Polynomial_Profile_Cls from 
           https://github.com/rparak/Trajectory_Generation/
         __________________________________________      
         */
        float s = 0;
        float w = 0;       // TODO: remove
        
        // TODO: No acceleration specifiead
        double[] Q = { (double)Start_Angle, (double)Start_Velocity, 0, (double)Target_Angle, (double)Target_Velocity, 0 };

        double[][] C;
        double[][] Q_temp = new double[6][];

        for (int i = 0; i < Q.Length; i++)
        {
            Q_temp[i] = new double[1];
            Q_temp[i][0] = Q[i];
        }

        // Get C vector 
        C = InteMath.MatrixProduct(T_inv, Q_temp);
        /*
             Analytic expression (position):
             s(t) = c_{0} + c_{1}*t + c_{2}*t^2 + c_{3}*t^3 + c_{4}*t^4 + c_{5}*t^5
        */

        for (int i = 0; i < 6; i++)
        {
            s += Mathf.Pow(Elapsed_Time, i) * (float)C[i][0];
        }

        double[] coef_arr = new double[] { 1.0, 2.0, 3.0, 4.0, 5.0 };
        /*
             Analytic expression (velocity):
             s_dot(t) = c_{1} + 2*c_{2}*t + 3*c_{3}*t^2 + 4*c_{4}*t^3 + 5*c_{5}*t^4
        */

        for (int i = 1; i < 6; i++)
        {
                w += Mathf.Pow(Elapsed_Time, (i - 1)) * (float)C[i][0] * (float)coef_arr[i-1];

        }

        return (s, w);
    }

    // TRAPEZOIDAL  - NOT CURRENTLY USED
    public static (float s, float w) Move_Trapezoidal(float Elapsed_Time, float Start_Angle, float Target_Angle, float Target_Time = 0, float Velocity_Initital = 0, float Velocity_Max = 0)
    // Comment
    {
        float w = 0;          // angular velocity
        float s = Start_Angle;          // angle
        // REMOVE
        float Angular_Velocity_Safe = 30;

        // Set default parameters if not specified 
        if (Velocity_Max == 0 && Target_Time == 0)
        {
            // Get direction
            Velocity_Max = Angular_Velocity_Safe;
            if (Start_Angle > Target_Angle)
            {
                Velocity_Max *= -1;
            }
        }
        // No velocity specified 
        if (Velocity_Max == 0 && Target_Time != 0)
        {
            // Delta_Direct respecting directions of move 
            Velocity_Max = 1.5f * ((Target_Angle - Start_Angle) / Target_Time);
        }
        // No Time specified 
        if (Target_Time == 0)
        {
            Target_Time = Mathf.Abs(1.5f * (Target_Angle - Start_Angle) / Velocity_Max);
        }

        // Time of constant acceleration phase.
        float T_a = ((Start_Angle - Target_Angle) + Velocity_Max * Target_Time) / Velocity_Max;
        // Express the acceleration with a simple formula.
        float a = Velocity_Max / T_a;

        // Phase 1: Acceleration
        if (Elapsed_Time <= Mathf.Abs(T_a))
        {
            s = Start_Angle + 0.5f * a * Mathf.Pow(Elapsed_Time, 2);
            w = a * Elapsed_Time;
        }

        // Phase 2: Constant velocity.
        if (Elapsed_Time > T_a && Elapsed_Time <= Target_Time - T_a)
        {
            s = Start_Angle - 0.5f * Velocity_Max * T_a + Velocity_Max * Elapsed_Time;
            w = Velocity_Max;
        }

        // Phase 3: Deceleration.
        if (Elapsed_Time > Target_Time - T_a)
        {
            s = Target_Angle - 0.5f * a * Mathf.Pow(Target_Time, 2) + a * Target_Time * Elapsed_Time - 0.5f * a * Mathf.Pow(Elapsed_Time, 2);
            w = a * Target_Time - a * Elapsed_Time;
        }
        if (Elapsed_Time > Target_Time)
        {
            w = 0;
            s = Target_Angle;
        }

        return (s, w);
    }

}