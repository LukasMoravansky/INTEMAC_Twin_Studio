using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovementUtilities 
{
    /*
    The class processes input parameters (conditions) for motion calculation including dynamics (acceleration, deceleration)
    Parameters are stored in variables (Target, start - velocity, position, time...)
    THE OUTPUT is the position S and the speed W
     
    TRAJECTORY class is used to calculate the trajectory
    */

    public float Target_T;   // Movement duration
    public float Start_S;   // Initial position
    public float Target_S;   // Final position
    public float Target_Velocity;
    public float Start_Velocity;

    // OUTPUT variables:
    public float w { get; private set; }    // Calculated angular velocity
    public float s { get; private set; }    // Calculated angle

    public enum MoveType { Trapezoidal, Polynomial, Linear, Linear_Velocity }
    public MoveType Move_Type;

    // Polynomial trajectory
    private double[][] T_inv;

    public struct PredictedlinearMove
    {
        /* 
            Special struct used for computing linear move
            Linear motion is composed of N MoveJ motions. 
            Inverse kinematics (Inverse_Control class) is used to calculate the linear motion. 
            At each point on the path there can be up to 8 solutions for rotating the robot's joints (6 DOF), and the following solution depends on the solution at the previous point
        */
       public float[,] Trajectory_Q;
       public float[,] Trajectory_W;
       public int Segement_index;
    } 

    // Multisegment parameters - in development
    public struct MultiSegmentMove
    {
        public float Start_Angle;
        public float[] Target_Angles;
        public float[] Target_Times;
        public float[] Blend_Times;
        public float[] Target_Velocities;
        public int Multi_Segment_State;             // State = Segment; State -1: Start motion, State 0: constant target velocity + blend on the end of segment 

        public bool Verify_Input()
        {
            // TODO
            bool Corect = true;

            return Corect;
        }
    }

    public void Init()
    {
        // Select trajectory type
        Move_Type = MoveType.Polynomial;
    }

    // Old - Get_Motion is caled directly -REMOVE
    public void Get_Simple_Motion(float Elapsed_Time)
    {
        Start_Velocity = 0;
        Target_Velocity = 0;
        Get_Motion(Elapsed_Time);
    }

    public void Stop(float Angle)
    {
        // Simulates an immediate stop of the joint during, for example, EMERGENCY STOP and interrupts the ongoing movement.
        // The call is made primarily from the Joint_Control class
        w = 0;
        s = Angle;
    }

    public void Get_Motion(float Elapsed_Time)
    {
        /*
         Main function calling the calculation of output values depending on time (Elapsed_Time) and selected trajectory type 
         and specified motion parameters (start, target parameters)
         */

        // Get movement parameters according to input time
        switch (Move_Type)
        {
            case MoveType.Trapezoidal:

                (s, w) = Trajectory.Move_Trapezoidal(Elapsed_Time, Start_S, Target_S, Target_T);

                break;

            case MoveType.Polynomial:
                // Calculate polynomial matrix 
                // TODO: Fix for multi segment
                if (Elapsed_Time == 0)
                {
                    T_inv = Quintic_Polynomial(0f, Target_T);
                }

                (s, w) = Trajectory.Move_Polynomial(Elapsed_Time, T_inv, Start_S, Target_S, Target_T, Start_Velocity: Start_Velocity, Target_Velocity: Target_Velocity);

                break;

            case MoveType.Linear:

                (s, w) = Trajectory.Move_Linear_Simple(Elapsed_Time, Start_S, Target_T, Target_Velocity);

                break;

            case MoveType.Linear_Velocity:

                (s, w) = Trajectory.Move_Linear_Velocity(Elapsed_Time, Start_S, Target_T, Start_Velocity, Target_Velocity);

                break;

        }
    }

    private double[][] Quintic_Polynomial(float t_0, float t_f)
    {
        // t_0, tf .. initial and final time constrain

    /*     _______________________________________________________________________________________
            Descrtiption:
            Obtain the modified polynomial matrix of degree 5.

            A polynomial of degree 5 (quintic) is defined as follows:
                s(t) = c_{0} + c_{ 1}*t + c_{ 2}*t ^ 2 + c_{ 3}*t ^ 3 + c_{ 4}*t ^ 4 + c_{ 5}*t ^ 5



            The quintic polynomial can be expressed by a system of 6 equations. These equations 
            can be converted into a matrix, which we have called T.

            source: https://github.com/rparak/Trajectory_Generation
            __________________________________________________________________________________________
        */

    double[][] T = new double[][]
         {
            new double[] { 1.0,  t_0, Mathf.Pow(t_0, 2), Mathf.Pow(t_0, 3), Mathf.Pow(t_0, 4), Mathf.Pow(t_0, 5) },
            new double[] { 0.0,  1.0, 2.0 * t_0, 3.0 * Mathf.Pow(t_0, 2), 4.0 * Mathf.Pow(t_0, 3), 5.0 * Mathf.Pow(t_0, 4) },
            new double[] { 0.0,  0.0, 2.0, 6.0 * t_0, 12.0 * Mathf.Pow(t_0, 2), 20.0 * Mathf.Pow(t_0, 3) },
            new double[] { 1.0,  t_f, Mathf.Pow(t_f, 2), Mathf.Pow(t_f, 3), Mathf.Pow(t_f, 4), Mathf.Pow(t_f, 5) },
            new double[] { 0.0,  1.0, 2.0 * t_f, 3.0 * Mathf.Pow(t_f, 2), 4.0 * Mathf.Pow(t_f, 3), 5.0 * Mathf.Pow(t_f, 4) },
            new double[] { 0.0,  0.0, 2.0, 6.0 * t_f, 12.0 * Mathf.Pow(t_f, 2), 20.0 * Mathf.Pow(t_f, 3) }
         };

        // Get matrix inversion
        double[][] T_INV = InteMath.MatrixInverse(T);

        return T_INV;
    }

    // IN DEVELPOMENT
    public void Get_Multi_Segment_Motion(float Elapsed_Time, MultiSegmentMove Params)
    {
        /*_________________________________________
         Source: class Multi_Segment_Cls from 
            https://github.com/rparak/Trajectory_Generation/

            Using polynomial trajectory ONLY
         __________________________________________      
        */

        // Check if parametes are valid
        if (Params.Verify_Input() == false)
        {
            throw new ArgumentException("Invalid input parameters");
        }

        int S_c = Params.Target_Times.Length;   // Total number of segments, Segment count
        int S_a = Params.Multi_Segment_State;   // Index of current segment in move
        float Segment_Time;                     // Time in actual segment, segment start -> Segment_Time = 0

        // Phase 1. Start velocity = 0
        if (S_a == -1)
        {
            // Positions
            Start_S = Params.Start_Angle;
            Target_S = Params.Start_Angle + Params.Target_Velocities[0] * Params.Blend_Times[0];

            // Velocities
            Start_Velocity = 0f;
            Target_Velocity = Params.Target_Velocities[0];

            // Time
            Target_T = Params.Blend_Times[0];

            // Calculate
            Get_Motion(Elapsed_Time);
        }

        // Phase #2
        if (Params.Multi_Segment_State >= 0 && S_a < S_c)
        {
            Segment_Time = Elapsed_Time - Sum_Up_To_Index(Params.Target_Times, S_a);

            // Linear Phase 
            if (Segment_Time < (Params.Target_Times[S_a] - Params.Blend_Times[S_a]))
            {
                Move_Type = MoveType.Linear;
                Start_S = Params.Start_Angle + Params.Target_Velocities[0] * Params.Blend_Times[0];
                Target_T = Params.Target_Times[S_a] - Params.Blend_Times[S_a + 1];
                Get_Motion(Segment_Time);
            }

            // Blend Phase

        }

    }

    // Internal calculation for the sum of array elements
    private float Sum_Up_To_Index(float[] Array, int Index)
    {
        float Sum = 0;
        if (Index < 0 || Index >= Array.Length)
        {
            return Sum;
        }

        for (int j = 0; j <= Index; j++)
        {
            Sum += Array[j];
        }
        return Sum;
    }

    public PredictedlinearMove Predict_Trajectory_CUT(Matrix4x4 Start_TCP, float[] Current_Q, Matrix4x4 Target_TCP, float Target_Time, int Subdivision = 100)
    {
        /*
         The function uses a simplified algorithm for calculating IK, where it does not calculate all solutions but only the solution closest to the input combination of angles
         */

        // TEMP
        //float[] Angles = IK.CalculateInverseKinematics_CLOSEST(Target_TCP, Current_Q);

        float[] Angles = Current_Q;
        for (int i = 0; i < 6; i++)
        {
            if (Angles[i] == 0)
            {
                Angles[i] = float.Epsilon;
            }
        }

        float[] Velocity = new float[6];

        PredictedlinearMove Output = new PredictedlinearMove();

        Target_T = Target_Time;

        // Solution matrices
        Output.Trajectory_Q = new float[Subdivision, 6];
        Output.Trajectory_W = new float[Subdivision, 6];
        int Sub = 0;

        // Simulate time sampling
        for (float t = 0; t <= Target_Time; t += Target_Time * 1 / Subdivision)
        {
            Matrix4x4 Current_TCP = new Matrix4x4();
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    Start_S = Start_TCP[i, j];
                    Target_S = Target_TCP[i, j];
                    Get_Motion(t);
                    Current_TCP[i, j] = this.s;
                }
            }

            // Get angles for "t" sample
            // TODO INITIAL VELOCITY
            if (t != 0)
            {
                Current_TCP = KinematicsUtilities.Convert_Coordinates(Current_TCP, true);
                float[] new_Angles = RobotKinematics.CalculateInverseKinematics_CLOSEST(Current_TCP, Angles);
                // Get angular velocity
                for (int l = 0; l < 6; l++)
                {
                    Velocity[l] = (new_Angles[l] - Angles[l]) / (Target_Time * 1 / Subdivision);
                    if (Velocity[l] > 30)
                    {
                        Debug.Log("Velocity Too High");
                    }
                }
                Angles = new_Angles;
            }

            // Write solution to matrix
            for (int joint = 0; joint < 6; joint++)
            {
                Output.Trajectory_Q[Sub, joint] = Angles[joint];
                Output.Trajectory_W[Sub, joint] = Velocity[joint];
            }

            if (Sub != Subdivision - 1)
            {
                Sub += 1;
            }
        }
        return Output; 
    }
    public PredictedlinearMove Predict_Trajectory(Matrix4x4 Start_TCP, float[] Current_Q, Matrix4x4 Target_TCP, float Target_Time, Matrix4x4 T_Tool = default, Matrix4x4 T_Base = default, int Subdivision = 100)
    {
        /*
         Function uses algorithmus to compute all available solutions for the called trajectory segments.
         It then selects one solution in each trajectory segment from a matrix of all solutions depending on the angle combination in the previous segment 
         */

        // TEMP
        //float[] Angles = IK.CalculateInverseKinematics_CLOSEST(Target_TCP, Current_Q);

        float[] Angles = Current_Q;
        for (int i = 0; i < 6; i++)
        {
            if (Angles[i] == 0)
            {
                Angles[i] = float.Epsilon;
            }
        }

        float[] Velocity = new float[6];

        PredictedlinearMove output = new PredictedlinearMove();

        Target_T = Target_Time;

        // Solution matrices
        output.Trajectory_Q = new float[Subdivision, 6];
        output.Trajectory_W = new float[Subdivision, 6];
        int Sub = 0;

        // Simulate time sampling
        for (float t = 0; t <= Target_Time; t += Target_Time * 1 / Subdivision)
        {
            Matrix4x4 Current_TCP = new Matrix4x4();
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    Start_S = Start_TCP[i, j];
                    Target_S = Target_TCP[i, j];
                    Get_Motion(t);
                    Current_TCP[i, j] = this.s;
                }
            }

            // Get angles for "t" sample
            // TODO INITIAL VELOCITY
            if (t != 0)
            {
                Current_TCP = KinematicsUtilities.Convert_Coordinates(Current_TCP, true);
                float[,] All_Solutions_IK = RobotKinematics.Calculate_Inverse_Kinematics_FULL(Current_TCP, T_Tool, T_Base);
                int idx = Get_Closest_Solution(Angles, All_Solutions_IK);
                float[] New_Angles = new float[6];

                for (int k = 0; k < 6; k++)
                {
                    New_Angles[k] = All_Solutions_IK[idx, k];
                }

                // Get angular velocity
                for (int l = 0; l < 6; l++)
                {
                    Velocity[l] = (Mathf.DeltaAngle(Angles[l], New_Angles[l]) / (Target_Time * 1 / Subdivision));
                    if (Velocity[l] > 300)
                    {
                        Debug.Log("Too High");
                    }
                }

                Angles = New_Angles;
            }

            // Write solution to matrix
            for (int Joint = 0; Joint < 6; Joint++)
            {
                output.Trajectory_Q[Sub, Joint] = Angles[Joint];
                output.Trajectory_W[Sub, Joint] = Velocity[Joint];
            }

            if (Sub != Subdivision - 1)
            {
                Sub += 1;
            }
        }
        return output;
    }

    // Find solution of inverse kinematics closest to given combination of joint angles [ degrees ]
    public int Get_Closest_Solution(float[] Angles, float[,] All_Solutions)
    {
        if (Angles.Length != 6)
        {
            throw new ArgumentException("Invalid number of joints");
        }

        // All_Solutions is matrix 8x6
        int R = 8;  //Row
        int C = 6;  // Column

        int Closest_Index = 0;
        float Lowest_Diff = float.MaxValue;

        for (int r = 0; r < R; r++)
        {
            float Solution_Distance = 0;

            for (int c = 0; c < C; c++)
            {
                float Diff = Mathf.Abs(Mathf.DeltaAngle(Angles[c], All_Solutions[r, c]));
                //float Diff = Mathf.Abs(Angles[c] - All_Solutions[r, c]);
                Solution_Distance += Diff * Diff;
            }

            if (Solution_Distance < Lowest_Diff)
            {
                Closest_Index = r;
                Lowest_Diff = Solution_Distance;
            }
        }
        return Closest_Index;
    }
}
