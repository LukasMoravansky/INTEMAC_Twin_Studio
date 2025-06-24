using System;
using UnityEngine;


public class MovementProperties
{
    /*
     The class is used to STORE parameters for movement - static and target positions (angles and TCP) and times
    */


    // Time properties 
    public float Elapsed_Time;
    public float Target_Time;
    public float Start_Time;
    public float Delta_Time;

    // TCP properties   [ Transform matrices]
    public Matrix4x4 Start_TCP { get; private set; }
    public Matrix4x4 Target_TCP { get; private set; }
    public Matrix4x4 T_Tool;
    public Matrix4x4 T_Base;

    // Angles properties [ Degrees ]
    public float[] Start_Q { get; private set; }
    public float[] Target_Q { get; private set; }
    public float[] Current_Q;


    // State variables
    public bool Linear_Busy;
    public bool Busy;
    public bool Done;
    public bool Stopped;    // Blocked
    // todo: bool Ready?

    // Number of Joints
    private int J = 6;

    // Movement Calculator
    public MovementUtilities Movement_Calculator;

    public MovementUtilities.PredictedlinearMove Path_Linear;

    public MovementProperties()
    {
        Start_Q = new float[6];
        Target_Q = new float[6];
        Current_Q = new float[6];
        Movement_Calculator = new MovementUtilities();
        Movement_Calculator.Init();
        T_Tool = Matrix4x4.identity;
        T_Base = Matrix4x4.identity;
    }


    // When using this function, all parameters for the target position are changed
    public void Set_Target_TCP(Matrix4x4 TCP)
    {
        // Set TCP to variable 
        Target_TCP = TCP;
        // Calculate all solutions of IK for taget TCP and get the closest one
        float[,] All_Solutions = new float[8, 6];
        All_Solutions = RobotKinematics.Calculate_Inverse_Kinematics_FULL(TCP, T_Tool, T_Base);
        int Index = Movement_Calculator.Get_Closest_Solution(Current_Q, All_Solutions);
        // Set Angles
        for (int i = 0; i < J; i++)
        {
            Target_Q[i] = All_Solutions[Index, i];
        }

    }

    // When using this function, all parameters for the target position are changed
    public void Set_Target_Angles(float[] Angles)
    {
        if (Angles.Length != J)
        {
            throw new ArgumentException("Invalid number of joints");
        }

        Target_Q = Angles;
        Matrix4x4 TCP = RobotKinematics.CalculateForwardKinematics(Angles);
        Target_TCP = TCP;
    }

    // When using this function, all parameters for the starting position are changed
    public void Set_Start_TCP(Matrix4x4 TCP)
    {
        float[,] All_Solutions = new float[8, 6];
        All_Solutions = RobotKinematics.Calculate_Inverse_Kinematics_FULL(TCP, T_Tool, T_Base);
        int Index = Movement_Calculator.Get_Closest_Solution(Current_Q, All_Solutions);

        // Set Angles
        for (int i = 0; i < J; i++)
        {
            Start_Q[i] = All_Solutions[Index, i];
        }

        // Set TCP
        Start_TCP = TCP;
    }

    // When using this function, all parameters for the starting position are changed
    public void Set_Start_Angles(float[] Angles)
    {
        if (Angles.Length != J)
        {
            throw new ArgumentException("Invalid number of joints");
        }

        Start_Q = Angles;
        Matrix4x4 TCP = RobotKinematics.CalculateForwardKinematics(Angles);
        TCP = T_Base * TCP * T_Tool;
        TCP = KinematicsUtilities.Convert_Coordinates(TCP, false);

        // Set TCP
        Start_TCP = TCP;
    }

    // Sets offset in Z-axis as tool
    public void Set_Tool(float Z_Axis)
    {
        T_Tool = Matrix4x4.identity;
        T_Tool.m23 =  Z_Axis;
    }

    // Removes Tool
    public void Remove_Tool()
    {
        T_Tool = Matrix4x4.identity;
    }

    // The function calculates values for the PredictedLinearMove structure based on parameters stored in the MovementProperties class
    public void Get_Linear_Path_Prediction(int Subdivision = 10)
    {
        // Subdivision parameter is the number of segments into which the resulting linear motion will be divided
        if (Subdivision < 1)
        { 
            throw new ArgumentException("Number of subdivisions must be grater than 1");
        }
        Delta_Time = (Target_Time * 1 / Subdivision);
        Path_Linear = new MovementUtilities.PredictedlinearMove();
        Path_Linear.Trajectory_Q = new float[Subdivision, 6];
        Path_Linear.Trajectory_W = new float[Subdivision, 6];
        Path_Linear = Movement_Calculator.Predict_Trajectory(Start_TCP, Current_Q, Target_TCP, Target_Time, T_Tool, T_Base, Subdivision);
    }
}
