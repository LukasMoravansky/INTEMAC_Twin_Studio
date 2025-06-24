
using System.Collections.Generic;
using UnityEngine;


public class SegmentTrajectory : ProgramUtilities
{
    /*
    ___________________________________________________________________________________________________
    Class that returns a List of vertices of the whole trajectory built from commands (MoveJ, MoveL...)
    Each MOVE is divided into N vertices that the robot endpoint passes through

    Class is used to plot the robot's endpoint trajectory

    - To calculate the movement, you also need to enter the time. 
    This class does not track the dynamics of the movement, only the trajectory, therefore a constant time is used everywhere - 10 sec
    ____________________________________________________________________________________________________
    */

    private float[] Init_Angles = new float[6]; // stored ANGLES position, where last movement ended and where should next movement start
    private Matrix4x4 Init_TCP;                 // stored TCP position, where last movement ended and where should next movement start

    float Subdivisions;
    private int Segment_Count;                  // Number of segments into which the movement will be divided

    // Kinematic properties
    public Matrix4x4 T_Base;                    
    public Matrix4x4 T_Tool;

    // Internal 
    private int Command_Idx;                    // Keeps track of which command in script is executed

    List<Vector3> Path_Vertices = new List<Vector3>();  // Stores whole trajectory 

    public void Clear_Memory()
    {
        Path_Vertices.Clear();
    }

    // Main call to get segmented path
    public List<Vector3> Get_Movement_Path(float[] Start_Angles, int Target_Segment_Vertices)
    {
        // Check for Error
        if (Target_Segment_Vertices <= 0)
        {
            return null;
        }

        Segment_Count = Target_Segment_Vertices;
        Subdivisions = 1f / (float)Target_Segment_Vertices;

        // Get init Agles
        Init_Angles = Start_Angles;
        // Get init TCP
        Init_Angles = Start_Angles;
        Matrix4x4 Temp_Matrix = RobotKinematics.CalculateForwardKinematics(Init_Angles);
        Init_TCP = T_Base * Temp_Matrix * T_Tool;
        // Conversion is needed
        Init_TCP = KinematicsUtilities.Convert_Coordinates(Init_TCP, false);

        Command_Idx = 0;
        // Unpack all commands
        foreach (var command in Path_Commands)
        {
            List < List < Vector3>> Temp_Vertices = new List<List<Vector3>>();
            Temp_Vertices.Add(command.Invoke());
            List<Vector3> lastList = Temp_Vertices[Temp_Vertices.Count - 1];
            Path_Vertices.AddRange(lastList);
            Command_Idx += 1; 
        }

        return Path_Vertices;
    }

    // Return endpoints for drawing path
    public List<Vector3> MoveJ(float[] Target)
    {
        float Time_Temp = 10;
        List<Vector3> Vertices = new List<Vector3>();

        // Initial handling
        Update_Initial_Coords(Target);

        // Movement class init
        MovementUtilities[] Movement = new MovementUtilities[J];
        for (int i = 0; i < J; i++)
        {
            Movement[i] = new MovementUtilities();
            Movement[i].Init();
            Movement[i].Target_S = Target[i];               // Angle
            Movement[i].Start_S = Init_Angles[i];           // Angle
            Movement[i].Target_T = Time_Temp;
        }

        float[] Angles = new float[J];
        Matrix4x4 Forward_End_Matrix = new Matrix4x4();  // Transformation matrix calculated by forward kinemtics

        // Simualtion of each joint movement in time step
        for (float t = 0; t <= Time_Temp; t += Time_Temp * Subdivisions)
        {
            for (int j = 0; j < J; j++)
            {
                Movement[j].Get_Simple_Motion(t);
                Angles[j] = Movement[j].s;
            } 

            // Get endpoint postion 
            Forward_End_Matrix = RobotKinematics.CalculateForwardKinematics(Angles);
            // Adjust endpoit according to Tool and Base
            Forward_End_Matrix = T_Base * Forward_End_Matrix * T_Tool;


            // Convert to Vector3 in unity coordinates
            Vector3 Translation = new Vector3(Forward_End_Matrix.m03, Forward_End_Matrix.m23, -Forward_End_Matrix.m13);
            Vertices.Add(Translation);
        }
        // Save Angles as new starting point
        Init_Angles = Angles;
        // Save TCP as new starting point
        Init_TCP = KinematicsUtilities.Convert_Coordinates(Forward_End_Matrix, false);

        return Vertices;
    }

    public List<Vector3> MoveL(float[] Target)
    {
        float Time_Temp = 10;
        List<Vector3> Vertices = new List<Vector3>();
        Matrix4x4 Forward_End_Matrix = new Matrix4x4();
        // Movement class init
        MovementUtilities Movement = new MovementUtilities();

        float[] Target_Q = new float[J];
        MovementUtilities.PredictedlinearMove Path_Linear = new MovementUtilities.PredictedlinearMove();

        // transform x,y,z,rx,ry,rz to Matrix
        Vector3 Temp_Vector = new Vector3(Target[0], Target[1], Target[2]);
        Quaternion Temp_Quaternion = Quaternion.Euler(Target[3], Target[4], Target[5]);
        Matrix4x4 Temp_Matrix = KinematicsUtilities.Quaternion_To_Matrix(Temp_Quaternion, Temp_Vector);

        // Get Q for all vertices
        Path_Linear = Movement.Predict_Trajectory(Init_TCP, Init_Angles, Temp_Matrix, Time_Temp, T_Tool, T_Base, Segment_Count);

        for (int s = 0; s < Path_Linear.Trajectory_Q.GetLength(0); s++)
        {
            
            for (int j = 0; j < J; j++)
            {
                Target_Q[j] = Path_Linear.Trajectory_Q[s, j];
            }

            // Save start of trajectory
            if (s == 0)
            {
                Update_Initial_Coords(Target_Q);
            }

            // Get endpoint postion 
            Forward_End_Matrix = RobotKinematics.CalculateForwardKinematics(Target_Q);
            // Adjust endpoit according to Tool and Base
            Forward_End_Matrix = T_Base * Forward_End_Matrix * T_Tool;

            // Convert to Vector3 in unity coordinates
            Vector3 Translation = new Vector3(Forward_End_Matrix.m03, Forward_End_Matrix.m23, -Forward_End_Matrix.m13);
            Vertices.Add(Translation);
        }
        // Save Angles as new starting point
        Init_Angles = Target_Q;

        // Save TCP as new starting point
        Init_TCP = KinematicsUtilities.Convert_Coordinates(Forward_End_Matrix, false);

        return Vertices;
    }
    private void Update_Initial_Coords(float[] Angles)
    {
        if (Command_Idx == 0)
        {
            Start_Of_Trajectory = Angles;
        }
    }
}