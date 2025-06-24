using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using Unity.VisualScripting;
using UnityEngine;

public class ProgramUtilities
{
    /*______________________________________________________
     Description:
      Class determined do preprocess Robot program commands
      Commands are stored in queues which allows to store whole seqeunce of commands/cycle
     _______________________________________________________
    */

    public int J;                               // Number of Joints
    // MoveJ properties
    protected Queue<float>[] Target_Angles;     // Stores target Angles of joints   UNUSED
    protected Queue<float> Target_Times;        // Stores target times of movements. UNUSED

    protected Queue<Matrix4x4> Target_TCP;
    public Matrix4x4 Last_TCP;

    public float[] Initial_Angles = new float[6];

    //  Path prediction properties 
    public float[] Start_Of_Trajectory = new float[6];      // Starting point of whole program
    private SegmentTrajectory Path;                         // Used for segmenting trajectory created by commands
    protected List<Func<List<Vector3>>> Path_Commands = new List<Func<List<Vector3>>>();        // Stores commands in current executed program

    // References to main Robot script 
    private Queue<Action> Commands = new Queue<Action>();   // FIFO used for calling commands in Robot class
    private Robot Parent_Script;                            // Parent Robot class

    public void Init(Robot Parent)
    {
        Parent_Script = Parent;

        // Arrays init
        Target_TCP = new Queue<Matrix4x4>();
        Target_Angles = new Queue<float>[6];
        for (int i = 0; i < 6; i++)
        {
            Target_Angles[i] = new Queue<float>();
        }

        Target_Times = new Queue<float>();
        J = 6;
    }

    // Preprocess parameters for MoveJ command in Joint_Control class
    public void MoveJ(float[] Target_Point, float Target_Time)
    {
        // Check for 6 coordinates
        if (Target_Point.Length != J)
        {
            throw new ArgumentException("Expected " + J + " joints." );
        }

        if (Target_Time < 0)
        {
            throw new ArgumentException("Expected non-negative value.");
        }

        // Add Values to FIFO
        Commands.Enqueue(() => Parent_Script.MoveJ(Target_Point, Target_Time));

        // Add Values to Path prediction
        Path_Commands.Add(() => Path.MoveJ(Target_Point));
    }

    public void MoveL(float[] Target_Point, float Target_Time)
    {
        if (Target_Time < 0)
        {
            throw new ArgumentException("Expected non-negative value.");
        }

        // Add Values to FIFO
        Commands.Enqueue(() => Parent_Script.MoveL(Target_Point, Target_Time));

        // Add Values to Path prediction
        Path_Commands.Add(() => Path.MoveL(Target_Point));
    }

    // Check if queue is empty
    public bool Is_Empty()
    {
        if (Commands.Count == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // Clear data (Queues, lists)
    public void Clear_Commands()
    {
        Commands.Clear();
        Path_Commands.Clear();

        Target_TCP.Clear();
        Target_Times.Clear();
        for (int i = 0; i < 6; i++)
        {
            Target_Angles[i].Clear();
        }
    }

    // If it is called, the function (MoveJ, MoveL..) in the Robot Class is called. The called function is obtained from the FIFO
    public void getCommand()
    {
        Commands.Dequeue().Invoke();
        if (Commands.Count == 0 && Path != null)
        {
            Path.Clear_Memory();
        }
    }
    
    // Calls SegementTrajectory class, returns vertices (segments) of trajectory stored in command list
    public List<Vector3> Get_Path_Verices(float[] Start_Angles, int Subdivisions = 10)
    {
        // Path predict init
        Path = new SegmentTrajectory();
        // Pass Kinematic parameters of parent script
        Path.T_Base = Parent_Script.Robot_Parent.T_Robot_Base;
        Path.T_Tool = Matrix4x4.identity;
        Path.T_Tool.m23 = Parent_Script.Tool;
        Path.Init(null);

        Path.Path_Commands = Path_Commands;

        // Get segmented path
        List<Vector3> Vertices = new List<Vector3>(); 
        Vertices = Path.Get_Movement_Path(Start_Angles, Subdivisions);
        Start_Of_Trajectory = Path.Start_Of_Trajectory;

        return Vertices;
    }
}
