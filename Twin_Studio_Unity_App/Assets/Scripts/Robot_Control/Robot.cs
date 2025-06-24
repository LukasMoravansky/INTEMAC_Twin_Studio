using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Unity.VisualScripting;
using System;
using UnityEngine.UIElements;

public class Robot : MonoBehaviour
{
    public GameObject Ghost_Robot;
    private ForwardControl Controller;
    private InverseControl Controller_Inverse;

    private int J = 6;              // Number of joints

    // OBJECT PROPERTIES OF ROBOT
    private Transform[] All_Joints;
    [HideInInspector]
    public GameObject[] Joints;
    private float[] Joint_Angles;

    private string[] Joint_Names = {"Joint_1",
                            "Joint_2",
                            "Joint_3",
                            "Joint_4",
                            "Joint_5",
                            "Joint_6"};

    private JointControl[] Move_Joint;

    // States
    private enum OperationState { Manual, Automatic, Stop, Linear_Movement }
    private OperationState Operation_State;

    // Program utilities
    private ProgramUtilities P;
    public float Velocity_Percentage;

    // Movement utilities
    private MovementProperties MP;

    // Kinematic Parameters
    [HideInInspector] public KinematicPrams Robot_Parent;
    [HideInInspector] public float Tool;


    // Draw utilities
    public DrawTrajectory Draw_Line_Renderer;

    // Start is called before the first frame update
    void Start()
    {
        //Init
        All_Joints = GetComponentsInChildren<Transform>();
        Joints = new GameObject[J];
        Move_Joint = new JointControl[J];
        // Set if Manual or Automatic
        Operation_State = OperationState.Linear_Movement;
        P = new ProgramUtilities();
        P.Init(this);

        Joint_Angles = new float[6];

        // Get children Joints, must be named as in "JointNames"
        int i = 0;
        foreach (Transform child in All_Joints)
        {
            if (Joint_Names.Contains(child.name) && i < J)
            {
                Joints[i] = child.gameObject;
                i += 1;
            }
        }
        Controller = Ghost_Robot.GetComponent<ForwardControl>();
        Controller_Inverse = Ghost_Robot.GetComponent<InverseControl>();

        // Init of Joint class
        for (int ii = 0; ii < J; ii++)
        {
            Move_Joint[ii] = gameObject.AddComponent<JointControl>();
            Move_Joint[ii].Init(Joints[ii]);
        }

        // Trajectory Properties
        MP = new MovementProperties();
        Robot_Parent = transform.parent.gameObject.GetComponent<KinematicPrams>();
        Controller.Set_Tool = true;

    }

    // Update is called once per frame
    void Update()
    {
        // Get actual joint angles
        for (int i = 0; i < J; i++ )
        {
            MP.Current_Q[i] = Move_Joint[i].Joint_Actual_Rotation;
        }

        // Base Check
        MP.T_Base = Robot_Parent.T_Robot_Base;

        // Tool check
        if (Controller.Set_Tool)
        {
            Set_Tool();
        }

        // Set operation mode
        Control_Select();

        // Set speed percentage
        Velocity_Percentage = 100f;

        // Draw path
        if (Controller.Show_Prediction)
        {
            Erase_Path();
            Draw_Path(P.Start_Of_Trajectory);
        }
        else
        {
            Erase_Path();
        }

        // States
        switch (Operation_State)
        {
            case OperationState.Linear_Movement:
                {
                    if (Controller.Execute_Linear)
                    {
                        bool Executable_Move = false;   // Indicator of vlaid input parameters

                        Executable_Move = MoveLinear();
                        /* MoveL operates with MovementProperties object
                            - Gets Start and Target position
                            - Generates prediction of linear path
                            - Sends movement parameters to each joint (fills up queues with path segments)
                        */

                        // Reset button command
                        Controller.Execute_Linear = false;

                        if (Executable_Move)
                        {
                            // Start executing linear movement
                            for (int j = 0; j < J; j++)
                            {
                                Move_Joint[j].Linear_Path.Execute = true;
                            }
                        }
                    }
                    break;
                }

            case OperationState.Manual:
                {
                    // Executed whe "Execute" variable is set to TRUE on Forward_Control script
                    // Robot target position is set from Ghost robot angles

                    if (Controller.Execute)
                    {
                        for (int i = 0; i < J; i++)
                        {
                            Joint_Angles[i] = Move_Joint[i].Joint_Actual_Rotation;
                        }

                        // Clear previous script commands
                        P.Clear_Commands();

                        Controller.Execute = false;
                        // Add target parameters to MoveJ preprocess;
                        P.MoveJ(Controller.Joints_Target, Controller.Target_Time);
                        P.getCommand();

                        // Draw path
                        //Draw_Path(Joint_Angles);
 
                    }
                    break;
                }

            case OperationState.Automatic:
                {
                    // Breaks case
                    if (Is_Robot_Moving())
                    {
                        break;
                    }

                    // check if Queue commands isnt empty (WHILE true)
                    if (P.Is_Empty())
                    {
                        P.Clear_Commands();
                        Script_Test_Rectangle_2(P);     
                    }

                    // Execute move
                    P.getCommand();
                    break;
                }


            case OperationState.Stop:
                {
                    // Clear previous script commands
                    P.Clear_Commands();

                    // Stop all Joints
                    for (int i = 0; i < J; i++)
                    {
                        Move_Joint[i].Stop();
                    }
                    break;
                }
        }
    }

    private void FixedUpdate()
    {
        // Controll of Linear Movement
        if (MP.Linear_Busy)
        {
            bool Segment_Done = true;
            bool Path_Done = true;

            // Check if all the joints have done the segment
            for (int j = 0; j < J; j++)
            {
                if (!Move_Joint[j].Linear_Path.In_Position && Move_Joint[j].Linear_Path.idx > 0)
                {
                    Segment_Done = false;
                    break;
                }
            }

            // Check if linear parh is done
            for (int j = 0; j < J; j++)
            {
                if (!Move_Joint[j].Linear_Path.Done)
                {
                    Path_Done = false;
                    break;
                }
            }

            // Synchronization of joints
            if (Segment_Done && !Path_Done)
            {
                for (int j = 0; j < J; j++)
                {
                    Move_Joint[j].Linear_Path.Takt = true;
                }
            }

            // Reset of command - movement done
            if (Path_Done)
            {
                MP.Linear_Busy = false;              
            }
            MP.Done = Path_Done;
        }
    }

    // Select case of states
    private void Control_Select()
    {
        //if (Controller.Stop_Movement)
        //{
        //    Operation_State = OperationState.Stop;
        //    Controller.Execute_Script = false;
        //    return;
        //}
        if (Controller.Execute_Script)
        {
            Operation_State = OperationState.Automatic;
        }
        else if (Controller.Execute)
        {
            Operation_State = OperationState.Manual;
        }
        else if (Controller.Execute_Linear)
        {
            Operation_State = OperationState.Linear_Movement;
        }
    }

    // Draws path when Show_Prediction in Control sript is enabled, hide when disabled
    private void Draw_Path(float[] Start_position)
    {
        if (Controller.Show_Prediction)
        {
            if (Draw_Line_Renderer == null)
            {
                Draw_Line_Renderer = gameObject.AddComponent<DrawTrajectory>();
            }
            else if (Draw_Line_Renderer != null)
            {
                Draw_Line_Renderer.Erase_Line();
            }

            List<Vector3> vertices = P.Get_Path_Verices(Start_position, Controller.Subdivision);
            
            Draw_Line_Renderer.Draw_Line(vertices);
        }
        else if (Draw_Line_Renderer != null)
        {
            Draw_Line_Renderer.Erase_Line();
        }
    }

    private void Erase_Path()
    {
        Draw_Line_Renderer = gameObject.GetComponent<DrawTrajectory>();
        if (Draw_Line_Renderer != null)
        {
            Draw_Line_Renderer.Erase_Line();
        }
    }

    public void MoveJ(float[] Angles, float Target_Time)
    {
        // Check for 6 coordinates
        if (Angles.Length != J)
        {
            throw new ArgumentException("Expected " + J + " joints.");
        }

        if (Target_Time < 0)
        {
            throw new ArgumentException("Expected non-negative value.");
        }

        Controller.Joints_Target = Angles;

        // Execute move
        for (int i = 0; i < J; i++)
        {
            Move_Joint[i].MoveJ(Angles[i], Target_Time: (100f / Velocity_Percentage * Target_Time));
        }
    }
    public void MoveL(float[] TCP, float Target_Time)
    {
        if (Target_Time < 0)
        {
            throw new ArgumentException("Expected non-negative value.");
        }

        // Set parameters                    
        MP.Linear_Busy = true;
        MP.Elapsed_Time = 0;
        MP.Target_Time = Target_Time;

        // Get Start position of Joints
        MP.Set_Start_Angles(MP.Current_Q);

        // Actual position of Target_TCP
        Move_TCP_ViewPoint(TCP);
        Controller_Inverse.Set_Target = true;
        Transform Target_Transform = Controller_Inverse.Target_Effector;

        // Set target TCP
        Matrix4x4 Target_TCP = KinematicsUtilities.Get_Transform_Matrix(Target_Transform);
        MP.Set_Target_TCP(Target_TCP);

        // Get prediction
        MP.Get_Linear_Path_Prediction(Controller.Subdivision);
        MP.Path_Linear.Segement_index = 0;

        // Fill-up Queues for MoveL command
        for (int s = 0; s < MP.Path_Linear.Trajectory_Q.GetLength(0); s++)
        {
            for (int j = 0; j < J; j++)
            {
                float Target_T = MP.Delta_Time;
                float Target_Q = MP.Path_Linear.Trajectory_Q[s, j];
                float Target_W = MP.Path_Linear.Trajectory_W[s, j];
                Move_Joint[j].MoveL(Target_Q, Target_W, Target_T);
            }
        }

        for (int j = 0; j < J; j++)
        {
            Move_Joint[j].Linear_Path.Execute = true;
        }
    }

    private bool MoveLinear()
    {
        bool Valid_Parameters;

        // Check for valid speed / subdivisions
        if (Controller.Target_Time / Controller.Subdivision < Time.fixedDeltaTime)
        {
            print("Velocity violation");
            Valid_Parameters = false;
            return Valid_Parameters;
        }

        // Set parameters                    
        MP.Linear_Busy = true;
        MP.Elapsed_Time = 0;
        MP.Target_Time = Controller.Target_Time;

        // Get Start position of Joints
        MP.Set_Start_Angles(MP.Current_Q);

        // Actual position of Target_TCP
        Transform Target_Transform = Controller_Inverse.Target_Effector;

        // Set target TCP
        Matrix4x4 Target_TCP = KinematicsUtilities.Get_Transform_Matrix(Target_Transform);
        MP.Set_Target_TCP(Target_TCP);

        // Get prediction
        MP.Get_Linear_Path_Prediction(Controller.Subdivision);
        MP.Path_Linear.Segement_index = 0;

        // Fill-up Queues for MoveL command
        for (int s = 0; s < MP.Path_Linear.Trajectory_Q.GetLength(0); s++)
        {     
            for (int j = 0; j < J; j++)
            {
                float Target_T = MP.Delta_Time;
                float Target_Q = MP.Path_Linear.Trajectory_Q[s, j];
                float Target_W = MP.Path_Linear.Trajectory_W[s, j];
                Move_Joint[j].MoveL(Target_Q, Target_W, Target_T);
            }
        }
        Valid_Parameters = true;
        return Valid_Parameters;
    }

    private void Set_Tool()
    {
        MP.Set_Tool(Controller.Tool);
        Tool = Controller.Tool;
        Controller.Set_Tool = false;
    }

    private void Move_TCP_ViewPoint(float[] Pose)
    {
        if (Pose.Length != 6)
        {
            throw new ArgumentException("Expected x, y, z, rx, ry, rz");
        }
        Controller_Inverse.Target_Effector.transform.position = new Vector3(Pose[0], Pose[1], Pose[2]);
        Controller_Inverse.Target_Effector.transform.rotation = Quaternion.Euler(Pose[3], Pose[4], Pose[5]);
    }

    // Get information about joint movement
    public bool Is_Robot_Moving()
    {
        bool Moving = false;
        for (int i = 0; i < J; i++)
        {
            if (Move_Joint[i].Busy ||  Move_Joint[i].Linear_Path.Execute || !Move_Joint[i].Stopped)
            {
                Moving = true;
                break;
            }
        }
        return Moving;
    }

    private void Script_Test_Rectangle(ProgramUtilities P)
    {
        float[] p1 = { 80, 111, 96, 60, -90, 168 };
        float[] p2 = { 0.1243f, 0.7635f, 1.154f, -180, 0, 0 };
        float[] p3 = { -0.1285f, 0.7635f, 1.154f, -180, 0, 0 };
        float[] p4 = { -0.1285f, 0.7635f, 0.937f, -180, 0, 0 };
        float[] p5 = { 0.1243f, 0.7635f, 0.937f, -180, 0, 0 };
        float[] p6 = { -180, 20, -20, 180, 90, 0 };
        float[] p7 = { -180, 120, -90, 60, 90, 0 };

        P.MoveJ(p1, 4f);
        P.MoveL(p2, 4f);
        P.MoveL(p3, 2f);
        P.MoveL(p4, 2f);
        P.MoveL(p5, 2f);
        P.MoveL(p2, 2f);
        P.MoveJ(p6, 4f);
        P.MoveJ(p7, 4f);
    }

    private void Script_Test_Rectangle_2(ProgramUtilities P)
    {
        float[] p1 = { 80, 111, 96, 60, -90, 168 };
        float[] p2 = { 0.1243f, 0.7635f, 1.154f, -180, 0, 0 };
        float[] p3 = { -0.1285f, 0.7635f, 1.154f, -180, 0, 0 };
        float[] p4 = { -0.1285f, 0.7635f, 0.937f, -180, 0, 0 };
        float[] p5 = { 0.1243f, 0.7635f, 0.937f, -180, 0, 0 };
        float[] p6 = { 0, 90, 90, 90, -90, 170 };
        float[] p7 = { 0, 130, 110, -65, -90, 170 };


        P.MoveJ(p1, 4f);
        P.MoveL(p2, 4f);
        P.MoveL(p3, 2f);
        P.MoveL(p4, 2f);
        P.MoveL(p5, 2f);
        P.MoveL(p2, 2f);
        P.MoveJ(p6, 4f);
        P.MoveJ(p7, 4f);
        P.MoveJ(p6, 4f);

    }

    private void TrajectoryTest(ProgramUtilities D)
    {

        float[] p1 = { 0, 0, 0, 0, 0, 0 };
        float[] p2 = { 0, 90, 0, 90,0, 0 };
        float[] p3 = { 0, 180, 0, 0, 0, 0 };
        float[] p4 = { 0, 90, 0, 0, 0, 0 };

        D.MoveJ(p1, 4f);
        D.MoveJ(p2, 4f);
        //D.MoveJ(p3, 4f);
        //D.MoveJ(p4, 4f);
        //D.MoveJ(p3, 4f);

    }
}




