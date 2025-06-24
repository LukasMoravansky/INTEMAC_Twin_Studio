using System.Collections.Generic;
using UnityEngine;


public class JointControl : MonoBehaviour
{
    /*
    ______________________________________________________________________________________________
    Description:
    The JointControl class manages the movement of a robot joint in Unity using an ArticulationBody.
    It provides methods for setting target angles, calculating motion trajectories, and controlling 
    the joint in real-time

    The script controls only one selected joint
    _______________________________________________________________________________________________
    */

    // Joint Object specification
    public GameObject Joint_Object;      //Referenced Joint on main Robot structure
    private Transform Joint_Transform;      // Transform of referenced Joint
    public float Joint_Actual_Rotation { get; private set; }
    private ArticulationBody Joint_Articulation;        // Articulation proerties of referenced Joint
    private ArticulationReducedSpace Joint_Actual_Velocity;         // Angular velocity from Articulation body
    private ArticulationReducedSpace Joint_Actual_Acceleration;         // Angular velocity from Articulation body
    private float Calculated_Actual_Velocity;
    private float Last_Pos;

    // Move specifications
    public float Angular_Velocity_Safe = 30;
    public float Angular_W { get; private set; }                            // Calculated angular velocity
    public float Start_Q { get; private set; }                      // Parameter is set in MoveJ
    public float Target_Angle { get; private set; }                 // Parameter is set in MoveJ
    public enum MoveType { Trapezoidal, Polynomial, Linear }
    private MoveType Move_Type;
    private MovementUtilities Movement_Calculator;

    // Inputs
    float Move_Time;

    // MoveL struct used for handling Linear movement
    public struct PlannedPath 
    {
        public Queue<(float, float, float)> Segments;  // Target Angle, Target Velocity, Target Time
        public int idx;                     // Segment indicator

        // Control
        public bool Takt;                   // Sychronization command 
        public float Elapsed_Time;          // Elapsed time of segment (UNUSED)

        // States
        public bool Execute;                // Start commmand
        public bool Done;                   // Path done
        public bool In_Position;            // Segment done 

        // Parameters
        public float Actual_Q;              // Joint Angle
        public float Actual_W;              // Angular Velocity
        public float Actual_T;              // Target Time of segment
    }
    public PlannedPath Linear_Path;

    // Outputs
    public bool Done { get; private set; }
    public bool Busy { get; private set; }
    public bool Ready { get; private set; }
    public bool Stopped { get; private set; }

    // Other
    public float Elapsed_Time { get; private set; } // Elapsed time since the start of the movement

    // Controller
    private bool PID_enable;
    PID_controller PID_Position;
    PID_controller PID_Velocity;

    // Joint structure
    public ArticulationDrive Drive;

    // Start is called before the first frame update
    void Start()
    {
        Ready = true;
        Busy = false;
        //Done = true;
        Drive.stiffness = float.MaxValue;
        Joint_Articulation.mass = 1;
        Drive.damping = 100;
        //Drive.forceLimit = 10000000;
        Drive.forceLimit = float.MaxValue;
        Drive.driveType = ArticulationDriveType.Acceleration;
        Joint_Articulation.xDrive = Drive;
        Joint_Articulation.useGravity = false;
        Joint_Articulation.angularDamping = 0;

        // Trajectory specifications
        Movement_Calculator = new MovementUtilities();
        Movement_Calculator.Init();

        // MoveL
        Linear_Path = new PlannedPath();
        Linear_Path.Segments = new Queue<(float, float, float)>();

        // PID
        PID_enable = false;
        PID_Position = new PID_controller();
        PID_Position.Kp = 0.5f;//0.5f;
        PID_Position.Ki = 0.000001f;
        PID_Position.Kd = 0.005f;
        PID_Velocity = new PID_controller();
        PID_Velocity.Kp = 0.1f;//1.0f;
        PID_Velocity.Ki = 0f;
        PID_Velocity.Kd = 0.0002f; //0.2f;

    }
    // Init of Joints
    public void Init(GameObject Joint)
    {
        // Referenced Joint
        Joint_Object = Joint;
        Joint_Transform = Joint_Object.GetComponent<Transform>();
        Joint_Articulation = Joint_Object.GetComponent<ArticulationBody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Get actual parameters
        Joint_Actual_Velocity = Joint_Articulation.jointVelocity;
        Joint_Actual_Rotation = Joint_Articulation.jointPosition[0] * Mathf.Rad2Deg;
        Joint_Actual_Acceleration = Joint_Articulation.jointAcceleration;
        
        float Delta_Time = Time.fixedDeltaTime;

        if (Linear_Path.Execute)
        {
            MoveL_Execution();
        }

        // Calculate parameters for move (depending on actual time)
        if (Busy)
        {
            Movement_Calculator.Get_Motion(Elapsed_Time);
            Elapsed_Time += Delta_Time;
            Angular_W = Movement_Calculator.w;
            Drive.target = Movement_Calculator.s;
        }
        else
        {
            Angular_W = 0f;
        }

        // PID controll for angular velocity (used for MoveL)
        if (PID_enable)
        {
             Angular_W = PID_Velocity.UpdatePID(Movement_Calculator.w, Joint_Actual_Velocity[0] * Mathf.Rad2Deg, Delta_Time);
        }

        // Move Joint - Set parameters for articulation body
        Drive.targetVelocity = Angular_W;
        //Drive.target = Movement_Calculator.s;
        Joint_Articulation.xDrive = Drive;

        // Check if movement is finished
        In_Position(Target_Angle);

        // Check if Stopped
        Calculated_Actual_Velocity = (Joint_Actual_Rotation - Last_Pos) / Time.fixedDeltaTime;

        if (Calculated_Actual_Velocity > -0.01 && Calculated_Actual_Velocity < 0.01)
        {
            Stopped = true;
        } else
        {
            Stopped = false;
        }
        Last_Pos = Joint_Actual_Rotation;
    }

    public bool In_Position(float Target)
    // Checks if Joint is in target postion
    {
        bool InPosition = false;
        float Actual = Joint_Articulation.jointPosition[0] * Mathf.Rad2Deg;

        if (Mathf.Abs(Actual - Target) < 0.001)
        {
            InPosition = true;
            Done = true;
            Ready = true;
            Busy = false;
        }
        else
        {
            Done = false;
            Ready = false;
        }
        return InPosition;

    }

    public bool Is_Home()
    // Check if Joint is in Home position 
    {
        bool isHome = false;
        /* TODO:
            At this point function takes 0° as home

        */

        isHome = In_Position(0f);

        return isHome;
    }

    public void Stop()
    {
        Movement_Calculator.Stop(Joint_Actual_Rotation);
        Movement_Calculator.Start_S = Joint_Actual_Rotation;
        Movement_Calculator.Target_T = 0.1f;
        Movement_Calculator.Target_S = Joint_Actual_Rotation;
        Target_Angle = Joint_Actual_Rotation;

        Stopped = true;
        Busy = false;
    }

    // Sets parameters for movement form start angle to target angle
    public void MoveJ(float Target_Q, float Start_Vel = 0, float Target_Vel = 0, float Target_Time = 0, MovementUtilities.MoveType Input_Move_Type = MovementUtilities.MoveType.Polynomial)
    {
        // Command preprocesses parameters for move
        Start_Q = Joint_Actual_Rotation;
        Target_Angle = Target_Q;

        // Set parameters for joint movement
        Movement_Calculator.Start_S = Start_Q;
        Movement_Calculator.Target_T = Target_Time;
        Movement_Calculator.Target_S = Target_Q;
        Movement_Calculator.Start_Velocity = Start_Vel;
        Movement_Calculator.Target_Velocity = Target_Vel;
        Movement_Calculator.Move_Type = Input_Move_Type;

        if (Linear_Path.Execute)
        {
            Elapsed_Time = 0.005f;
        }
        else
        {
            Elapsed_Time = 0;
        }

        // Set States
        Busy = true;
        Ready = false;
    }

    // Preprocess MoveL command
    public void MoveL(float Target_Q, float Target_W, float Target_T)
    {
        // Create Tuple
        (float Target_Q, float Target_W, float Target_T) Segment_Protperties = (Target_Q, Target_W, Target_T);

        // Add Tuple to Queue
        Linear_Path.Segments.Enqueue(Segment_Protperties);
        Linear_Path.idx = 0;
        Linear_Path.Done = false;
        Elapsed_Time = 0;
    }

    // Executing MoveL
    private void MoveL_Execution()
    {
        // Command validation check
        if (Linear_Path.Segments.Count == 0)
        {
            Linear_Path.Done = true;
            Linear_Path.Execute = false;
            PID_enable = false;
            Busy = false;
            return;
        }

        PID_enable = true;

        // Check if segment is done
        if (Elapsed_Time > Linear_Path.Actual_T)
        {
            Linear_Path.In_Position = true;
        }
        else
        {
            Busy = true;
            Linear_Path.In_Position = false;
        }

        // Synchronization with other joints
        if (!Linear_Path.Takt)
        {         
            // if "takt" insn't called, another segment is not executed
            return;
        }
        else
        {
            Linear_Path.Elapsed_Time = 0;
            Linear_Path.In_Position = false;
            Linear_Path.Takt = false;
        }

        // Get parameters from queue
        var Segment = Linear_Path.Segments.Dequeue() ;
        (float Target_Q, float Target_W, float Target_T) = Segment;

        // Call MoveJ between segments
        MoveJ(Target_Q, Linear_Path.Actual_W, Target_W, Target_T, Input_Move_Type: MovementUtilities.MoveType.Linear_Velocity);

        // Write parameters form tuple
        Linear_Path.Actual_Q = Target_Q;
        Linear_Path.Actual_W = Target_W;
        Linear_Path.Actual_T = Target_T;
        Linear_Path.idx += 1;               // index of current segment
    }

}
