using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;
//using static UnityEditor.ShaderGraph.Internal.KeywordDependentCollection;

public class Gripper : MonoBehaviour
{
    public GameObject Gripper_Reference_Object;
    public GameObject Robot_Model;
    public GameObject Pickup_Plane;
    private Robot Robot_Skript;
    private Ctrl_Robotiq_2F_85 Gripper_Control;
    public GameObject Ghost_Gripper;
    private Ctrl_Robotiq_2F_85 Ghost_Gripper_Script;
    [Range(0, 50)]
    public float Stroke;
    public bool Execute;                        // Execute gripper stroke
    [HideInInspector]
    public bool Pick_up_Object;
    public bool Execute_Picking_Seqeunce;       // if true - sequence is executed. Checked only on the start of sequece 
    public bool Kinematic_Grab;
    private bool Grab_Execute;
    private InverseControl IK_Controller;
    private ForwardControl Controller;

    // Stroke
    private float Previous_Stroke;

    // Gripper colliders
    private Collider[] Finger_Colliders;
    public CollisionDetection[] Finger_Collision_Script;

    public GameObject Object_1;
    public GameObject Object_2;
    public GameObject Object_3;
    public GameObject Object_4;

    // picked-up object properties
    public struct PickedObjectProperties 
    {
        public GameObject[] Object_To_Pick;
        public int Object_Count;
        public int Selected_Object;
    }
    private int s;  // internal
    public PickedObjectProperties Picking;

    // Movement States
    private enum PickupStates
    {
        Idle,
        Set_To_Initial,
        Execute_To_Init,
        Moving_To_Init,
        Set_To_Object,
        Execute_To_Object,
        Moving_To_Object,
        Grab_Position,
        Grabbing,
        Set_To_Pickup,
        Pickup_Execute,
        Moving_To_Target,
        Show_Setup,
        Move_To_Show,
        Show_Rotate,
        Move_Before_Drop,
        Move_To_Drop_Linear,
        Move_To_Drop,
        Move_After_Drop,
        Finish_Sequence,
        Error
    }

    private PickupStates Pickup_States;
    public bool Object_Grabbed { get; private set; }

    // Compression
    public struct CompressionPropperties
    {
        public float Stiffness;
        public float Damping;
        public Vector3[] Previous_Gripper_Position;
        public Vector3 Force;
    }
    private CompressionPropperties Compression;

    // Posiitons
    float[] Current_Init_Pos;
    float[] Drop_Position;
    float[] Pickup_Position;

    // internal
    private float Wait_Time;

    // Start is called before the first frame update
    void Start()
    {
        Gripper_Control = Gripper_Reference_Object.GetComponent<Ctrl_Robotiq_2F_85>();
        Ghost_Gripper_Script = Ghost_Gripper.GetComponent<Ctrl_Robotiq_2F_85>();
        IK_Controller = GetComponent<InverseControl>();
        Controller = GetComponent<ForwardControl>();
        Robot_Skript = Robot_Model.GetComponent<Robot>();
        Finger_Colliders = Gripper_Reference_Object.GetComponentsInChildren<Collider>();
        for (int i = 0; i < Finger_Colliders.Length; i++)
        {
            Finger_Colliders[i].contactOffset = float.Epsilon;
        }

        // Compression
        Compression = new CompressionPropperties();
        Compression.Stiffness = 200f;
        Compression.Damping = 100f;
        Compression.Previous_Gripper_Position = new Vector3[Finger_Colliders.Length];

        // Default setup
        Kinematic_Grab = true;

        Pickup_States = PickupStates.Idle;

        // Obejct to pick properties
        Picking.Object_Count = 4;
        Picking.Object_To_Pick = new GameObject[Picking.Object_Count];
        Picking.Object_To_Pick[0] = Object_1;
        Picking.Object_To_Pick[1] = Object_2;
        Picking.Object_To_Pick[2] = Object_3;
        Picking.Object_To_Pick[3] = Object_4;

        Picking.Selected_Object = 0;
        s = Picking.Selected_Object;

        // Adds collision detector skript on fingers
        Add_Detection_On_Fingers();

        // Move Robot to Home position
        Controller.Move_Home();

        // Postion memory init
        Current_Init_Pos = new float[6];
    }

    // Update is called once per frame
    void Update()
    {
        s = Picking.Selected_Object;

        // Error State
        if (Controller.Error)
        {
            Execute_Picking_Seqeunce = false;
            Pickup_States = PickupStates.Error;
        }

        // Stroke Execute
        if (Execute)
        {
            Ghost_Gripper_Script.stroke = Stroke;
            Ghost_Gripper_Script.speed = 150.0f;
            Ghost_Gripper_Script.start_movemet = true;
            Gripper_Control.stroke = Stroke;
            Gripper_Control.start_movemet = true;            
            Check_For_Loose();
            Previous_Stroke = Stroke;
            Execute = false;
        }

        // Internal bool for start sequence
        if (Pick_up_Object)
        {
            Pickup_States = PickupStates.Set_To_Initial;

            Pick_up_Object = false;
        }

        switch (Pickup_States)
        {
            case PickupStates.Idle:
                {
                    if (Execute_Picking_Seqeunce)
                    {
                        Pick_up_Object = true;
                    }
                    break;
                }

            // Initial setup
            case PickupStates.Set_To_Initial:
                {
                    Controller.Target_Time = 2;
                    Set_Pickup_Point(0.15f, Picking.Object_To_Pick[s]);
                    IK_Controller.Set_Target = true;
                    Pickup_States = PickupStates.Execute_To_Init;

                    // Settup collider for target
                    if (Picking.Object_To_Pick[s] != null)
                    {
                        Collider Target_Collider = Picking.Object_To_Pick[s].GetComponent<Collider>();
                        Target_Collider.contactOffset = float.Epsilon;
                    }
                    break;
                }

            // Setup ghost position
            case PickupStates.Execute_To_Init:
                {
                    Current_Init_Pos = Controller.Joints_Target;
                    Controller.Execute = true;
                    Pickup_States = PickupStates.Moving_To_Init;
                    Wait_Time = 0f;
                    break;
                }

            // Move
            case PickupStates.Moving_To_Init:
                {
                    Wait_Time += Time.deltaTime;
                    if (Wait_Time < 1f)
                    {
                        break;
                    }

                    if (!Robot_Skript.Is_Robot_Moving() && !Object_Grabbed)
                    {
                        Pickup_States = PickupStates.Set_To_Object;
                    }
                    if (!Robot_Skript.Is_Robot_Moving() && Object_Grabbed)
                    {
                        Pickup_States = PickupStates.Idle;
                    }
                    break;
                }


            case PickupStates.Set_To_Object:
                {
                    Set_Pickup_Point(0.01f, Picking.Object_To_Pick[s]);
                    Pickup_Position = Controller.Joints_Target;
                    IK_Controller.Set_Target = true;
                    Pickup_States = PickupStates.Execute_To_Object;
                    break;
                }

            case PickupStates.Execute_To_Object:
                {
                    Controller.Execute_Linear = true;
                    Pickup_States = PickupStates.Moving_To_Object;
                    Wait_Time = 0;
                    break;
                }

            // Moves Lineary near object. Opens gripper
            case PickupStates.Moving_To_Object:
                {
                    Drop_Position = Controller.Joints_Target;
                    Wait_Time += Time.deltaTime;
                    if (Wait_Time < 1f)
                    {
                        break;
                    }

                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        Stroke = 50;
                        Execute = true;
                    }

                    if (Gripper_Control.in_position && !Robot_Skript.Is_Robot_Moving())
                    {
                        Pickup_States = PickupStates.Grab_Position;
                    }
                    break;
                }

            // Moves on object level with gripper oppened
            case PickupStates.Grab_Position:
                {
                    Set_Pickup_Point(-0.005f, Picking.Object_To_Pick[s]);
                    Controller.Execute_Linear = true;

                    Pickup_States = PickupStates.Grabbing;
                    Wait_Time = 0;

                    break;
                }

            case PickupStates.Grabbing:
                {
                    if (!Robot_Skript.Is_Robot_Moving() && !Object_Grabbed)
                    {
                        Grab_Execute = true;
                    }
                    else if (Object_Grabbed)
                    {
                        Grab_Execute = false;
                        Wait_Time += Time.deltaTime;
                        if (Wait_Time >= 1f)
                        {
                            Pickup_States = PickupStates.Set_To_Pickup;
                        }
                    }
                    break;
                }

            case PickupStates.Set_To_Pickup:
                // Position above plane with picked up object
                {
                    float PICKUP_DISTANCE = 0.15f;   // Distance above plane
                    Matrix4x4 Pickup_Pos = KinematicsUtilities.Get_Transform_Matrix(Pickup_Plane.transform);
                    Transform Object_Transform = Picking.Object_To_Pick[s].transform;
                    Pickup_Pos.m03 = Object_Transform.position.x;
                    Pickup_Pos.m13 = Pickup_Plane.transform.position.y + PICKUP_DISTANCE;
                    Pickup_Pos.m23 = Pickup_Plane.transform.position.z;


                    IK_Controller.Target_Effector.position = new Vector3(Pickup_Pos.m03, Pickup_Pos.m13, Pickup_Pos.m23);
                    Vector3 Plane_Rotation = Pickup_Plane.transform.rotation.eulerAngles;
                    Quaternion Target_Rotation = Quaternion.Euler(180, Plane_Rotation.y, Plane_Rotation.z);
                    IK_Controller.Target_Effector.rotation = Target_Rotation;
                    IK_Controller.Set_Target = true;
                    Pickup_States = PickupStates.Pickup_Execute;
                    break;
                }

            case PickupStates.Pickup_Execute:
                {
                    Controller.Execute = true;
                    //Drop_Position = Controller.Joints_Target;
                    Pickup_States = PickupStates.Moving_To_Target;
                    break;
                }

            case PickupStates.Moving_To_Target:
                {
                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        Pickup_States = PickupStates.Show_Setup;
                    }
                    break;
                }

            case PickupStates.Show_Setup:
                {
                    float[] Position_1 = { 98.86819f, 114.4171f, 98.99996f, 16.99945f, 52.5984f, -160.7045f };
                    Controller.Joints_Target = Position_1;
                    Controller.Target_Time = 4;
                    Controller.Execute = true;
                    Pickup_States = PickupStates.Move_To_Show;
                    break;
                }

            case PickupStates.Move_To_Show:
                {
                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        float[] Position_2 = { 98.86819f, 114.4171f, 98.99996f, 16.99945f, 52.5984f, 146f };
                        Controller.Joints_Target = Position_2;
                        Controller.Target_Time = 5;
                        Controller.Execute = true;
                        Pickup_States = PickupStates.Show_Rotate;
                    }
                    break;
                }

            case PickupStates.Show_Rotate:
                {
                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        Controller.Joints_Target = Pickup_Position;
                        Controller.Target_Time = 4;
                        Controller.Execute = true;
                        Pickup_States = PickupStates.Move_Before_Drop;
                    }
                    break;
                }

            case PickupStates.Move_Before_Drop:
                {
                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        Controller.Joints_Target = Drop_Position;
                        Controller.Target_Time = 4;
                        Pickup_States = PickupStates.Move_To_Drop_Linear;
                    }
                    break;
                }

            case PickupStates.Move_To_Drop_Linear:
                {
                    Controller.Execute = true;
                    Pickup_States = PickupStates.Move_To_Drop;
                    break;
                }

            case PickupStates.Move_To_Drop:
                {
                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        // Drop
                        Stroke = 42f;
                        Execute = true;
                        Pickup_States = PickupStates.Move_After_Drop;
                        Wait_Time = 0;
                    }
                    break;
                }

            case PickupStates.Move_After_Drop:
                {
                    Wait_Time += Time.deltaTime;
                    if (Wait_Time >= 0.5f)
                    {
                        Controller.Joints_Target = Current_Init_Pos;
                        Controller.Target_Time = 2;
                        Controller.Execute = true;
                        Pickup_States = PickupStates.Finish_Sequence;
                    }
                    break;
                }


            case PickupStates.Finish_Sequence:
                { 
                    if (!Robot_Skript.Is_Robot_Moving())
                    {
                        Picking.Selected_Object += 1;
                        Picking.Selected_Object = (Picking.Selected_Object) % Picking.Object_Count;
                        Change_Detection_On_Fingers();
                        Pickup_States = PickupStates.Idle;
                    }
                    break;
                }

            case PickupStates.Error:
                {
                    if (!Controller.Error)
                    {
                        Controller.Move_Home();
                        Pickup_States = PickupStates.Idle;
                    }
                    break;
                }
        }
    }

    private void FixedUpdate()
    {
        if (Grab_Execute)
        {
            Grab_Object();
        }

        if (Detect_Contact() && !Kinematic_Grab)
        {
            Add_Conpression();
        }
        //else if (!Detect_Contact() && Kinematic_Grab)
        //{
        //    Object_To_Pick.transform.parent = null;
        //    Rigidbody ObjectBody = Object_To_Pick.GetComponent<Rigidbody>();
        //    ObjectBody.isKinematic = false;

        //}

        else if (!Detect_Contact() && !Kinematic_Grab)
        {
            for (int g = 0; g < Finger_Colliders.Length; g++)
            {
                Compression.Previous_Gripper_Position[g] = Finger_Colliders[g].transform.position;
            }
            Rigidbody ObjectBody = Picking.Object_To_Pick[s].GetComponent<Rigidbody>();
            var ForceAccumulated = ObjectBody.GetAccumulatedForce();
            ObjectBody.AddForce(ForceAccumulated * -1f, ForceMode.Force);
            Object_Grabbed = false;
        }

    }

    private void Set_Pickup_Point(float Distance, GameObject Object)
    {
        // Get object position + pickup distance
        Vector3 Position = Object.transform.position;
        Position.y += Distance;
        // Set viewpoint posiiton
        IK_Controller.Target_Effector.position = Position;
        // Get Object roation
        IK_Controller.Target_Effector.rotation = Picking.Object_To_Pick[s].transform.rotation;
        // Rotate Transform Matrix if object id flipped
        float X_ROTATION = -90f;

        float Tolerance = 0.01f;
        float Dot_Product = Vector3.Dot(Object.transform.forward, Vector3.up);

        if (Dot_Product < -1 + Tolerance) // Flip Axis
        {
            X_ROTATION += 180f;
        }
        IK_Controller.Target_Effector.Rotate(X_ROTATION, 0f, 0f);
    }
    private void Grab_Object()
    {
        bool Grabbed = Detect_Contact();

        // Check if all fingers are in touch with target object
        Object_Grabbed = Grabbed;
        if (Grabbed)
        {
            //Stroke = Stroke + 10 * 20 * Time.fixedDeltaTime;
            if (Kinematic_Grab)
            {
                Picking.Object_To_Pick[s].transform.parent = Gripper_Reference_Object.transform;
                Rigidbody ObjectBody = Picking.Object_To_Pick[s].GetComponent<Rigidbody>();
                ObjectBody.isKinematic = true;
            }
            //Stroke -= 0.1f;
            //Execute = true;
            return;
        }

        if (!Grabbed && Gripper_Control.in_position)
        {
            Stroke = Stroke - 5 * Time.fixedDeltaTime;
            Execute = true;
        }
    }
    private void Get_Collision_Script()
    {
        Finger_Collision_Script = new CollisionDetection[Finger_Colliders.Length];
        for (int i = 0; i < Finger_Colliders.Length; i++)
        {
            Finger_Collision_Script[i] = Finger_Colliders[i].gameObject.GetComponent<CollisionDetection>();
            Add_Object_To_Detect(Finger_Collision_Script[i]);
        }
    }


    private void Add_Detection_On_Fingers()
    {
        // Skript has to be added on gameobject containing Rigid or Articulation Body
        Transform ParentTransform = Gripper_Reference_Object.transform.parent;
        GameObject ParentObject = ParentTransform.gameObject;       // Skript goes here, object can evaluate collision event, because is it last articulation body.
        CollisionDetection Skript = ParentObject.gameObject.AddComponent<CollisionDetection>();
        Add_Object_To_Detect(Skript);
    }

    private void Change_Detection_On_Fingers()
    {
        // Skript has to be added on gameobject containing Rigid or Articulation Body
        Transform ParentTransform = Gripper_Reference_Object.transform.parent;
        GameObject ParentObject = ParentTransform.gameObject;       // Skript goes here, object can evaluate collision event, because is it last articulation body.
        CollisionDetection Skript = ParentObject.gameObject.GetComponent<CollisionDetection>();
        Add_Object_To_Detect(Skript);
    }

    private void Add_Object_To_Detect(CollisionDetection Skript)
    {
        Skript.Target_Object = Picking.Object_To_Pick[Picking.Selected_Object].GetComponent<Collider>();
    }

    private bool Detect_Contact()
    {
        bool Grabbed = false;

        List<Collider> Lookup_List = new List<Collider>();
        for (int i = 0; i < Finger_Colliders.Length; i++)
        {
            Lookup_List.Add(Finger_Colliders[i]);
        }

        CollisionDetection Detection_Script = Gripper_Reference_Object.transform.parent.gameObject.GetComponent<CollisionDetection>();
        Collider Target_Collider = Picking.Object_To_Pick[s].GetComponent<Collider>();

        if (Detection_Script.Collision_Details == null || Detection_Script.Collision_Details.Count == 0)
        {
            return Grabbed;
        }

        foreach (var Collision_Details in Detection_Script.Collision_Details)
        {
            foreach (ContactPoint Col in Collision_Details.contacts)
            {
                for (int i = 0; i < Finger_Colliders.Length; i++)
                {
                    if (Lookup_List.Contains(Col.thisCollider) && Col.otherCollider == Target_Collider)
                    {
                        Lookup_List.Remove(Col.thisCollider);
                    }
                }

                // Check if Lookup_Table is empty
                if (Lookup_List.Count == 0)
                {
                    Grabbed = true;
                }
            }
        }
        return Grabbed;

    }
    private void Add_Conpression()
    {
        //for (int g = 0; g < Finger_Colliders.Length; g++)
        //{
        int g = 0;
        float Distance = Mathf.Abs(Finger_Colliders[g].transform.position.x - Picking.Object_To_Pick[s].transform.position.x);
        float Elastic_Force = - Compression.Stiffness * Distance;
        Vector3 Gripper_Velocity = (Finger_Colliders[g].transform.position - Compression.Previous_Gripper_Position[g]) / Time.fixedDeltaTime;
        Vector3 Object_Velocity = Picking.Object_To_Pick[s].GetComponent<Rigidbody>().velocity;
        Vector3 Relative_Velocity = Gripper_Velocity - Object_Velocity;

        // Damping
        Vector3 Damping_Force = -Compression.Damping * Relative_Velocity;

        // Apply force
        int Sign = 1;
        if (g != 0)
        {
            Sign *= -1;
        }
        Vector3 El_Force = new Vector3(Sign * Elastic_Force, 0, 0);
        Vector3 Damp = new Vector3(Damping_Force.x, 0 ,0);
        Vector3 Sum_Force = El_Force + Damp;
        Sum_Force.x = Mathf.Clamp(Sum_Force.x, -10f, 10f);
        Compression.Force = Sum_Force;
        if (Sum_Force.x > 0f)
        {
            return;
        }
        Picking.Object_To_Pick[s].GetComponent<Rigidbody>().AddForce(Sum_Force);

        Compression.Previous_Gripper_Position[g] = Finger_Colliders[g].transform.position;
        //}
    }

    private void Check_For_Loose()
    {
        if (Previous_Stroke < Stroke && Object_Grabbed)
        {
            Rigidbody Object_Body = Picking.Object_To_Pick[s].GetComponent<Rigidbody>();
            Object_Body.isKinematic = false;
            Picking.Object_To_Pick[s].transform.parent = null;
            Object_Grabbed = false;
        }
    }

}
