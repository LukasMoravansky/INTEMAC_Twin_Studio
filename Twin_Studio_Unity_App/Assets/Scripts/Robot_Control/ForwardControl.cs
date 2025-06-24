
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class ForwardControl : MonoBehaviour
{
    /*
    Main script for controling robot. 
    Ghost-robot indicates target position stored in Joints_Target array
    */

    private Transform[] All_Joints ;
    [HideInInspector]
    public Transform[] Joints_Object;

    private string[] JointNames = {"Joint_1",
                                   "Joint_2",
                                   "Joint_3",
                                   "Joint_4",
                                   "Joint_5",
                                   "Joint_6" };

    // Joints_Target is used for forward kinematics. Angles of ghost robot are set according to values in thisa array
    [Range(-180, 180)]  
    public float[] Joints_Target;

    [HideInInspector]  public float Target_Velocity;
    [Min(0)] public float Target_Time;
    [Range(10, 400)]
    public int Subdivision;

    // Control buttons
    public bool Execute;
    public bool Execute_Linear;
    public bool Go_Home;
     public bool Execute_Script;
     public bool Show_Prediction;
    [HideInInspector] public bool Stop_Movement;
    [HideInInspector] public bool Homing;
    [HideInInspector] public bool Error;

    // Kinematic parameters
    public float Tool;  // Z-axis of T in [m]
    public bool Set_Tool;
    void Start()
    {
        Target_Velocity = 0;
        Target_Time = 4;
        Joints_Object = new Transform[6];
        Joints_Target = new float[6];
        int i = 0;
        All_Joints = GetComponentsInChildren<Transform>();
        foreach (Transform child in All_Joints)
        {
            if (JointNames.Contains(child.name) && i <= 6 ) 
            {
            Joints_Object[i] = child.GetComponent<Transform>();
            i += 1;
            }
        }
        // Line Rendererr smoothness
        Subdivision = 40;

        // Temporary, ROBOTIQ 
        Tool = 0.1628f;
    }

    // Update is called once per frame
    void Update()
    {
        for (int ii = 0; ii < Joints_Target.Length; ii++)
        {
            if (float.IsNaN(Joints_Target[ii]))
            {
                Debug.LogError("Invalid Target Angle");
                Execute = false;
                Execute_Linear = false;
                Error = true;
                break;
            }
            else
            {
                Error = false;
            }
            float default_X_rotation = Joints_Object[ii].transform.localEulerAngles.x;
            float default_Y_rotation = Joints_Object[ii].transform.localEulerAngles.y;
            Joints_Object[ii].transform.localEulerAngles = new Vector3(default_X_rotation, 0, Joints_Target[ii]);
        }

        if (Go_Home)
        {
            Move_Home();
            Go_Home = false;
        }
    }

    // Sets Angles for home position
    public void Move_Home()
    {
        float[] POSITION = { 80, 111, 96, 60, -90, 168 };
        Joints_Target = POSITION;
        Execute = true;
    }
}
