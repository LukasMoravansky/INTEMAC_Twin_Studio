using UnityEngine;

/*
___________________________________________________________________________________________
  Script is handling inverse kinematics, transforms target view point to joint angles
___________________________________________________________________________________________
*/
public class InverseControl : MonoBehaviour
{
    public Transform Target_Effector;       // ViewPoint object in Unity
    private ForwardControl Controller;      // Main control script
    private KinematicPrams RobotParent;     // Script 

    private Matrix4x4 T_Tool = Matrix4x4.identity;    // Trasnform matrix of tool
    private Matrix4x4 T_Base = Matrix4x4.identity;    // Trasnform matrix of Base

    private int J;   // Number of joints

    public bool Set_Target;     // Sets ghost_robot according to viewpoint position (IK is calculated)

    void Start()
    {
        Controller = GetComponent<ForwardControl>();
        RobotParent = transform.parent.gameObject.GetComponent<KinematicPrams>();
        J = 6;
    }

    // Update is called once per frame
    void Update()
    {
        // Checks actual parameters
        T_Tool.m23 = Controller.Tool;
        T_Base = RobotParent.T_Robot_Base;

        // Set_Target called
        if (Set_Target)
        {
            float[] Taget_Angels = new float[6];
            Matrix4x4 Target_Matix = KinematicsUtilities.Get_Transform_Matrix(Target_Effector);
            // Convert unity coordinate system 
            Target_Matix = KinematicsUtilities.Convert_Coordinates(Target_Matix, true);       // convert coordinates from unity to script
            Taget_Angels = Get_Joint_Angles(Target_Matix, T_Tool, T_Base);
            Controller.Joints_Target = Taget_Angels;

            Set_Target = false;
        }
    }
    // Returns one solution of IK on given target transfom matrix
    public float[] Get_Joint_Angles(Matrix4x4 Target_Matrix, Matrix4x4 T_Tool = default, Matrix4x4 T_Base = default, float[] Angles = null)
    {
        MovementUtilities MU = new MovementUtilities();

        // Compute Inverse Kinematics
        float[,] Joint_Angles = new float[8, 6];
        Joint_Angles = RobotKinematics.Calculate_Inverse_Kinematics_FULL(Target_Matrix, T_Tool, T_Base);

        // Get target solution
        if (Angles == null)
        {
            Angles = Controller.Joints_Target;
        }

        int idx = MU.Get_Closest_Solution(Angles, Joint_Angles);

        float[] Target_Angles = new float[6];
        // Set Angles
        for (int i = 0; i < J; i++)
        {
            Target_Angles[i] = Joint_Angles[idx, i];
        }
        return Target_Angles;
    }
}
