using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicPrams : MonoBehaviour
{
    /*
     Script is used to get T_Robot_Base parameter (Base position in Unity enviroment)
    */

    private Transform RobotPos;
    [HideInInspector]
    public Matrix4x4 T_Robot_Base;    

    // Start is called before the first frame update
    void Start()
    {
        RobotPos = GetComponent<Transform>();
    }

    // Update is called once per frame
    void Update()
    {
        Matrix4x4 T_Temp_Unity_Coors = KinematicsUtilities.Get_Transform_Matrix(RobotPos);
        T_Robot_Base = KinematicsUtilities.Convert_Coordinates(T_Temp_Unity_Coors, true);
    }
}
