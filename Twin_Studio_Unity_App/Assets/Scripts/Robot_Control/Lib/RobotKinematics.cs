using Opc.Ua;
using Org.BouncyCastle.Asn1.X509;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using UnityEngine;
using UnityEngine.UIElements;
using static UnityEngine.Rendering.DebugUI;


public class RobotKinematics
{
    // DH Table UR10e
    // in order: | Theta | a | d | alpha |
    public static float[,] DH_Table = new float[,]
    {
        { 0,     0f,         0.181f,    Mathf.PI / 2},     // Joint 1
        { 0 ,    0.6127f,   0f,            0  },           // Joint 2
        { 0,     0.57155f,  0f,            0 },           // Joint 3
        { 0 ,     0f,         0.17415f,  Mathf.PI / 2 },    // Joint 4
        { 0,     0f,         0.120f,   - Mathf.PI / 2 },   // Joint 5
        { 0,     0f,        0.117f,      0  }           // Joint 6
    };

    // Method to calculate the forward kinematics
    public static Matrix4x4 CalculateForwardKinematics(float[] jointAngles, Matrix4x4 T_Tool = default, Matrix4x4 T_Base = default)
    {
        // Initial Matrices
        Matrix4x4 T = Matrix4x4.identity;
        Matrix4x4 Tb = Matrix4x4.identity;
        Matrix4x4 Tp = Matrix4x4.identity;

        // Iterate through each joint and calculate the transformation
        for (int i = 0; i < DH_Table.GetLength(0); i++)
        {
            float a;
            float alpha;
            float theta = jointAngles[i] * Mathf.Deg2Rad + DH_Table[i, 0]; // theta + joint angle

                a = DH_Table[i, 1];
                alpha = DH_Table[i, 3];
            float d = DH_Table[i, 2];
            

            // Generate the transformation matrix for the current joint
            Matrix4x4 Ti = CalculateTransformMatrix(theta, a, d, alpha);

            // Multiply the current transformation with the overall transformation
            T = T * Ti;
        }
       //T = T_Base * T * T_Tool;
        return T ;
    }

    /* Method to calculate the transformation matrix based on DH parameters
            Get transform matrix of frame i to frame i+1
            Input joint is joint "i"
    */
    private static Matrix4x4 CalculateTransformMatrix(float theta, float a, float d, float alpha)
    {
        Matrix4x4 T = new Matrix4x4();

        T[0, 0] = Mathf.Cos(theta);
        T[0, 1] = -Mathf.Sin(theta) * Mathf.Cos(alpha); ;
        T[0, 2] = Mathf.Sin(theta) * Mathf.Sin(alpha);
        T[0, 3] = a * Mathf.Cos(theta);

        T[1, 0] = Mathf.Sin(theta);
        T[1, 1] = Mathf.Cos(theta) * Mathf.Cos(alpha);
        T[1, 2] = -Mathf.Cos(theta) * Mathf.Sin(alpha);
        T[1, 3] = a * Mathf.Sin(theta);

        T[2, 0] = 0;
        T[2, 1] = Mathf.Sin(alpha);
        T[2, 2] =  Mathf.Cos(alpha);
        T[2, 3] =  d;
        
        T[3, 0] = 0;
        T[3, 1] = 0;
        T[3, 2] = 0;
        T[3, 3] = 1;

        return T;
    }

    public static float[ , ] Calculate_Inverse_Kinematics_FULL(Matrix4x4 TCP, Matrix4x4 T_Tool = default, Matrix4x4 T_Base = default)
    /*
     Solving Inverse Kinematics according to Ryan Keating and Noah J. Cowan (UR5 Inverse Kinematics, 2016)
        pdf URL: https://tianyusong.com/wp-content/uploads/2017/12/ur5_inverse_kinematics.pdf
        P - Position (Vector3)
        T - Transform matrix (Matrix4x4)
     */
    {
        // Thetas init
        float[] Theta_1 = new float[2];         // The two solutions for Theta_1 above correspond to the shoulder being either "left" or "right"
        float[] Theta_5 = new float[4];         // The two solutions for Theta_5 above correspond to the wrist being "down" and "up".
        float[] Theta_6 = new float[4];
        float[] Theta_3 = new float[8];         // The two solutions for Theta_3 above correspond to the elbow being "down" and "up".
        float[] Theta_2 = new float[8];         // The two solutions for Theta_2 above correspond to the elbow being "down" and "up".
        float[] Theta_4 = new float[8];

        TCP = (T_Base == default(Matrix4x4)) ? TCP : T_Base.inverse * TCP;
        Matrix4x4 T = (T_Tool == default(Matrix4x4)) ? TCP : TCP * T_Tool.inverse;

        // Extraction of data from the End-point transformation matrix
        Vector3 P06 = new Vector3(T.m03, T.m13, T.m23); // Position of End-point
        Vector3 z5 = new Vector3(T.m02, T.m12, T.m22);  // Direction of z5 axis

        // Step 1. Theta_1
        float d6 = DH_Table[5, 2];
        Vector3 P05 = P06 - d6 * z5;

        float cosPhi = DH_Table[3, 2] / P05.magnitude;
        if (Mathf.Abs(cosPhi) > 1)
        {
            Debug.LogError("P05 is out of reachable workspace.");
            return null;
        }

        for (int i = 0; i < 2; i++)
        {
            // Calculate Theta_1
            if (i == 0)
            {
                Theta_1[i] = Mathf.Atan2(P05.y, P05.x) + Mathf.Acos(DH_Table[3, 2] / (Mathf.Sqrt(P05.y * P05.y + P05.x * P05.x))) + Mathf.PI / 2;
            }
            else
            {
                Theta_1[i] = Mathf.Atan2(P05.y, P05.x) - Mathf.Acos(DH_Table[3, 2] / (Mathf.Sqrt(P05.y * P05.y + P05.x * P05.x))) + Mathf.PI / 2;
            }

            // Step 2. Theta_5
            for (int ii = (i * 2); ii < (i * 2 + 2); ii++)
            {
                float P16_Z = P06.x * Mathf.Sin(Theta_1[i]) - P06.y * Mathf.Cos(Theta_1[i]);
                float argument = (P16_Z - DH_Table[3, 2]) / DH_Table[5, 2];
                // Calculate Theta_5
                if (ii % 2 == 0)
                {
                    // plus for even "ii"
                    Theta_5[ii] = Mathf.Acos(argument);
                }
                else
                {
                    // minus for odd "ii"
                    Theta_5[ii] = -Mathf.Acos(argument);
                }

                // Step 3. Theta_6
                Matrix4x4 T01 = CalculateTransformMatrix(Theta_1[i], DH_Table[0, 1], DH_Table[0, 2], DH_Table[0, 3]);
                Matrix4x4 T61 = (T01.inverse * T);      // Preprocess matrix
                T61 = T61.inverse;                      // Final form

                /*
                − sin(Theta_6) * sin(Theta_5) =  T61.m12  = Zy
                  cos(Theta_6) * sin(Theta_5) =  T61.m02  = Zx
                */

                // Calculate Theta_6
                Theta_6[ii] = Mathf.Atan2(-T61.m12 / Mathf.Sin(Theta_5[ii]), T61.m02 / Mathf.Sin(Theta_5[ii]));

                // Step 4. Theta_3
                for (int iii = (2 * ii); iii < (ii * 2 + 2); iii++)
                {
                    Matrix4x4 T45 = CalculateTransformMatrix(Theta_5[ii], DH_Table[4, 1], DH_Table[4, 2], DH_Table[4, 3]);
                    Matrix4x4 T56 = CalculateTransformMatrix(Theta_6[ii], DH_Table[5, 1], DH_Table[5, 2], DH_Table[5, 3]);
                    Matrix4x4 T14 = (T45 * T56);            // Preprocess matrix
                    T14 = T61.inverse * T14.inverse;        // Final form

                    // Extraction of data from transformation matrix
                    Vector3 P14 = new Vector3(T14.m03, T14.m13, T14.m23);
                    Vector3 z3 = new Vector3(T14.m01, T14.m11, T14.m21);
                    float d4 = DH_Table[3, 2];
                    Vector3 P13 = P14 - d4 * z3;            // Similiar as in Step 1.

                    float P13_magnitude = Mathf.Sqrt(P13.y * P13.y + P13.x * P13.x);    // Preprocess
                                                                                        
                    float c3 = ((P13_magnitude * P13_magnitude) - DH_Table[1, 1] * DH_Table[1, 1] - DH_Table[2, 1] * DH_Table[2, 1]) / (2 * DH_Table[1, 1] * DH_Table[2, 1]);
                    //c3 = Mathf.Clamp(c3, -1f, 1f);
                    // Calculate Theta_3
                    if (iii % 2 == 0)
                    {
                        // plus for even "iii"
                        Theta_3[iii] = Mathf.Acos(c3);
                    }
                    else
                    {
                        // minus for odd "iii"
                        Theta_3[iii] = -Mathf.Acos(c3);
                    }

                    // Step 5. Theta_2    
                    float delta = Mathf.Atan2(P13.y, P13.x);        // REMOVED MINUS in P13.x compared to the source article
                        float sinEpsilon = (DH_Table[2, 1] * Mathf.Sin(Theta_3[iii])) / P13_magnitude;
                        float epsilon = Mathf.Asin(sinEpsilon);
                        // Calculate Theta_2
                        Theta_2[iii] = (delta - epsilon); // REMOVED MINUS before bracked - compared to the source article

                    // Step 6. Theta_4
                    Matrix4x4 T12 = CalculateTransformMatrix(Theta_2[iii], DH_Table[1, 1], DH_Table[1, 2], DH_Table[1, 3]);
                    Matrix4x4 T23 = CalculateTransformMatrix(Theta_3[iii], DH_Table[2, 1], DH_Table[2, 2], DH_Table[2, 3]);
                    Matrix4x4 Temp_M = T12 * T23;       // Preprocess matrix
                    Temp_M = Temp_M.inverse;            // inverse of preprocessed product matrix
                    Matrix4x4 T34 = Temp_M * T14;       // Final form

                    // Calculate Theta_4
                    Theta_4[iii] = Mathf.Atan2(T34.m10, T34.m00);
                }
            }
        }
        // Fill up solution matrix
        float[,] jointAngles =
            {
            { Theta_1[0], Theta_2[0], Theta_3[0],Theta_4[0], Theta_5[0], Theta_6[0]},
            { Theta_1[0], Theta_2[1], Theta_3[1],Theta_4[1], Theta_5[0], Theta_6[0]},
            { Theta_1[0], Theta_2[2], Theta_3[2],Theta_4[2], Theta_5[1], Theta_6[1]},
            { Theta_1[0], Theta_2[3], Theta_3[3],Theta_4[3], Theta_5[1], Theta_6[1]},
            { Theta_1[1], Theta_2[4], Theta_3[4],Theta_4[4], Theta_5[2], Theta_6[2]},
            { Theta_1[1], Theta_2[5], Theta_3[5],Theta_4[5], Theta_5[2], Theta_6[2]},
            { Theta_1[1], Theta_2[6], Theta_3[6],Theta_4[6], Theta_5[3], Theta_6[3]},
            { Theta_1[1], Theta_2[7], Theta_3[7],Theta_4[7], Theta_5[3], Theta_6[3]}
        };
            // Convert Rads to Deg
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 8; j++)
                {
                    jointAngles[j, i] *= Mathf.Rad2Deg;
                }
            }
            return jointAngles;
    }
    public static float[] CalculateInverseKinematics_Validation(Matrix4x4 T, float[] Original_Angles)
    /*
     Solving Inverse Kinematics according to Ryan Keating and Noah J. Cowan (UR5 Inverse Kinematics, 2016)
        pdf URL: https://tianyusong.com/wp-content/uploads/2017/12/ur5_inverse_kinematics.pdf
        P - Position (Vector3)
        T - Transform matrix (Matrix4x4)
     */
    {
        float[] jointAngles = new float[6];

        // Extraction of data from the End-point transformation matrix
        Vector3 P06 = new Vector3(T.m03, T.m13, T.m23); // Position of End-point
        Vector3 z5 = new Vector3(T.m02, T.m12, T.m22);  // Direction of z5 axis

        // Step 1. Theta_1
        float d6 = DH_Table[5, 2];
        Vector3 P05 = P06 - d6 * z5;

        float cosPhi = DH_Table[3, 2] / P05.magnitude;
        if (Mathf.Abs(cosPhi) > 1)
        {
            Debug.LogError("P05 is out of reachable workspace.");
            return null;
        }

        // Calculate Theta_1
        jointAngles[0] = Mathf.Atan2(P05.y, P05.x) + Mathf.Acos(DH_Table[3, 2] / (Mathf.Sqrt(P05.y * P05.y + P05.x * P05.x))) + Mathf.PI / 2;

        /* Verification
                There are two posible solutions Theta_1 - Shoulder left/ Shoulder right
        */

        float original = Original_Angles[0];
        if (!CompareAngles(original, jointAngles[0] * Mathf.Rad2Deg))
        {
            jointAngles[0] = Mathf.Atan2(P05.y, P05.x) - Mathf.Acos(DH_Table[3, 2] / (Mathf.Sqrt(P05.y * P05.y + P05.x * P05.x))) + Mathf.PI / 2;
        }
        if (!CompareAngles(original, jointAngles[0] * Mathf.Rad2Deg))
        {
            Debug.LogError("Non valid");

        }

        // Step 2. Theta_5
        float theta1 = jointAngles[0];
        float P16_Z = P06.x * Mathf.Sin(theta1) - P06.y * Mathf.Cos(theta1);
        float argument = (P16_Z - DH_Table[3, 2]) / DH_Table[5, 2];
        // Calculate Theta_5
        jointAngles[4] = Mathf.Acos(argument);

        /* Verification
                There are two posivble solutions Theta_5 - Wrist down/ Wrist up
        */
        original = Original_Angles[4];              // Get desired angle for comparison
        if (!CompareAngles(original, jointAngles[4] * Mathf.Rad2Deg))
        {
            jointAngles[4] = - Mathf.Acos(argument);
        }
        if (!CompareAngles(original, jointAngles[4] * Mathf.Rad2Deg))
        {
            Debug.LogError("Non valid");

        }
        float theta5 = jointAngles[4];

        // Step 3. Theta_6
        Matrix4x4 T01 = CalculateTransformMatrix(jointAngles[0], DH_Table[0, 1], DH_Table[0, 2], DH_Table[0, 3]);   
        Matrix4x4 T61 = (T01.inverse * T);      // Preprocess matrix
        T61 = T61.inverse;                      // Final form

        /*
        − sin(Theta_6) * sin(Theta_5) =  T61.m12  = Zy
          cos(Theta_6) * sin(Theta_5) =  T61.m02  = Zx
        */

        // Calculate Theta_6
        jointAngles[5] = Mathf.Atan2(-T61.m12 / Mathf.Sin(theta5), T61.m02 / Mathf.Sin(theta5));

        // Verification
        original = Original_Angles[5];
        if (!CompareAngles(original, jointAngles[5] * Mathf.Rad2Deg))
        {
            Debug.LogError("Non valid");
        }

        // Step 4. Theta_3
        Matrix4x4 T45 = CalculateTransformMatrix(jointAngles[4], DH_Table[4, 1], DH_Table[4, 2], DH_Table[4, 3]);
        Matrix4x4 T56 = CalculateTransformMatrix(jointAngles[5], DH_Table[5, 1], DH_Table[5, 2], DH_Table[5, 3]);
        Matrix4x4 T14 = (T45 * T56);            // Preprocess matrix
        T14 = T61.inverse * T14.inverse;        // Final form

        // Extraction of data from transformation matrix
        Vector3 P14 = new Vector3(T14.m03, T14.m13, T14.m23);
        Vector3 z3 = new Vector3(T14.m01, T14.m11, T14.m21);
        float d4 = DH_Table[3, 2];
        Vector3 P13 = P14 - d4 * z3;            // Similiar as in Step 1.

        float P13_magnitude = Mathf.Sqrt(P13.y * P13.y + P13.x * P13.x);    // Preprocess
        // Calculate Theta_3
        float c3 = ((P13_magnitude * P13_magnitude) - DH_Table[1, 1] * DH_Table[1, 1] - DH_Table[2, 1] * DH_Table[2, 1]) / (2 * DH_Table[1, 1] * DH_Table[2, 1]);
        jointAngles[2] = Mathf.Acos(c3);

        // Verification
        original = Original_Angles[2];
        if (!CompareAngles(original, jointAngles[2] * Mathf.Rad2Deg))
        {
            jointAngles[2] = - Mathf.Acos(c3);
        }
        if (!CompareAngles(original, jointAngles[2] * Mathf.Rad2Deg))
        {
            Debug.LogError("Non valid");
        }

        // Step 5. Theta_2
        float delta = Mathf.Atan2(P13.y, P13.x);        // REMOVED MINUS in P13.x compared to the source article
        float sinEpsilon = (DH_Table[2, 1] * Mathf.Sin(jointAngles[2])) / P13_magnitude;
        // sinEpsilon = Mathf.Clamp(sinEpsilon, -1f, 1f); // Numerical error handle
        float epsilon = Mathf.Asin(sinEpsilon);

        // Calculate Theta_2
        jointAngles[1] = (delta - epsilon); // REMOVED MINUS before bracked - compared to the source article

        // Verification
        original = Original_Angles[1];
        if (!CompareAngles(original, jointAngles[1] * Mathf.Rad2Deg))
        {
            float angle = jointAngles[1] * Mathf.Rad2Deg;
            Debug.LogError("Non valid");
        }

        // Step 6. Theta_4
        Matrix4x4 T12 = CalculateTransformMatrix(jointAngles[1], DH_Table[1, 1], DH_Table[1, 2], DH_Table[1, 3]);
        Matrix4x4 T23 = CalculateTransformMatrix(jointAngles[2] ,DH_Table[2, 1], DH_Table[2, 2], DH_Table[2, 3]);
        Matrix4x4 Temp_M = T12 * T23;       // Preprocess matrix
        Temp_M = Temp_M.inverse;            // inverse of preprocessed product matrix
        Matrix4x4 T34 = Temp_M * T14;       // Final form

        // Calculate Theta_4
        jointAngles[3] = Mathf.Atan2(T34.m10, T34.m00);

        // Verification
        original = Original_Angles[3];
        if (!CompareAngles(original, jointAngles[3] * Mathf.Rad2Deg))
        {
            float angle = jointAngles[3] * Mathf.Rad2Deg;
            Debug.LogError("Non valid");
        }
        // Convert Rads to Deg
        for (int i = 0; i < 6; i++) 
        {
            jointAngles[i] = jointAngles[i] * Mathf.Rad2Deg;
        }
        return jointAngles;
    }

    public static float[] CalculateInverseKinematics_CLOSEST(Matrix4x4 T, float[] Current_Angles)
    /*
     Solving Inverse Kinematics according to Ryan Keating and Noah J. Cowan (UR5 Inverse Kinematics, 2016)

        Algortithm chooses splution that is closest angle to input angles and return only one target solution.
        Faster than computing all solutions.

        pdf URL: https://tianyusong.com/wp-content/uploads/2017/12/ur5_inverse_kinematics.pdf
        P - Position (Vector3)
        T - Transform matrix (Matrix4x4)
     */
    {
        float[] jointAngles = new float[6];

        // Extraction of data from the End-point transformation matrix
        Vector3 P06 = new Vector3(T.m03, T.m13, T.m23); // Position of End-point
        Vector3 z5 = new Vector3(T.m02, T.m12, T.m22);  // Direction of z5 axis

        // Step 1. Theta_1
        float d6 = DH_Table[5, 2];
        Vector3 P05 = P06 - d6 * z5;

        float cosPhi = DH_Table[3, 2] / P05.magnitude;
        if (Mathf.Abs(cosPhi) > 1)
        {
            Debug.LogError("P05 is out of reachable workspace.");
            return null;
        }

        // Calculate Theta_1
            // There are two posible solutions Theta_1 -Shoulder left / Shoulder right
        float[] Theta_1 = new float[2];
        Theta_1[0] = Mathf.Atan2(P05.y, P05.x) + Mathf.Acos(DH_Table[3, 2] / (Mathf.Sqrt(P05.y * P05.y + P05.x * P05.x))) + Mathf.PI / 2;
        Theta_1[1] = Mathf.Atan2(P05.y, P05.x) - Mathf.Acos(DH_Table[3, 2] / (Mathf.Sqrt(P05.y * P05.y + P05.x * P05.x))) + Mathf.PI / 2;
        // choose between solutions
        jointAngles[0] = Get_Suitable_Angle(Current_Angles[0], Theta_1);

        // Step 2. Theta_5
            // There are two posivble solutions Theta_5 - Wrist down/ Wrist up

        float[] Theta_5 = new float[2];
        float P16_Z = P06.x * Mathf.Sin(jointAngles[0]) - P06.y * Mathf.Cos(jointAngles[0]);
        float argument = (P16_Z - DH_Table[3, 2]) / DH_Table[5, 2];
        argument = Mathf.Clamp(argument, -1f, 1f);

        // Calculate Theta_5
        Theta_5[0] = Mathf.Acos(argument);
        Theta_5[1] = -Mathf.Acos(argument);
        jointAngles[4] = Get_Suitable_Angle(Current_Angles[4], Theta_5);
        if (jointAngles[4] == 0)
        {
            jointAngles[4] = float.Epsilon;
        }

        // Step 3. Theta_6
        Matrix4x4 T01 = CalculateTransformMatrix(jointAngles[0], DH_Table[0, 1], DH_Table[0, 2], DH_Table[0, 3]);
        Matrix4x4 T61 = (T01.inverse * T);      // Preprocess matrix
        T61 = T61.inverse;                      // Final form

        /*
        − sin(Theta_6) * sin(Theta_5) =  T61.m12  = Zy
          cos(Theta_6) * sin(Theta_5) =  T61.m02  = Zx
        */

        // Calculate Theta_6
        jointAngles[5] = Mathf.Atan2(-T61.m12 / Mathf.Sin(jointAngles[4]), T61.m02 / Mathf.Sin(jointAngles[4]));
        
        // Step 4. Theta_3
        float[] Theta_3 = new float[2];
        Matrix4x4 T45 = CalculateTransformMatrix(jointAngles[4], DH_Table[4, 1], DH_Table[4, 2], DH_Table[4, 3]);
        Matrix4x4 T56 = CalculateTransformMatrix(jointAngles[5], DH_Table[5, 1], DH_Table[5, 2], DH_Table[5, 3]);
        Matrix4x4 T14 = (T45 * T56);            // Preprocess matrix
        T14 = T61.inverse * T14.inverse;        // Final form

        // Extraction of data from transformation matrix
        Vector3 P14 = new Vector3(T14.m03, T14.m13, T14.m23);
        Vector3 z3 = new Vector3(T14.m01, T14.m11, T14.m21);
        float d4 = DH_Table[3, 2];
        Vector3 P13 = P14 - d4 * z3;            // Similiar as in Step 1.

        float P13_magnitude = Mathf.Sqrt(P13.y * P13.y + P13.x * P13.x);    // Preprocess
        // Calculate Theta_3
        float c3 = ((P13_magnitude * P13_magnitude) - DH_Table[1, 1] * DH_Table[1, 1] - DH_Table[2, 1] * DH_Table[2, 1]) / (2 * DH_Table[1, 1] * DH_Table[2, 1]);
        c3 = Mathf.Clamp(c3, -1f, 1f);

        Theta_3[0] = Mathf.Acos(c3);
        Theta_3[1] = -  Mathf.Acos(c3);
        jointAngles[2] = Get_Suitable_Angle(Current_Angles[2], Theta_3);
        if (jointAngles[2] == 0)
        {
            jointAngles[2] = float.Epsilon;
        }

        // Step 5. Theta_2
        float delta = Mathf.Atan2(P13.y, P13.x);        // REMOVED MINUS in P13.x compared to the source article
        float sinEpsilon = (DH_Table[2, 1] * Mathf.Sin(jointAngles[2])) / P13_magnitude;
        if (sinEpsilon == 0)
        {
            sinEpsilon = float.Epsilon;
        }

        // sinEpsilon = Mathf.Clamp(sinEpsilon, -1f, 1f); // Numerical error handle
        float epsilon = Mathf.Asin(sinEpsilon);

        // Calculate Theta_2
        jointAngles[1] = (delta - epsilon); // REMOVED MINUS before bracked - compared to the source article

        // Step 6. Theta_4
        Matrix4x4 T12 = CalculateTransformMatrix(jointAngles[1], DH_Table[1, 1], DH_Table[1, 2], DH_Table[1, 3]);
        Matrix4x4 T23 = CalculateTransformMatrix(jointAngles[2], DH_Table[2, 1], DH_Table[2, 2], DH_Table[2, 3]);
        Matrix4x4 Temp_M = T12 * T23;       // Preprocess matrix
        Temp_M = Temp_M.inverse;            // inverse of preprocessed product matrix
        Matrix4x4 T34 = Temp_M * T14;       // Final form

        // Calculate Theta_4
        jointAngles[3] = Mathf.Atan2(T34.m10, T34.m00);

        // Convert Rads to Deg
        for (int i = 0; i < 6; i++)
        {
            jointAngles[i] = jointAngles[i] * Mathf.Rad2Deg;
        }
        return jointAngles;
    }

    // Used for verfication 
    private static bool CompareAngles(float original, float calculated, float tolerance = 0.1f)
    {
        float diff = Mathf.Abs(Mathf.DeltaAngle(original , calculated));
        if (diff > tolerance)
        {
            return false;
        }
        else 
        { 
            return true;
        }
    }

    private static float Get_Suitable_Angle(float original, float[] calculated)
    {
        int index_better = 0;
        float Compare_Value = float.MaxValue;
        for(int i = 0; i < calculated.Length; i++) 
        {
            //float diff = Mathf.Abs(Mathf.DeltaAngle(original, calculated[i] * Mathf.Rad2Deg));
            float diff = Mathf.Abs(original - calculated[i] * Mathf.Rad2Deg);
            if (diff < Compare_Value)
            {
                index_better = i;
                Compare_Value = diff;
            }
        }
        return calculated[index_better];
    }
}

