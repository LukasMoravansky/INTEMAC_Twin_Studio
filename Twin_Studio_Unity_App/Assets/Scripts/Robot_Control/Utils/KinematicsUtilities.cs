using UnityEngine;

public class KinematicsUtilities
{
    /*
     The class is used for operations used in the calculation of robot kinematics
     */

    public static Matrix4x4 Quaternion_To_Matrix(Quaternion Q, Vector3 Translation)
    {
        float w = Q.w;
        float x = Q.x;
        float y = Q.y;
        float z = Q.z;

        // Get Rotation matrix
        float[,] Rotation = new float[3, 3]
        {
        { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
        { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
        { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) }
        };

        // Fill up transfrom matrix
        Matrix4x4 Matrix = new Matrix4x4();
        Matrix.SetRow(0, new Vector4(Rotation[0, 0], Rotation[0, 1], Rotation[0, 2], Translation.x));
        Matrix.SetRow(1, new Vector4(Rotation[1, 0], Rotation[1, 1], Rotation[1, 2], Translation.y));
        Matrix.SetRow(2, new Vector4(Rotation[2, 0], Rotation[2, 1], Rotation[2, 2], Translation.z));

        Matrix.SetRow(3, new Vector4(0, 0, 0, 1));

        return Matrix;
    }

    // Function used for conversions between right-hand and left-hand coordinate systems
    public static Matrix4x4 Convert_Coordinates(Matrix4x4 Input_Matrix, bool From_Unity)
    {
        /*
            bool From_Unity - TRUE means Matrix coordinates are taken FROM Unity enviroment
                            - FALSE means Matrix coordinates are standard and are converted TO the Unity environment
        */

        Matrix4x4 Modified_Matrix = new Matrix4x4();

        // Changed coordinates matrix
        Matrix4x4 M = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, 0, -1, 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(0, 0, 0, 1)
        );
        if (From_Unity)
        {
            Modified_Matrix = M.inverse * Input_Matrix * M;
        }
        else
        {
            Modified_Matrix = M * Input_Matrix * M.inverse;
        }
        return Modified_Matrix;
    }

    // Converts Unity Transform to Transform Matrix
    public static Matrix4x4 Get_Transform_Matrix(Transform T)
    {
        Vector3 T_position = T.position;
        Quaternion T_Quaternion = T.rotation;
        Matrix4x4 T_Matrix = Quaternion_To_Matrix(T_Quaternion, T_position);

        return T_Matrix;
    }

}
