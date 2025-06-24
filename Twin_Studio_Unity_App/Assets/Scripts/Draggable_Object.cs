using System.Collections;
using System.Collections.Generic;
//using System.Numerics;
using UnityEngine;

public class Draggable_Object : MonoBehaviour
{



    UnityEngine.Vector3 Last_Mouse_Position;
    public float Drag_Speed = 1f;
  

    private void OnMouseDown()
    {
    Last_Mouse_Position = Input.mousePosition - Camera.main.WorldToScreenPoint(transform.position);
    }

     private void OnMouseDrag()
    {
        UnityEngine.Vector3 Delta = Input.mousePosition - Last_Mouse_Position;
        UnityEngine.Vector3 Position = transform.position;
       // Position.y = Delta.y * Drag_Speed;
        transform.position = Camera.main.ScreenToWorldPoint(Delta);

    }
}
