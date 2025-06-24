using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
______________________________________________________________________________________________
This script is attached to Main camera and enables free movement when right click (on hold)
    Space - UP
    Ctrl  - DOWN
    Mouse scroll - speed of camera (can go negative values)
    T - Start rotation  
    R - stop rotation
_______________________________________________________________________________________________
*/

public class CameraMovement : MonoBehaviour
{
    public float Sensitivity;       // Mouse rotation sensitivity
    public float Movement_Speed;    // Speed coeficient
    public float Scrool_Step;       // Coeficient of scrolling - changing speed coeficient

    // Rotation around center
    public float Rotation_Speed;    // Rotation velocity
    private bool Is_Rotating;       // internal
    private float Radius;           
    private float Angle;
    private float Height;


    void Start()
    {
        // Initial coeficients
        Scrool_Step = 0.05f;
        Movement_Speed = 3f;
        Rotation_Speed = 1.0f;

        Is_Rotating = false;
    }


    // Update is called once per frame
    void Update()
    {
        // Changing speed coeficient by scrolling on mouse
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll > 0)
        {
            Movement_Speed += Scrool_Step;
        }
        else if (scroll < 0)
        {
            Movement_Speed -= Scrool_Step;
        }

        // Camera movement
        if (Input.GetMouseButton(1)) // When right-click
        {
            Cursor.visible = false;
            Cursor.lockState = CursorLockMode.Locked;
            Movement();
            Rotation();
        }
        else
        {
            Cursor.visible = true;
            Cursor.lockState = CursorLockMode.None;
        }

        // Roating effect handle
        if (Input.GetKeyDown(KeyCode.T))
        {
            StartRotation();
        }
        if (Input.GetKeyDown(KeyCode.R))
        {
            StopRotation();
        }
        if (Is_Rotating)
        {
            Rotate_Camera_To_Center();
        }

    }

    // Mouse rotation
    public void Rotation()
    {
        Vector3 OrientationInput = new Vector3(-Input.GetAxis("Mouse Y"), Input.GetAxis("Mouse X"), 0);
        transform.Rotate(OrientationInput * Sensitivity);
        Vector3 eulerRotation = transform.rotation.eulerAngles;
        transform.rotation = Quaternion.Euler(eulerRotation.x, eulerRotation.y, 0);
    }

    // Camera movement by mouse
    public void Movement()
    {
        float MoveUp;
        Vector3 DirectionInput = new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical"));
        transform.Translate(DirectionInput * Movement_Speed * Time.deltaTime);
        // Moving up
        if (Input.GetKey("space"))
        {
            MoveUp = 1;
        }
        else if (Input.GetKey("left ctrl"))
        {
            MoveUp = -1;
        }
        else
        {
            MoveUp = 0;
        }
        transform.Translate(0, MoveUp * Time.deltaTime, 0);
    }

    // Rotating around center effect
    void StartRotation()
    {
        Vector3 position = transform.position;
        Radius = new Vector2(position.x, position.z).magnitude; // Save actual radius
        Angle = Mathf.Atan2(position.z, position.x); // Initial angle
        Height = position.y; // Save actual height
        Is_Rotating = true;
    }

    void StopRotation()
    {
        Is_Rotating = false;
    }

    void Rotate_Camera_To_Center()
    {
        Angle += Movement_Speed * Time.deltaTime; // Angle shift
        float newX = Mathf.Cos(Angle) * Radius;
        float newZ = Mathf.Sin(Angle) * Radius;

        transform.position = new Vector3(newX, Height, newZ);
        transform.LookAt(new Vector3(0, Height, 0)); // Turns transofrm to center
    }





}