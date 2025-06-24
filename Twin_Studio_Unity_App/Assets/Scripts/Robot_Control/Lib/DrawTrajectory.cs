using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
    The class is purely concerned with drawing/deleting a line as a render based on the specified points
*/
public class DrawTrajectory : MonoBehaviour
{
    private LineRenderer Line_Renderer;
    private Material Line_Material;

    private void Awake()
    {
        if (Line_Renderer == null) 
        { 
        Line_Renderer = gameObject.AddComponent<LineRenderer>();
        };

        Line_Renderer = gameObject.GetComponent<LineRenderer>();

        Line_Renderer.widthMultiplier = 0.0025f;
        Line_Material = new Material(Shader.Find("Unlit/Color"));
        Line_Material.color = Color.white;
        Line_Renderer.material = Line_Material;
    }

    public void Draw_Line(List<Vector3> points)
    {
        if (Line_Renderer == null)
        {
            Awake();
        }

        if (points == null || points.Count == 0)
        {
            Debug.LogWarning("List is empty");
            return;
        }

        Line_Renderer.positionCount = points.Count;
        Line_Renderer.SetPositions(points.ToArray());
    }

    public void Erase_Line()
    {
        Line_Renderer.positionCount = 0;
    }
}
