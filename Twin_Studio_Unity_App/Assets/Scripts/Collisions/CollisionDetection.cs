using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetection : MonoBehaviour
{
    /*
     Script checks if the two requested objects are in collision.
     One of the objects has this script assigned to it and the other is stored in the Target_Object variable 
    */
    public Collider Target_Object;
    public bool Contanct_With_Target { get; private set; }
    public List<Collision> Collision_Details { get; private set; }
    

    // Start is called before the first frame update
    void Start()
    {
        Collision_Details = new List<Collision>();
    }

    void OnCollisionEnter(Collision Col)
    {
        if (Col.collider == Target_Object)
        {
            Collision_Details.Add(Col);
        }       
    }

    void OnCollisionExit(Collision Col)
    {
        Collision Tracked_Collision = Collision_Details.Find(c => c.collider == Col.collider);

        if (Tracked_Collision != null)
        {
            Collision_Details.Remove(Tracked_Collision);
        }
    }
}
