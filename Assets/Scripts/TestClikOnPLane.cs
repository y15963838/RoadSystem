using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestClikOnPLane : MonoBehaviour
{

    Plane plane;
    GameObject rf;

	

	void Start ()
    {
        plane = new Plane(Vector3.up, new Vector3());
        rf = GameObject.CreatePrimitive(PrimitiveType.Cube);
	}
	


	
	void Update ()
    {
        
        OnMouseDown();
	}
    private void OnMouseDown()
    {
        Vector3 mousePos = Input.mousePosition;
        Ray ray = Camera.main.ScreenPointToRay(mousePos);
        float d;
        plane.Raycast(ray, out d);
        Vector3 xp = ray.GetPoint(d);

        rf.transform.position = xp;
    }
}
