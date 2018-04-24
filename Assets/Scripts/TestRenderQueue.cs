using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using SGGeometry;

public class TestRenderQueue : MonoBehaviour {
    List<Polyline> queue;
    List<Vector3> pts;
    List<GameObject> markers;
    Plane plane;
    Polyline ply;
    Text debugText;
    bool drawing = false;
	// Use this for initialization
	void Start () {
        queue = new List<Polyline>();
        pts = new List<Vector3>();
        markers = new List<GameObject>();
        plane = new Plane(Vector3.up, Vector3.zero);
        ply = new Polyline();
        queue.Add(ply);
        debugText = GameObject.Find("DText").GetComponent<Text>();
    }
	
	// Update is called once per frame
	void Update () {
        if (Input.GetMouseButtonDown(0))
        {
            OnDrawPoints();

        }
        else if (Input.GetMouseButtonDown(1))
        {
            OnFinishPoints();

        }
        else if (Input.GetMouseButtonDown(2))
        {
            OnErraseAll();
        }

    }
    private void OnFinishPoints()
    {
        drawing = false;
        DestryAllMarkers();
    }
    private void OnErraseAll()
    {
        drawing = false;
        queue.Clear();
        DestryAllMarkers();
    }
    private void OnDrawPoints()
    {
        if (!drawing)
        {
            ply = new Polyline();
            queue.Add(ply);
            pts = new List<Vector3>();
        }

            
        drawing = true;
        float d;
        Ray r = Camera.main.ScreenPointToRay(Input.mousePosition);
        plane.Raycast(r, out d);
        Vector3 xp = r.GetPoint(d);
        markers.Add(createMarker(xp));
        pts.Add(xp);
        ply.vertices = pts.ToArray();
        Debug.Log(string.Format("mouseDown ply.verticesCount={0}",ply.vertices.Length));
    }


    private void OnRenderObject()
    {
        string txt = "";
        foreach (Polyline pl in queue)
        {
            txt += "pl.vcount:" + pl.vertices.Length;
            if(pl.vertices.Length>1)
            {
                //pl.OnRenderObject();
                SGGeometry.GLRender.Polyline(pl.vertices, false, null, Color.black);
            }
                //pl.OnRenderObject();
        }
        debugText.text = txt;
    }
    GameObject createMarker(Vector3 v)
    {
        GameObject o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        o.transform.position = v;
        o.transform.localScale = new Vector3(0.1f,0.1f,0.1f);
        return o;
    }
    void DestryAllMarkers()
    {
        foreach (GameObject o in markers)
        {
            Destroy(o);
        }
        markers.Clear();
    }
}
