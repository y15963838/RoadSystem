using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;
using DrawTool;
using UnityEngine.UI;

public class Executor : MonoBehaviour
{
    private Plane workPlane;
    private Tool tool;
    private RoadSystem roadSystem;
    public Button bt_polyline, bt_polygon, bt_road;

    public enum Stat { line,gon,road};
    public Stat stat;

    private void Awake()
    {
    }
    void Start()
    {
        workPlane = new Plane(Vector3.up, Vector3.zero);
        roadSystem = new RoadSystem();

        tool = new DrawRoad(roadSystem);
        stat = Stat.road;
        tool.workPlane = workPlane;
        Tool.onToolComplete += GetInfo;
       
        bt_polyline.onClick.AddListener(drawPolyline);
        bt_polygon.onClick.AddListener(drawPolygon);
        bt_road.onClick.AddListener(drawRoad);
    }
    
    void drawPolyline()
    { 
        tool = new DrawPolyline();
        stat = Stat.line;
    }
    void drawPolygon()
    {     
        tool = new DrawPolygon();
        stat = Stat.gon;
    }
    void drawRoad()
    {      
        tool = new DrawRoad(roadSystem);
        stat = Stat.road;
    }

    //从画线和画几何体中提取信息
    public void GetInfo(Tool sender, ToolArgs args)      
    {
        Vector3[] pts = args.vector3s;
        Polygon polygon = args.polygon;
       

        if (stat == Stat.line)
        {
            Debug.Log(pts);
        }
        else if (stat == Stat.gon)
        {
            Debug.Log(polygon);
        }
    }
   
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            tool.OnMouseDown(0);
        }
        else if (Input.GetMouseButtonDown(1))
        {
            tool.OnMouseDown(1);
        }
        else if (Input.GetKeyDown(KeyCode.Space))
        {
            Debug.Log("hh:"+roadSystem.GetRoad(1).nodes[0].position.x);
        }
        else
        {
            tool.OnMouseMove();
        }
        
    }

   
    private void OnRenderObject()
    {
       tool.OnRenderObject();
      
        if (stat == Stat.road)
        {
            roadSystem.OnRenderObject();
        }
    }

}
