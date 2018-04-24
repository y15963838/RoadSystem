using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using SGGeometry;

namespace DrawTool
{
    public class Tool
    {
        public delegate void OnToolCompleteDelegate(Tool tool, ToolArgs arg);
        public static OnToolCompleteDelegate onToolComplete;
               
        public Plane workPlane;
        public ToolArgs toolArgs;

        internal Vector3 MouseHover;     //鼠标悬停位置         
        internal Polyline dynamic;    //绘画过程中产生的动态线  
        internal bool IsNext;         //绘画过程中是否遇到之前的点
        internal List<GameObject> markers;
        internal List<Polyline> queue;
        internal List<Vector3> pts;   //记录按鼠标点下的位置
        internal bool drawing;
       
        public Tool()
        {
            toolArgs = new ToolArgs();
            workPlane = new Plane(Vector3.up, Vector3.zero);
            markers = new List<GameObject>();
            pts = new List<Vector3>();
            queue = new List<Polyline>();
            drawing = false;
        }
        public virtual void Execute() { }
        public virtual void OnRenderObject() { }
        public virtual void Finish()
        {
            onToolComplete(this, toolArgs);
        }
        public virtual GameObject CreateMarker(Vector3 v)
        {
            GameObject o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            o.transform.position = v;
            o.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            return o;
        }
        public virtual void DestryAllMarkers()
        {
            foreach (GameObject o in markers)
            {
                GameObject.Destroy(o);
            }
            markers.Clear();
        }
        public virtual void OnMouseDown(int button) { }
        public virtual void OnMouseUp(int button) { }
        public virtual void OnMouseDrag(int button) { }
        public virtual void OnMouseMove() { }
     
    }

    public class ToolArgs
    {
        public Vector3[] vector3s;
        public Polygon polygon = new Polygon();
        
    }

}
