using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;

namespace DrawTool
{
    public class DrawPolyline : Tool
    {
        internal Polyline polyline;
        public DrawPolyline() : base() { }
        
        
        public override void Execute()
        {
            if (!drawing)
            {
                polyline = new Polyline();
                pts = new List<Vector3>();
                queue.Add(polyline);
            }
            drawing = true;
            Vector3 xp = Utility.MouseToWorld(workPlane);

            foreach (var item in pts)
            {
                if (Utility.IsClose(item,xp))
                {
                    xp = item;
                    Debug.Log("Close");
                }
            }

            markers.Add(base.CreateMarker(xp));
            pts.Add(xp);
            polyline.vertices = pts.ToArray();
                     
        }
        public virtual void PreFinish()
        {
            if (pts.Count < 2)
            {
                return;
            }
            drawing = false;
            DestryAllMarkers();

            Vector3[] pop = new Vector3[pts.Count];
            for (int i = 0; i < pts.Count; i++)
            {
                pop[i] = pts[i];
            }

            toolArgs.vector3s = pop;         
        }    
        public override void Finish()
        {
            PreFinish();
            base.Finish();
        }
        public override void OnMouseDown(int button)
        {
            if (button == 0)
                Execute();
            else if (button == 1)
                Finish();
        }
        public override void OnMouseMove()
        {
            if (pts.Count > 0 && drawing)
            {
                MouseHover = Utility.MouseToWorld(workPlane);
                dynamic = new Polyline();
                List<Vector3> pip = new List<Vector3>();

                pip.Add(pts[pts.Count - 1]);
                pip.Add(MouseHover);
                dynamic.vertices = pip.ToArray();
             
            }


        }
       
        public override void OnRenderObject()
        {
            foreach (Polyline pl in queue)
            {
       
                if (pl.vertices.Length > 1)
                {
                    GLRender.Polyline(pl.vertices, false, null, Color.black);
                }

            }
            //动态线
            if (dynamic != null && drawing)
            {
           
                SGGeometry.GLRender.Polyline(dynamic.vertices, false, null, Color.yellow);

            }

        }

       

    }
}
