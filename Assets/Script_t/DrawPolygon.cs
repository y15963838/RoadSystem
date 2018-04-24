using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;

namespace DrawTool
{
    public class DrawPolygon : DrawPolyline
    {
        internal Polygon polygon;
        public DrawPolygon() : base() { }

        public override void Execute()
        {         
            base.Execute();
        }
        public override void PreFinish()
        {
            base.PreFinish();
            toolArgs.polygon = new Polygon(toolArgs.vector3s);
        }
        public override void Finish()
        {
             base.Finish();
        }

      
        public override void OnRenderObject()
        {
            foreach (Polyline pl in queue)
            {
                if (pl.vertices.Length > 1)
                {
                    GLRender.Polyline(pl.vertices, true, null, Color.black);
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

