using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace SGGeometry
{
    //Base classes for geometries
    
    public class GLRender
    {
        public static Material lineMat;
        public static Material confirmMat(Material m)
        {
            if (m == null)
            {
                if (GLRender.lineMat == null)
                {
                    GLRender.lineMat = GetLineMaterial();
                }
                m = GLRender.lineMat;
            }
            return m;
        }
        public static Material GetLineMaterial()
        {
            if (lineMat != null)
            {
                return lineMat;
            }

            Material lineMaterial;
            // Unity has a built-in shader that is useful for drawing
            // simple colored things.
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            lineMaterial = new Material(shader);
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            // Turn on alpha blending
            lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            // Turn backface culling off
            lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            // Turn off depth writes
            lineMaterial.SetInt("_ZWrite", 0);
            lineMat = lineMaterial;
            return lineMaterial;
        }

        public static void Polyline(Vector3[] pts, bool closed, Material mat, Color color,Transform transform=null)
        {
            mat = confirmMat(mat);
            mat.SetPass(0);

            GL.PushMatrix();
            if (transform != null)
            {
                GL.MultMatrix(transform.localToWorldMatrix);
            }
            GL.Begin(GL.LINE_STRIP);
            GL.Color(color);
            foreach (Vector3 v in pts)
            {
                GL.Vertex(v);
            }
            if (closed)
            {
                GL.Vertex(pts[0]);
            }
            GL.End();
            GL.PopMatrix();
            //Debug.Log("on render object");
        }
        public static void GridPlane(Vector3 position, Vector3 normal,Vector3 vectU,Vector3 vectV, float gridSize = 0.1f, int gridCount = 10)
        {
            //make points

            Vector3[] pu = new Vector3[gridCount];
            Vector3[] pv = new Vector3[gridCount];
            float w = gridCount * gridCount;
            float max = w / 2;
            float min = -max;

            Vector3 left = position - vectU * max;
            Vector3 bot = position - vectV * max;

            for (int i =0; i < gridCount; i++)
            {
                pu[i] = left + (vectU * gridSize * i);
                pv[i] = bot + (vectV * gridSize * i);
            }
                
            Material mat = GetLineMaterial();
            mat.SetPass(0);
            GL.PushMatrix();
          
            GL.Begin(GL.LINE_STRIP);
            GL.Color(Color.grey);
            foreach (Vector3 v in pu)
            {
                GL.Vertex(v);
                GL.Vertex(v+vectV*w);
            }
            foreach (Vector3 v in pv)
            {
                GL.Vertex(v);
                GL.Vertex(v + vectU*w);
            }
            GL.Color(Color.red);
            GL.Vertex(position);
            GL.Vertex(position + vectU * max);
            GL.Color(Color.green);
            GL.Vertex(position);
            GL.Vertex(position + vectV * max);
            GL.Color(Color.blue);
            GL.Vertex(position);
            GL.Vertex(position + normal * max/2);


            GL.End();
            GL.PopMatrix();
        }
    }
    public class BoundingBox
    {
        public Vector3 position;
        public Vector3[] vects;
        public Vector3 size;
        public Vector3[] vertices;
        public BoundingBox()
        {

        }
        public static BoundingBox CreateFromPoints(Vector3[] pts, Vector3? direction=null)
        {
            Vector3 vu;
            if (direction.HasValue) vu = direction.Value;
            else vu= new Vector3(1, 0, 0);
            BoundingBox bbox = new BoundingBox();
            Vector3[] vects = new Vector3[3];

            vects[0]= vu.normalized;
            vects[1] = Vector3.up;
            vects[2] = Vector3.Cross(vects[0], vects[1]);

            Plane[] plnMins = new Plane[3];
            Plane[] plnMaxs = new Plane[3];
            Vector3[] ptMins = new Vector3[3];
            Vector3[] ptMaxs = new Vector3[3];

            for(int i = 0; i < 3; i++)
            {
                plnMins[i] = new Plane(-vects[i], pts[0]);
                plnMaxs[i] = new Plane(vects[i], pts[0]);
                ptMins[i] = pts[0];
                ptMaxs[i] = pts[0];
            }
            for(int i = 0; i < pts.Length; i++)
            {
                for(int j = 0; j < 3; j++)
                {
                    if (plnMins[j].GetSide(pts[i]))
                    {
                        plnMins[j] = new Plane(-vects[j], pts[i]);
                        ptMins[j] = pts[i];
                    }
                    if (plnMaxs[j].GetSide(pts[i]))
                    {
                        plnMaxs[j] = new Plane(vects[j], pts[i]);
                        ptMaxs[j] = pts[i];
                    }
                    
                }
            }

            Vector3[] horPts = new Vector3[4];
            horPts[0] = plnMins[2].ClosestPointOnPlane(ptMins[0]);
            horPts[1] = plnMins[2].ClosestPointOnPlane(ptMaxs[0]);
            horPts[3] = plnMaxs[2].ClosestPointOnPlane(ptMins[0]);
            horPts[2] = plnMaxs[2].ClosestPointOnPlane(ptMaxs[0]);

            Vector3[] outPts = new Vector3[8];
            for(int i = 0; i < 4; i++)
            {
                outPts[i] = plnMins[1].ClosestPointOnPlane(horPts[i]);
                outPts[i + 4] = plnMaxs[1].ClosestPointOnPlane(horPts[i]);
            }

            bbox.position = outPts[0];
            bbox.vects = vects;
            bbox.size = new Vector3(
                Vector3.Distance(outPts[0], outPts[1]),
                Vector3.Distance(outPts[0], outPts[4]),
                Vector3.Distance(outPts[0], outPts[3])
                );
            bbox.vertices = outPts;
            return bbox;
        }
    }
    public class GeometryBase
    {
        public Material _lineMatDefault;
        public string name="unnamedGeometry";
        public Material lineMatDefault
        {
            get
            {
                if (_lineMatDefault == null)
                    _lineMatDefault = CreateLineMaterial();
                return _lineMatDefault;
            }
            set
            {
                _lineMatDefault = value;
            }
        }

        public static Material CreateLineMaterial()        //创建GL绘制材质
        {
            Material lineMaterial;            
            // Unity has a built-in shader that is useful for drawing
            // simple colored things.
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            lineMaterial = new Material(shader);
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            // Turn on alpha blending
            lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            // Turn backface culling off
            lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            // Turn off depth writes
            lineMaterial.SetInt("_ZWrite", 0);
            return lineMaterial;
        }
        public virtual void update() { }
        //do regular OpenGL drawings
        public virtual void OnRenderObject() { }
        //post rendering GL drawings
        public virtual void OnPostRender() { }

    }
    public class PointsBase: GeometryBase
    {
        public Vector3[] vertices;
        public PointsBase()
        {
            vertices = new Vector3[0];
        }
        public PointsBase(Vector3[] pts)
        {
            vertices = pts;
        }
        public Vector3 LongestDirection(bool normalized = true)
        {
            Vector3? ld = PointsBase.LongestDirection(vertices, normalized);
            return ld.Value;
        }
        public static Vector3? LongestDirection(Vector3[] pts, bool normalized = true)
        {
            if (pts.Length < 2) return null;
            Vector3 ld = pts[1] - pts[0];
            for(int i = 0; i < pts.Length; i++)
            {
                int j = i + 1;
                if (j >= pts.Length) j = 0;
                Vector3 direct = pts[j] - pts[i];
                if (direct.magnitude > ld.magnitude)
                {
                    ld = direct;
                }
            }
            return ld;
        }
        public static Vector3[] MultiplyMatrix(Vector3[] pts, Matrix4x4 matrix)
        {
            for (int i = 0; i < pts.Length; i++)
            {
                pts[i] = matrix * pts[i];
            }
            return pts;
        }
        public static Vector3[] Translate(Vector3[] pts, Vector3 vect)
        {
            for (int i = 0; i < pts.Length; i++)
            {
                pts[i] += vect;
            }
            return pts;
        }
        public static Vector3[] Rotate(Vector3[] pts, Vector3 degrees)
        {
            Quaternion q = Quaternion.EulerRotation(degrees);
            Matrix4x4 matrix = Matrix4x4.Rotate(q);
            for (int i = 0; i < pts.Length; i++)
            {
                pts[i] = matrix * pts[i];
            }
            return pts;
        }
        public static Vector3[] Scale(Vector3[] pts, Vector3 scale)
        {
            for (int i = 0; i < pts.Length; i++)
            {
                for (int j = 0; j < 3; j++)
                    pts[i][j] *= scale[j];
            }
            return pts;
        }
        public static Vector3[] Scale(Vector3[] pts, Vector3 scale, Vector3[]vects, Vector3 origin)
        {
            Quaternion q1 = Quaternion.FromToRotation(vects[0], new Vector3(1, 0, 0));
            Quaternion q2 = Quaternion.FromToRotation(new Vector3(1, 0, 0), vects[0]);
            Matrix4x4 matrix1 = Matrix4x4.Rotate(q1);
            Matrix4x4 matrix2 = Matrix4x4.Rotate(q2);

            for (int i = 0; i < pts.Length; i++)
            {
                //push
                pts[i] = pts[i] - origin;
                pts[i] = matrix1 * pts[i];
                //scale
                for(int j=0;j<3;j++)
                    pts[i][j] *= scale[j];
                //pop
                pts[i] = matrix2 * pts[i];
                pts[i] = pts[i] + origin;
            }
            return pts;
        }

        public BoundingBox GetBoundingBox(Vector3? direction = null)
        {
            return BoundingBox.CreateFromPoints(vertices, direction);
        }
    }
    public class TrianglesBase : PointsBase
    {
        public int[] triangles;
        public TrianglesBase() : base()
        {
            triangles = new int[0];
        }
    }
    public class Meshable : TrianglesBase
    {
        public Vector3 direction=new Vector3(1,0,0);
        public Meshable()
        {
            this.vertices = new Vector3[0];
            this.triangles = new int[0];
        }
        public Mesh GetMeshForm()
        {
            Mesh m = new Mesh();
            m.vertices = this.vertices;
            m.triangles = this.triangles;
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }
        public virtual Meshable Scale(Vector3 scale, Vector3[] vects, Vector3 origin, bool duplicate=true)
        {
            Vector3[] pts = vertices.Clone() as Vector3[];
            pts = PointsBase.Scale(pts, scale, vects, origin);
            if (duplicate)
            {
                Meshable mb = new Meshable();
                mb.vertices = pts;
                mb.triangles = triangles.Clone() as int[];
                return mb;
            }
            else
            {
                vertices = pts;
            }
            return null;
        }

        /// <summary>
        /// merges the new given Meshable onto its own data
        /// also return the given Meshable with adjusted triangle index  
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public void merge(Meshable m)
        {
            int vertCount = this.vertices.Length;
            int[] tris = new int[m.triangles.Length];
            for (int i = 0; i < m.triangles.Length; i++)
            {
                tris[i] = m.triangles[i] + vertCount;
            }
            
            this.vertices = (this.vertices.Concat(m.vertices)).ToArray();
            this.triangles = (this.triangles.Concat(tris)).ToArray();
            
        }
        public virtual Meshable[] SplitByPlane(Plane pln)
        {
            Polyline edge;
            return SplitByPlane(pln, out edge);
        }
        public virtual Meshable[] SplitByPlane(Plane pln, out Polyline nakedEdge)
        {
            int nullP = 0;
            foreach (Vector3 p in vertices)
            {
                if (p == null) nullP++;
            }
            if (nullP > 0)
                Debug.LogWarningFormat("FOUND NULL POINTS, COUNT={0}", nullP);

            //split polygon by a plane and returns the naked edge
            Vector3? nkp1 = new Vector3?();
            Vector3? nkp2 = new Vector3?();
            List<Vector3> left = new List<Vector3>();
            List<Vector3> right = new List<Vector3>();

            Vector3 lastP = vertices[vertices.Length - 1];
            bool lastIsRight = pln.GetSide(lastP);
            bool isRight;
            List<Vector3> nakedPts = new List<Vector3>();

            for (int i = 0; i < vertices.Length; i++)
            {
                Vector3 p = vertices[i];
                isRight = pln.GetSide(p);

                if (lastIsRight != isRight)
                {
                    Ray r = new Ray(p, lastP - p);
                    float d;
                    pln.Raycast(r, out d);
                    Vector3 xp = r.GetPoint(d);
                    left.Add(xp);
                    right.Add(xp);
                    nakedPts.Add(xp);
                }
                if (isRight) right.Add(p);
                else left.Add(p);

                lastIsRight = isRight;
                lastP = p;
            }
            Polygon[] pgs = new Polygon[2];
            if (left.Count > 2) pgs[0] = new Polygon(left.ToArray());
            else pgs[0] = null;
            if (right.Count > 2) pgs[1] = new Polygon(right.ToArray());
            else pgs[1] = null;

            if (nakedPts.Count > 1)
            {
                nakedEdge = new Polyline(nakedPts.ToArray());
            }
            else nakedEdge = new Polyline();
            return pgs;
        }
        public Mesh GetNormalizedMesh(BoundingBox bbox)
        {
            //Vector3 org = bbox.position;
            Vector3 org = bbox.position;
            Quaternion q = Quaternion.FromToRotation(bbox.vects[0], new Vector3(1, 0, 0));
            Matrix4x4 matrix = Matrix4x4.Rotate(q);
            Vector3 scale = bbox.size;
            for(int i = 0; i < 3; i++)
            {
                if (scale[i] == 0) scale[i] = 1;
                scale[i] = 1 / scale[i];
            }
;           Mesh m = GetMeshForm();
            Vector3[] verts = m.vertices;
            for (int i = 0; i < verts.Length; i++)
            {
                verts[i] -= org;
                verts[i] = matrix * verts[i];
                verts[i].Scale(scale);
                //verts[i] += new Vector3(0.5f, 0, 0.5f);
            }
            m.vertices = verts;
            m.RecalculateNormals();
            m.RecalculateTangents();
            m.RecalculateBounds();
            return m;
        }
    }
    public class CompositMeshable : Meshable
    {
        public List<Meshable> components;
        public CompositMeshable()
        {
            this.components = new List<Meshable>();
        }
    
        public void Add(Meshable m)
        {
            merge(m);
            components.Add(m);
        }
        public void AddRange(Meshable[] ms)
        {
            foreach (Meshable m in ms)
            {
                this.Add(m);
            }
        }
        public Meshable Get(int index)
        {
            return this.components[index];
        }
        public override Meshable Scale(Vector3 scale, Vector3[] vects, Vector3 origin, bool duplicate = true)
        {
            Meshable dup = base.Scale(scale, vects, origin, duplicate);
            List<Meshable> comps = new List<Meshable>();
            foreach(Meshable m in components)
            {
                comps.Add(m.Scale(scale, vects, origin, true));
            }

            if (duplicate)
            {
                CompositMeshable cm = new CompositMeshable();
                cm.vertices = dup.vertices;
                cm.triangles = dup.triangles;
                cm.components = comps;
                return cm;
            }

            components = comps;
            return null;
        }
        public override Meshable[] SplitByPlane(Plane blade)
        {
            List<Polyline> nakedEdges = new List<Polyline>();
            List<Polygon> rights = new List<Polygon>();
            List<Polygon> lefts = new List<Polygon>();
            Form[] forms = new Form[2];
            foreach (Meshable pg in components)
            {
                Polyline edge;
                Meshable[] sides = pg.SplitByPlane(blade, out edge);
                //Debug.Log("edgeVertCount=" + edge.vertices.Length.ToString());
                if (edge.vertices.Length > 1)
                    nakedEdges.Add(edge);
                if (sides[0] != null) rights.Add((Polygon)sides[0]);
                if (sides[1] != null) lefts.Add((Polygon)sides[1]);
            }
            Debug.Log("nakeEdgeCount=" + nakedEdges.Count.ToString());
            if (nakedEdges.Count > 2)
            {
                Vector3[] capPts = GetCapVerts(nakedEdges, blade);
                Polygon leftCap = new Polygon(capPts.Reverse().ToArray());
                Polygon rightCap = new Polygon(capPts);
                rights.Add(rightCap);
                lefts.Add(leftCap);
            }
            else
            {
                Debug.LogWarning("nakedEdges.Count<2; =" + nakedEdges.Count);
            }

            if (rights.Count > 0) forms[0] = new Form(rights.ToArray());
            if (lefts.Count > 0) forms[1] = new Form(lefts.ToArray());
            return forms;
        }
        Vector3[] GetCapVerts(List<Polyline> nakedEdges, Plane pln)
        {
            //orient the edges according to plan
            Vector3 center = new Vector3();
            foreach (Polyline pl in nakedEdges)
            {
                center += pl.startPoint;
            }
            center /= nakedEdges.Count;



            for (int i = 0; i < nakedEdges.Count; i++)
            {
                Polyline pl = nakedEdges[i];
                Vector3 v1 = (center - pl.startPoint).normalized;
                Vector3 v2 = (pl.endPoint - pl.startPoint).normalized;
                Vector3 nml = Vector3.Cross(v2, v1);
                nml.Normalize();
                bool sameAsPlanNml = nml == pln.normal;
                //Debug.Log("nml=" + nml.ToString() + "pln,nml="+pln.normal.ToString() + sameAsPlanNml);
                if (!sameAsPlanNml)
                {
                    Vector3 temp = pl.vertices[0];
                    pl.vertices[0] = pl.vertices[1];
                    pl.vertices[1] = temp;
                }
            }

            List<Vector3> pts = new List<Vector3>();
            Polyline edge = nakedEdges[0];
            Polyline lastEdge = nakedEdges[nakedEdges.Count - 1];
            //pts.Add(edge.startPoint);
            int count = 0;
            while (edge != lastEdge && count < nakedEdges.Count)
            {
                //Debug.Log("edge=" + edge.ToString() + " count=" + count.ToString());
                count++;
                lastEdge = edge;
                for (int j = 0; j < nakedEdges.Count; j++)
                {
                    Polyline edge2 = nakedEdges[j];
                    //if (edge == edge2) continue;
                    bool flag = edge.endPoint == edge2.startPoint;
                    //Debug.Log(edge.startPoint.ToString() + "-" + edge.endPoint.ToString() + "," + edge2.startPoint.ToString()+"-"+edge2.endPoint.ToString() + flag.ToString());
                    if (flag)
                    {
                        pts.Add(edge2.startPoint);
                        edge = edge2;
                        break;
                    }
                }
                //string txt = "";
                //foreach(Vector3 v in pts)
                //{
                //    txt += v.ToString() + ",";
                //}
                //Debug.Log(txt);
            }
            return pts.ToArray();
        }
    }

    //Geometries
    public interface OnRender
    {
        void OnRenderObject();
    }

    public class Point : PointsBase { }
    public class Line : PointsBase
    {
        public Vector3 startPoint { get { return vertices[0]; } }
        public Vector3 endPoint { get { return vertices[1]; } }
        public Line() : base() { }
        public Line(Vector3 p1, Vector3 p2):base(new Vector3[] { p1, p2 })
        {
            
        }
    }
    public class Polyline : PointsBase, OnRender
    {
        Color color;
        bool closed = true;
        public Vector3 startPoint { get { return vertices[0]; } }
        public Vector3 endPoint { get { return vertices[vertices.Length-1]; } }
        public Polyline() : base() { }
        public Polyline(Vector3[] pts) : base(pts)
        {
            color = new Color(1f, 0.82f, 0.25f);
        }
        public override void OnRenderObject()
        {
            base.OnRenderObject();
            SGGeometry.GLRender.Polyline(vertices, closed, null, color);
                
        }
        public Line[] Segments()
        {

            Line[] lns = new Line[this.vertices.Length-1];
            for ( int i = 0; i < vertices.Length - 1; i++)
            {
                Vector3[] pts = new Vector3[2];
                pts[0] = vertices[i];
                pts[1] = vertices[i + 1];
                Line l = new Line();
                l.vertices = pts;
                lns[i] = l;
            }
            return lns;
        }

    }
    public class Face : Meshable
    {
        public Face() : base() { }
    }
    public class Quad : Face
    {
        public Quad(Vector3[] verts) : base()
        {
            this.vertices = verts;
            this.triangles = new int[] { 0, 1, 2, 0, 2, 3 };
        }
    }
    public class Triangle : Face
    {
        public Triangle(Vector3[] verts) : base()
        {
            this.vertices = verts;
            this.triangles = new int[] { 0, 1, 2 };
        }
    }
    public class Polygon : CompositMeshable
    {
        Polyline boundary;
        public Polygon() : base()
        {
        }
        public Polygon(Polyline ply) : this(ply.vertices)
        {
        }
        public Polygon(Vector3[] pts):base()
        {
            boundary = new Polyline(pts);
            if (pts.Length > 4)
            {
                TriangulatorV3 tr = new TriangulatorV3(pts);
                triangles = tr.Triangulate();
            }
            else if(pts.Length == 4)
            {
                triangles = new int[] { 0, 1, 2, 0, 2, 3 };
            }
            else if(pts.Length ==3)
            {
                triangles = new int[] { 0, 1, 2 };
            }
            this.vertices = pts;
        } 
        public Form Extrude(Vector3 magUp)
        {
            List<Polygon> pgs = new List<Polygon>();
            pgs.Add(new Polygon(vertices));
            Vector3[] ptsTop = new Vector3[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                ptsTop[i] = vertices[i] + magUp;
                int j = i + 1;
                if (j >= vertices.Length) j = 0;

                Vector3[] pts = new Vector3[4];
                pts[3] = vertices[i];
                pts[2] = vertices[j];
                pts[1] = vertices[j] + magUp;
                pts[0] = vertices[i] + magUp;
                Polygon pg = new Polygon(pts);
                pgs.Add(pg);
            }
            pgs.Add(new Polygon(ptsTop.Reverse().ToArray()));
            Form outForm = new Form(pgs.ToArray());
            return outForm;
        }
       
    }
    public class Form : CompositMeshable
    {
        public Form() : base()
        {
        }
        public Form(Polygon[] polygons) : base()
        {
            //foreach(Polygon pg in polygons)
            //{
            //    if (pg.vertices.Length > 2) this.Add(pg);
            //}
            this.AddRange(polygons);
        }

        
        
    }

    public class TriangulatorV3
    {
        private List<Vector3> m_points = new List<Vector3>();

        public TriangulatorV3(Vector3[] points)
        {
            points = flipPoints(points);
            m_points = new List<Vector3>(points);

        }

        public Vector3[] flipPoints(Vector3[] ipts)
        {
            Vector3[] pts = new Vector3[ipts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                float x = ipts[i].x;
                float y = ipts[i].z;
                float z = 0;
                pts[i] = new Vector3(x, y, z);
            }
            return pts;
        }

        public int[] Triangulate()
        {
            List<int> indices = new List<int>();

            int n = m_points.Count;
            if (n < 3)
                return indices.ToArray();

            int[] V = new int[n];
            if (Area() > 0)
            {
                for (int v = 0; v < n; v++)
                    V[v] = v;
            }
            else
            {
                for (int v = 0; v < n; v++)
                    V[v] = (n - 1) - v;
            }

            int nv = n;
            int count = 2 * nv;
            for (int m = 0, v = nv - 1; nv > 2;)
            {
                if ((count--) <= 0)
                    return indices.ToArray();

                int u = v;
                if (nv <= u)
                    u = 0;
                v = u + 1;
                if (nv <= v)
                    v = 0;
                int w = v + 1;
                if (nv <= w)
                    w = 0;

                if (Snip(u, v, w, nv, V))
                {
                    int a, b, c, s, t;
                    a = V[u];
                    b = V[v];
                    c = V[w];
                    indices.Add(a);
                    indices.Add(b);
                    indices.Add(c);
                    m++;
                    for (s = v, t = v + 1; t < nv; s++, t++)
                        V[s] = V[t];
                    nv--;
                    count = 2 * nv;
                }
            }

            indices.Reverse();
            return indices.ToArray();
        }

        private float Area()
        {
            int n = m_points.Count;
            float A = 0.0f;
            for (int p = n - 1, q = 0; q < n; p = q++)
            {
                Vector3 pval = m_points[p];
                Vector3 qval = m_points[q];
                A += pval.x * qval.y - qval.x * pval.y;
            }
            return (A * 0.5f);
        }

        private bool Snip(int u, int v, int w, int n, int[] V)
        {
            int p;
            Vector3 A = m_points[V[u]];
            Vector3 B = m_points[V[v]];
            Vector3 C = m_points[V[w]];
            if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
                return false;
            for (p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                    continue;
                Vector3 P = m_points[V[p]];
                if (InsideTriangle(A, B, C, P))
                    return false;
            }
            return true;
        }

        private bool InsideTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
        {
            float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
            float cCROSSap, bCROSScp, aCROSSbp;

            ax = C.x - B.x; ay = C.y - B.y;
            bx = A.x - C.x; by = A.y - C.y;
            cx = B.x - A.x; cy = B.y - A.y;
            apx = P.x - A.x; apy = P.y - A.y;
            bpx = P.x - B.x; bpy = P.y - B.y;
            cpx = P.x - C.x; cpy = P.y - C.y;

            aCROSSbp = ax * bpy - ay * bpx;
            cCROSSap = cx * apy - cy * apx;
            bCROSScp = bx * cpy - by * cpx;

            return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
        }
    }

    public class TriangulatorV2
    {
        private List<Vector2> m_points = new List<Vector2>();

        public TriangulatorV2(Vector3[] points)
        {
            Vector2[] pts = v3tov2(points);
            m_points = new List<Vector2>(pts);

        }

        public Vector2[] v3tov2(Vector3[] ipts)
        {
            Vector2[] pts = new Vector2[ipts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                float x = ipts[i].x;
                float y = ipts[i].z;
                pts[i] = new Vector2(x, y);
            }
            return pts;
        }
        public Vector3[] v2tov3(Vector2[] ipts)
        {
            Vector3[] pts = new Vector3[ipts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                float x = ipts[i].x;
                float y = ipts[i].y;
                pts[i] = new Vector3(x, 0, y);
            }
            return pts;
        }

        public int[] Triangulate()
        {
            List<int> indices = new List<int>();

            int n = m_points.Count;
            if (n < 3)
                return indices.ToArray();

            int[] V = new int[n];
            if (Area() > 0)
            {
                for (int v = 0; v < n; v++)
                    V[v] = v;
            }
            else
            {
                for (int v = 0; v < n; v++)
                    V[v] = (n - 1) - v;
            }

            int nv = n;
            int count = 2 * nv;
            for (int m = 0, v = nv - 1; nv > 2;)
            {
                if ((count--) <= 0)
                    return indices.ToArray();

                int u = v;
                if (nv <= u)
                    u = 0;
                v = u + 1;
                if (nv <= v)
                    v = 0;
                int w = v + 1;
                if (nv <= w)
                    w = 0;

                if (Snip(u, v, w, nv, V))
                {
                    int a, b, c, s, t;
                    a = V[u];
                    b = V[v];
                    c = V[w];
                    indices.Add(a);
                    indices.Add(b);
                    indices.Add(c);
                    m++;
                    for (s = v, t = v + 1; t < nv; s++, t++)
                        V[s] = V[t];
                    nv--;
                    count = 2 * nv;
                }
            }

            indices.Reverse();
            return indices.ToArray();
        }

        private float Area()
        {
            int n = m_points.Count;
            float A = 0.0f;
            for (int p = n - 1, q = 0; q < n; p = q++)
            {
                Vector2 pval = m_points[p];
                Vector2 qval = m_points[q];
                A += pval.x * qval.y - qval.x * pval.y;
            }
            return (A * 0.5f);
        }

        private bool Snip(int u, int v, int w, int n, int[] V)
        {
            int p;
            Vector2 A = m_points[V[u]];
            Vector2 B = m_points[V[v]];
            Vector2 C = m_points[V[w]];
            if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
                return false;
            for (p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                    continue;
                Vector2 P = m_points[V[p]];
                if (InsideTriangle(A, B, C, P))
                    return false;
            }
            return true;
        }

        private bool InsideTriangle(Vector2 A, Vector2 B, Vector2 C, Vector2 P)
        {
            float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
            float cCROSSap, bCROSScp, aCROSSbp;

            ax = C.x - B.x; ay = C.y - B.y;
            bx = A.x - C.x; by = A.y - C.y;
            cx = B.x - A.x; cy = B.y - A.y;
            apx = P.x - A.x; apy = P.y - A.y;
            bpx = P.x - B.x; bpy = P.y - B.y;
            cpx = P.x - C.x; cpy = P.y - C.y;

            aCROSSbp = ax * bpy - ay * bpx;
            cCROSSap = cx * apy - cy * apx;
            bCROSScp = bx * cpy - by * cpx;

            return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
        }
    }

    public class OldCreator
    {
        public static GameObject CreateMeshObject(Mesh m)
        {
            GameObject obj = new GameObject();
            MeshFilter filter = obj.AddComponent<MeshFilter>() as MeshFilter;
            filter.mesh = m;
            return obj;
        }
        public static GameObject CreateLineObject(SGGeometry.PointsBase line, bool closed=false, float width=0.1f)
        {
            Vector3[] verts = line.vertices;
            GameObject obj = new GameObject();
            LineRenderer renderer = obj.AddComponent<LineRenderer>() as LineRenderer;
            renderer.positionCount = verts.Length;
            renderer.SetPositions(verts);
            renderer.startColor = Color.black;
            renderer.startWidth = width;
            renderer.loop = closed;
            return obj;
        }
            
    }

    public class Rider
    {
        public virtual void updateObject() { }
        public virtual void Destroy() { }
    }
    public class SingleObjectRider : Rider
    {
        public GameObject gameObject;
        string name = "unnamed<singleObjectRider>";
        public SingleObjectRider()
        {
            gameObject = new GameObject(this.name);
        }
    }
    public class MultiObjectRider : Rider
    {
        public List<GameObject> gameObjects;
        string name = "unnamed<multiObjectRider>";
        public MultiObjectRider()
        {
            gameObjects = new List<GameObject>();
        }
        public MultiObjectRider(List<GameObject> objs)
        {
            this.gameObjects = objs;
        }
        public virtual void updateObject() { }
    }
    public class MultiLineRider : MultiObjectRider
    {
        float width = 0.1f;
        Color color = Color.black;
        List<SGGeometry.PointsBase> lines;

        public MultiLineRider() : base()
        {
            lines = new List<SGGeometry.PointsBase>();
        }
            
        public MultiLineRider(SGGeometry.PointsBase[] inLines) : base()
        {
            lines = new List<SGGeometry.PointsBase>(inLines);
            foreach (SGGeometry.PointsBase line in lines)
            {
                this.AddLine(line);
            }
        }
        public void AddLine(SGGeometry.PointsBase line)
        {
            GameObject obj = OldCreator.CreateLineObject(line,true);
            this.gameObjects.Add(obj);
        }
        public override void updateObject()
        {
            for(int i =0;i<this.gameObjects.Count;i++)
            {
                GameObject obj = this.gameObjects[i];
                LineRenderer lr = obj.GetComponent<LineRenderer>() as LineRenderer;
                lr.startWidth = this.width;
                lr.startColor = this.color;
                lr.SetPositions(this.lines[i].vertices);
            } 
        }
            
            
    }
}

namespace MeshFactory
{
    //public class MeshQuad : RMesh
    //{
    //    public MeshQuad(Vector3[] verts)
    //    {
    //        int[] triangles = new int[]
    //        {
    //        0,1,2,3
    //        };
    //        _mesh.vertices = verts;
    //        _mesh.triangles = triangles;
    //        _mesh.RecalculateNormals();
    //    }
    //}

    /// <summary>
    /// Triangulator is a class for triangulating any given polyline
    /// http://wiki.unity3d.com/index.php?title=Triangulator
    ///    Vector3[] vertices2D = new Vector3[] {.....}
    ///    Triangulator tr = new Triangulator(vertices2D);
    ///    int[] indices = tr.Triangulate();
    ///    Mesh msh = new Mesh();
    ///    msh.vertices = vertices;
    ///    msh.triangles = indices;
    ///    msh.RecalculateNormals();
    ///    msh.RecalculateBounds();
    /// </summary>
    public class TriangulatorV3
    {
        private List<Vector3> m_points = new List<Vector3>();

        public TriangulatorV3(Vector3[] points)
        {
            points = flipPoints(points);
            m_points = new List<Vector3>(points);

        }

        public Vector3[] flipPoints(Vector3[] ipts)
        {
            Vector3[] pts = new Vector3[ipts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                float x = ipts[i].x;
                float y = ipts[i].z;
                float z = 0;
                pts[i] = new Vector3(x, y, z);
            }
            return pts;
        }

        public int[] Triangulate()
        {
            List<int> indices = new List<int>();

            int n = m_points.Count;
            if (n < 3)
                return indices.ToArray();

            int[] V = new int[n];
            if (Area() > 0)
            {
                for (int v = 0; v < n; v++)
                    V[v] = v;
            }
            else
            {
                for (int v = 0; v < n; v++)
                    V[v] = (n - 1) - v;
            }

            int nv = n;
            int count = 2 * nv;
            for (int m = 0, v = nv - 1; nv > 2;)
            {
                if ((count--) <= 0)
                    return indices.ToArray();

                int u = v;
                if (nv <= u)
                    u = 0;
                v = u + 1;
                if (nv <= v)
                    v = 0;
                int w = v + 1;
                if (nv <= w)
                    w = 0;

                if (Snip(u, v, w, nv, V))
                {
                    int a, b, c, s, t;
                    a = V[u];
                    b = V[v];
                    c = V[w];
                    indices.Add(a);
                    indices.Add(b);
                    indices.Add(c);
                    m++;
                    for (s = v, t = v + 1; t < nv; s++, t++)
                        V[s] = V[t];
                    nv--;
                    count = 2 * nv;
                }
            }

            indices.Reverse();
            return indices.ToArray();
        }

        private float Area()
        {
            int n = m_points.Count;
            float A = 0.0f;
            for (int p = n - 1, q = 0; q < n; p = q++)
            {
                Vector3 pval = m_points[p];
                Vector3 qval = m_points[q];
                A += pval.x * qval.y - qval.x * pval.y;
            }
            return (A * 0.5f);
        }

        private bool Snip(int u, int v, int w, int n, int[] V)
        {
            int p;
            Vector3 A = m_points[V[u]];
            Vector3 B = m_points[V[v]];
            Vector3 C = m_points[V[w]];
            if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
                return false;
            for (p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                    continue;
                Vector3 P = m_points[V[p]];
                if (InsideTriangle(A, B, C, P))
                    return false;
            }
            return true;
        }

        private bool InsideTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
        {
            float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
            float cCROSSap, bCROSScp, aCROSSbp;

            ax = C.x - B.x; ay = C.y - B.y;
            bx = A.x - C.x; by = A.y - C.y;
            cx = B.x - A.x; cy = B.y - A.y;
            apx = P.x - A.x; apy = P.y - A.y;
            bpx = P.x - B.x; bpy = P.y - B.y;
            cpx = P.x - C.x; cpy = P.y - C.y;

            aCROSSbp = ax * bpy - ay * bpx;
            cCROSSap = cx * apy - cy * apx;
            bCROSScp = bx * cpy - by * cpx;

            return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
        }
    }

    public class TriangulatorV2
    {
        private List<Vector2> m_points = new List<Vector2>();

        public TriangulatorV2(Vector3[] points)
        {
            Vector2[] pts = v3tov2(points);
            m_points = new List<Vector2>(pts);

        }

        public Vector2[] v3tov2(Vector3[] ipts)
        {
            Vector2[] pts = new Vector2[ipts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                float x = ipts[i].x;
                float y = ipts[i].z;
                pts[i] = new Vector2(x, y);
            }
            return pts;
        }
        public Vector3[] v2tov3(Vector2[] ipts)
        {
            Vector3[] pts = new Vector3[ipts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                float x = ipts[i].x;
                float y = ipts[i].y;
                pts[i] = new Vector3(x, 0, y);
            }
            return pts;
        }

        public int[] Triangulate()
        {
            List<int> indices = new List<int>();

            int n = m_points.Count;
            if (n < 3)
                return indices.ToArray();

            int[] V = new int[n];
            if (Area() > 0)
            {
                for (int v = 0; v < n; v++)
                    V[v] = v;
            }
            else
            {
                for (int v = 0; v < n; v++)
                    V[v] = (n - 1) - v;
            }

            int nv = n;
            int count = 2 * nv;
            for (int m = 0, v = nv - 1; nv > 2;)
            {
                if ((count--) <= 0)
                    return indices.ToArray();

                int u = v;
                if (nv <= u)
                    u = 0;
                v = u + 1;
                if (nv <= v)
                    v = 0;
                int w = v + 1;
                if (nv <= w)
                    w = 0;

                if (Snip(u, v, w, nv, V))
                {
                    int a, b, c, s, t;
                    a = V[u];
                    b = V[v];
                    c = V[w];
                    indices.Add(a);
                    indices.Add(b);
                    indices.Add(c);
                    m++;
                    for (s = v, t = v + 1; t < nv; s++, t++)
                        V[s] = V[t];
                    nv--;
                    count = 2 * nv;
                }
            }

            indices.Reverse();
            return indices.ToArray();
        }

        private float Area()
        {
            int n = m_points.Count;
            float A = 0.0f;
            for (int p = n - 1, q = 0; q < n; p = q++)
            {
                Vector2 pval = m_points[p];
                Vector2 qval = m_points[q];
                A += pval.x * qval.y - qval.x * pval.y;
            }
            return (A * 0.5f);
        }

        private bool Snip(int u, int v, int w, int n, int[] V)
        {
            int p;
            Vector2 A = m_points[V[u]];
            Vector2 B = m_points[V[v]];
            Vector2 C = m_points[V[w]];
            if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
                return false;
            for (p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                    continue;
                Vector2 P = m_points[V[p]];
                if (InsideTriangle(A, B, C, P))
                    return false;
            }
            return true;
        }

        private bool InsideTriangle(Vector2 A, Vector2 B, Vector2 C, Vector2 P)
        {
            float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
            float cCROSSap, bCROSScp, aCROSSbp;

            ax = C.x - B.x; ay = C.y - B.y;
            bx = A.x - C.x; by = A.y - C.y;
            cx = B.x - A.x; cy = B.y - A.y;
            apx = P.x - A.x; apy = P.y - A.y;
            bpx = P.x - B.x; bpy = P.y - B.y;
            cpx = P.x - C.x; cpy = P.y - C.y;

            aCROSSbp = ax * bpy - ay * bpx;
            cCROSSap = cx * apy - cy * apx;
            bCROSScp = bx * cpy - by * cpx;

            return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
        }
    }

    public class MeshMaker
    {
        public static Mesh joinMeshes(Mesh[] meshs, bool mergeSubMesh = false)
        {
            Mesh outMesh = new Mesh();
            List<Vector3> pts = new List<Vector3>();
            List<int> tris = new List<int>();
            int vertsCount = 0;

            foreach (Mesh mesh in meshs)
            {
                vertsCount = pts.Count;
                foreach (Vector3 p in mesh.vertices)
                {
                    pts.Add(p);
                }
                List<int> mtris = new List<int>();
                foreach (int t in mesh.triangles)
                {
                    int nt = t + vertsCount;
                    tris.Add(nt);
                    mtris.Add(nt);

                }
                
            }

            outMesh.vertices = pts.ToArray();
            outMesh.triangles = tris.ToArray();
            outMesh.RecalculateNormals();

            return outMesh;
        }
        public static Mesh makeQuad(Vector3[] verts)
        {
            Mesh m = new Mesh();
            int[] triangles = new int[]
            {
            0,1,2,3
            };
            m.vertices = verts;
            m.triangles = triangles;
            m.RecalculateNormals();
            return m;
        }
        public static Mesh makeTriangle(Vector3[] verts)
        {
            Mesh m = new Mesh();
            int[] triangles = new int[]
                {
            0,1,2
                };
            m.vertices = verts;
            m.triangles = triangles;
            m.RecalculateNormals();
            return m;
        }
        public static Mesh makePolygon(Vector3[] verts)
        {
            Mesh m = new Mesh();
            TriangulatorV3 tr = new TriangulatorV3(verts);
            int[] triangles = tr.Triangulate();
            m.vertices = verts;
            m.triangles = triangles;
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }
        public static Mesh makeExtrusion(Vector3[] iverts, float height, bool cap = true)
        {
            Mesh m = new Mesh();
            Vector3 up = Vector3.up * height;
            List<Vector3> verts = new List<Vector3>();
            List<int> tris = new List<int>();
            for (int i = 0; i < iverts.Length; i++)
            {
                int j = i + 1;
                if (j >= iverts.Length)
                {
                    j -= iverts.Length;
                }
                int vc = verts.Count;
                verts.Add(iverts[i]);
                verts.Add(iverts[j]);
                verts.Add(iverts[j] + up);
                verts.Add(iverts[i] + up);

                tris.Add(vc);
                tris.Add(vc + 2);
                tris.Add(vc + 1);
                tris.Add(vc);
                tris.Add(vc + 3);
                tris.Add(vc + 2);
            }
            m.vertices = verts.ToArray();
            m.triangles = tris.ToArray();

            //make caps
            if (cap)
            {
                Vector3[] upbound = MeshMaker.addVects(iverts, up);
                Mesh top = makePolygon(upbound);
                Mesh bot = makePolygon(iverts);
                m = joinMeshes(new Mesh[] { m, top, bot });
            }

            m.RecalculateBounds();
            m.RecalculateNormals();
            return m;
        }

        public static Vector3[] addVects(Vector3[] vects, Vector3 vect)
        {
            List<Vector3> outVects = new List<Vector3>();
            foreach (Vector3 v in vects)
            {
                outVects.Add(vect + v);
            }
            return outVects.ToArray();
        }
        public static Vector3[] addVects(Vector3 vect, Vector3[] vects)
        {
            return addVects(vects, vect);
        }
        public static Vector3[] addVects(Vector3[] vects1, Vector3[] vects2)
        {
            List<Vector3> outVects = new List<Vector3>();
            for (int i = 0; i < vects1.Length; i++)
            {
                outVects.Add(vects1[i] + vects2[i]);
            }
            return outVects.ToArray();
        }
            
    }


}

