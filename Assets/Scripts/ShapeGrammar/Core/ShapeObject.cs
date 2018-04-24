using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;


public class ShapeObject : MonoBehaviour
{
    public Vector3 Size
    {
        get
        {
            return transform.localScale;
        }
        set
        {
            for (int i = 0; i < 3; i++)
                if (value[i] == 0) value[i] = 1;
            transform.localScale = value;
        }
    }
    public Vector3[] Vects
    {
        get
        {
            Vector3 z = transform.forward;
            Vector3 y = transform.up;
            Vector3 x = Vector3.Cross(y,z);
            return new Vector3[] { x, y, z };
        }
    }
    

    public static Material DefaultMat
    {
        get
        {
            if(_defaultMat == null)
            {
                _defaultMat = Resources.Load("Mat0") as Material;
            }
            return _defaultMat;
        }
    }

    public Meshable meshable;
    public int step;
    public bool drawScope = true;
    private static Material _defaultMat;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;
   
    public string Format()
    {
        string txt = "";
        string draw = "-";
        if (gameObject.activeSelf) draw = "+";
        string ruleName;
        //if (parentRule == null) ruleName = "unnamedRule";
        //else ruleName = parentRule.name;
        txt += draw + name + "_(step" + step+")";

        return txt;
    }

 
    Vector3[] makeBoxPoints()
    {
        Vector3[] opts = new Vector3[8];
        Vector3[] vects = Vects;
        for (int i = 0; i < 3; i++)
            vects[i] *= Size[i];
        Vector3 size = Size;

        //opts[0] = new Vector3(-0.5f,-0.5f,-0.5f);
        opts[0] = transform.position;
        opts[1] = opts[0] + (vects[0]);
        opts[2] = opts[1] + (vects[2]);
        opts[3] = opts[0]+ (vects[2]);
        for( int i = 0; i < 4; i++)
        {
            opts[i + 4] = opts[i] + vects[1];
        }
        return opts;

    }
    
   
    public void SetMeshable(Meshable imeshable, Vector3? direction=null)
    {
        meshable = imeshable;
        Vector3 vectu;
        if (direction.HasValue) vectu = direction.Value;
        else vectu = new Vector3(1, 0, 0);
        BoundingBox bbox = meshable.GetBoundingBox(vectu);

        transform.position = bbox.vertices[0];
        transform.LookAt(bbox.vertices[3]);
        Size = bbox.size;
        
        Mesh mesh = meshable.GetNormalizedMesh(bbox);
        GetComponent<MeshFilter>().mesh = mesh;
        //print("mesh.verticeCount=" + mesh.vertexCount.ToString());

    }
    private void OnDestroy()
    {
        Debug.LogWarning("SHAPE OBJECT DESTROY WARNING:" + Format());
    }
    public static ShapeObject CreateBasic()
    {
        GameObject o = new GameObject();
        ShapeObject so = o.AddComponent<ShapeObject>();
        so.meshFilter = o.AddComponent<MeshFilter>();
        so.meshRenderer = o.AddComponent<MeshRenderer>();
        BoxCollider bc= o.AddComponent<BoxCollider>();
        bc.center = new Vector3(0.5f, 0.5f, 0.5f);
        //o.AddComponent<HighlightMouseOver>();
        so.meshRenderer.material = DefaultMat;
        return so;
    }
    public static ShapeObject CreatePolygon(Vector3[] pts)
    {
        Polygon pg = new Polygon(pts);
        ShapeObject so = ShapeObject.CreateBasic();
        Vector3? ld = PointsBase.LongestDirection(pts);
        so.SetMeshable(pg, ld);
        return so;

    }
    public static ShapeObject CreateExtrusion(Vector3[] pts, float d)
    {
        Vector3 magUp = new Vector3(0, d, 0);
        Polygon pg = new Polygon(pts);
        Form ext = pg.Extrude(magUp);

        ShapeObject so = ShapeObject.CreateBasic();
        Vector3? ld = PointsBase.LongestDirection(pts);
        so.SetMeshable(ext, ld);
        return so;
    }
    public static ShapeObject CreateMeshable(Meshable mb)
    {
        ShapeObject so = ShapeObject.CreateBasic();
        Vector3 ld = mb.direction;
        so.SetMeshable(mb, ld);
        return so;
    }
}
