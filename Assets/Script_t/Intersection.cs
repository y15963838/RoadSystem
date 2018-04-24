using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;

namespace DrawTool
{
    public class Node
    {
        public Vector3 position;
        public List<Edge> edges;

        public Node()
        {
            edges = new List<Edge>();
        }
        public Node(Vector3 v) : this()
        {
            position = v;
        }
        public void sortEdgeCW()//sort counter clockwise 顺时针排序
        {
            //把排序后的edge直接update this.edges
        }
    }


    public class Intersection : Node
    {    
        public float radious;
        public Polygon geomtry;

        public Intersection() : base() { }
        public Intersection(Vector3 v) : base(v) { }
       

    }
}

