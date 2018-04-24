using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;
using System;

namespace DrawTool
{
    public class Edge : Polyline
    {
        public Node[] nodes;
        public Edge()
        {
            nodes = new Node[2];
        }
        public Edge(Node n1, Node n2) : this()
        {
            nodes[0] = n1;
            nodes[1] = n2;
        }
     
    }

    public class Road : Edge
    {
        public float width;
        public float sideWalkWidth;
        public Polyline centerLine          //中心线
        {
            get { return this; }
            set { vertices = value.vertices; }
        }
        public Polyline[] edges;           //道路边线  
        public Polyline[] sideWalkEdges;   //人行道边线     

        public Road() : base() { }
        
        public Road(Intersection itrA, Intersection itrB) : base(itrA,itrB)  
        {
            itrA.edges.Add(this);
            itrB.edges.Add(this);

            width = 1.6f;
            sideWalkWidth = 0.2f;

            centerLine = new Polyline(new Vector3[] { itrA.position, itrB.position });
            List<Vector3[]> pls = Utility.CountVectorDeviation(itrA.position, itrB.position, width / 2);
            try
            {
                edges = new Polyline[2] { new Polyline(pls[0]), new Polyline(pls[1]) };
            }
            catch(Exception e)
            {
                //Debug.LogFormat("edges.Count={0},exception:{1}", pls.Count, e.ToString());
            }
            
          
            //sideWalkEdges = new Polyline[2] { new Polyline(Utility.CountVectorDeviation(itrA.position, itrB.position, width / 2 - sideWalkWidth)[0]), new Polyline(Utility.CountVectorDeviation(itrA.position, itrB.position, width / 2 - sideWalkWidth)[1]) };
           
        }


        public Line EdgeFromIntersection(Intersection inr,int side)
        {
            return null;
        }
       

       
    }
}
