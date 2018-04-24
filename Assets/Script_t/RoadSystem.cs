using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;

namespace DrawTool
{
    public class NodeEdgeSystem
    {
        public List<Node> nodes;       
        public List<Edge> edges;
        public List<Polygon> polygons;
     
        public NodeEdgeSystem()
        {
            nodes = new List<Node>();
            edges = new List<Edge>();
        }

    }

    public class RoadSystem:NodeEdgeSystem
    {
       
        public List<Plot> plots;
        public List<Road> roads;
        public List<Intersection> intersections;

        public RoadSystem() : base()
        {
            plots = new List<Plot>();
            roads = new List<Road>();
            intersections = new List<Intersection>();
        }
      
        public Road GetRoad(int i)
        {
            return (Road)edges[i];
        }
   
        public void AddIntersection(Intersection ints)      
        {
            if(!intersections.Contains(ints))
                intersections.Add(ints);
        }
        public void AddIntersection(Vector3 v)
        {
            Intersection ints = new Intersection(v);
            AddIntersection(ints);
        }

        public void AddRoad(Road road)
        {
            roads.Add(road);
        
        }
        public void AddRoad(Intersection itrA, Intersection itrB)
        {
            if(itrA ==null || itrB == null)
            {
                Debug.Log("null node found:");
                return;
            }
            Road road = new Road(itrA, itrB);
            AddRoad(road);
        }


        public void OnRenderObject()
        {
            if (roads == null) return;
          
            for (int i = 0; i < roads.Count; i++)
            {
                SGGeometry.GLRender.Polyline(roads[i].centerLine.vertices,false, null, Color.black);

                //SGGeometry.GLRender.Polyline(roads[i].edges[0].vertices, false, null, Color.blue);
                //SGGeometry.GLRender.Polyline(roads[i].edges[1].vertices, false, null, Color.blue);

                //SGGeometry.GLRender.Polyline(roads[i].sideWalkEdges[0].vertices, false, null, Color.white);
                //SGGeometry.GLRender.Polyline(roads[i].sideWalkEdges[1].vertices, false, null, Color.white);
            }
        }

        public void FindPolygon()
        {
            //traverse
            List<List<Node>> newPolygons = new List<List<Node>>();
            traverse(nodes[0], newPolygons, nodes[0], null, true);
            //运行后，在 polygons 里会有一堆带重复的闭合

            //remove duplicated polygons
            //remove nolonger exist polygons:对比 newPolygons vs this.polygons
            //add to system : this.polygons = .....
        }

        public void traverse(Node node, List<List<Node>> polygons, Node initNode, List<Node> history=null, bool first=true)
        {

            //建立历史
            if (history == null) history = new List<Node>();
            if (history.Contains(node))
            {
                int index = history.IndexOf(node);
                history = history.GetRange(index, history.Count - index);
                history.Add(node);

                if (history.Count > 3)
                    polygons.Add(history.ToList());
                return;
            }

            //记录历史
            history.Add(node);

            //结束条件
            if ((node == initNode && first == false) || node.edges.Count == 0)
            {
                return;
            }

            //遍历子树
            foreach (Edge e in node.edges)
            {
                Node nextNode;
                if (e.nodes[0] == node) nextNode = e.nodes[1];
                else nextNode = e.nodes[0];
                traverse(nextNode, polygons, initNode, history, false);
            }
        }

    }
}
