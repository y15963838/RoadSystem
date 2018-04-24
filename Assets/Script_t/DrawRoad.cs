using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SGGeometry;
using System.Linq;

namespace DrawTool
{
    public class DrawRoad : Tool
    {
        private Intersection lastNode;
        private Intersection currNode;
        private Vector3 FinalPoint;   
        private int pause = -1;
      
        List<Intersection> intersections;   
        List<Road> roads;
        RoadSystem roadSystem;

        public DrawRoad() : base()
        {     
            intersections = new List<Intersection>();
            roads = new List<Road>();
            workPlane = new Plane(Vector3.up, Vector3.zero);
            markers = new List<GameObject>();
            pts = new List<Vector3>();
            drawing = false;
            lastNode = null;
        }
        public DrawRoad(RoadSystem rs):this()
        {
            SetRoadSystem(rs);
        }
        public void SetRoadSystem(RoadSystem rs)
        {
            roadSystem = rs;
        }

        public override void Execute()
        {
            if (!drawing)
            {
        
                pts = new List<Vector3>();
              
            }
            drawing = true;    
            Vector3 xp = Utility.MouseToWorld(workPlane);

            ////////////////////////////////////////////////////////////////////////
            currNode=AddPoint(xp);   //添加点
            pts.Add(FinalPoint);
            AddLine();      //添加线
            ////////////////////////////////////////////////////////////////////////
            markers.Add(CreateMarker(FinalPoint));
            Debug.Log(string.Format("点数：{0}, 线数：{1}", roadSystem.intersections.Count, roadSystem.roads.Count));
            //lastNode.sortEdgeCW();
            lastNode = currNode;
        }

        private int HasClosePointInRoadSystem(Vector3 v)
        {
            if (roadSystem.intersections == null) return -1;
        
            for (int i = 0; i < roadSystem.intersections.Count; i++)
            {
                bool flag = Utility.IsClose(v, roadSystem.intersections[i].position);     //比较的应该是  xp与之前记录进去的所有点
                if (flag)
                {
                    Debug.Log("遇到重复点，点号是:" + i);
                    return i;     
                }
            }
            return -1;
        }
        private Intersection AddPoint(Vector3 xp)
        {  
            //看看系统里鼠标附近是否已存在节点
            int index = HasClosePointInRoadSystem(xp);
        
            Intersection ints;
            if (index >= 0)
            {
                ints = (Intersection)roadSystem.intersections[index];
              
            }
            else
            {
                ints = new Intersection(xp);
              
            }

            Debug.Log(ints.position.x);
            roadSystem.AddIntersection(ints);
            FinalPoint = ints.position;
            int nodesCount = roadSystem.intersections.Count;

            if (index >= 0)
            {
                return (Intersection)roadSystem.intersections[index];
            }
            else
            {
                return roadSystem.intersections[nodesCount - 1];
            }
            

            //把与 node “ints” 项连的edge排序

        }
        private void AddLine()
        {
            if (roadSystem.intersections.Count > 1)
            {
                //int n1 = pts.Count - 1;
                //int n2 = pts.Count - 2;
                if(lastNode!=null)
                //if (n2 != pause)
                {
                    //int ii = roadSystem.intersections.Count - 1;
                    //Intersection it1 = roadSystem.intersections[ii - 1];
                    //Intersection it2 = roadSystem.intersections[ii];

                    //Intersection itrA = new Intersection(pts[n2]);
                    //Intersection itrB = new Intersection(pts[n1]);

                    //Road road = new Road(itrA,itrB);    
                    // Road road = new Road(it1, it2);
                    Debug.Log("C:"+currNode.position.x);
                    Debug.Log("L:"+lastNode.position.x);

                    Road road = new Road(currNode, lastNode);
                    roads.Add(road);
                    roadSystem.AddRoad(road);                                                 
                }
            }
        }

        public override void Finish()
        {
            if (pts.Count < 2)
            {
                return;
            }
            drawing = false;
            lastNode = null;
            //roadSystem.FindPolygon();
           // Debug.Log("polygon count=" + roadSystem.polygons.Count);

          //DestryAllMarkers();
 
      
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
            if (pts.Count>0 && drawing)
            {
                MouseHover = Utility.MouseToWorld(workPlane);
                dynamic = new Polyline();
                List<Vector3> pip = new List<Vector3>();

                pip.Add(pts[pts.Count - 1]);
                pip.Add(MouseHover);
                dynamic.vertices = pip.ToArray();
                IsNext = IsN();              
            }
            
        }
        public bool IsN()
        {
            int count = 0;
            foreach (var item in roadSystem.intersections)
            {
                if (Utility.IsClose(MouseHover, item.position))
                {
                    count++;
                }
            }

            if (count == 0)
            {
                return false;
            }
            else
            {
                return true;                                        
            }        
        }

        public override void OnRenderObject()
        {  
 
            //动态线
            if (dynamic!=null && drawing)
            {
                if (IsNext)
                {
                    SGGeometry.GLRender.Polyline(dynamic.vertices, false, null, Color.red);
                }
                else
                    SGGeometry.GLRender.Polyline(dynamic.vertices, false, null, Color.yellow);

            }


        }

        //给定一个点，找到所有包含该点的向量的 另一端点按顺时针组成的数组
        private Vector3[] FindPoints(Vector3 point,Vector3 LastPoint)
        {
     
            List<Vector3> TargetPoints = new List<Vector3>();
            //List<Node> children = new List<Node>();
           
            foreach (var item in roadSystem.roads)
            {
                if (item.centerLine.vertices[0] == point && item.centerLine.vertices[0] != LastPoint)
                {
                    TargetPoints.Add(item.centerLine.vertices[1]);
                }
                else if (item.centerLine.vertices[1] == point && item.centerLine.vertices[0] != LastPoint)
                {
                    TargetPoints.Add(item.centerLine.vertices[0]);

                }

            }
          

            Vector3[] pop = Utility.ColockwiseSortVector(TargetPoints.ToArray());
            Debug.Log("关联点数量："+pop.Length);
            return pop;

                
        }


        private void CreatTree1()
        {

         
            

           
        }

        public Obj[] CreatTree()
        {
            Obj[] bc = new Obj[roadSystem.intersections.Count];
            for (int i = 0; i < roadSystem.intersections.Count; i++)
            {
                bc[i] = new Obj();
                bc[i].pos = roadSystem.intersections[i].position;
            }

            //make tree
            for (int i = 0; i < bc.Length; i++)
            {
                Vector3[] pop;
                if (i > 0)
                {
                    pop = FindPoints(bc[i].pos, bc[i - 1].pos);
                }
                else
                {
                    pop = FindPoints(bc[0].pos, bc[0].pos);
                }

                List<int> index = new List<int>();
                foreach (var item in pop)
                {
                    for (int j = 0; j < bc.Length; j++)
                    {
                        if (bc[j].pos == item)
                        {
                            index.Add(j);
                        }
                    }
                }

                foreach (var item in index)
                {
                    Debug.Log(bc[item].pos.x);
                    bc[i].Add(bc[item]);

                }

            }


            //bc[0].Add(bc[1]);
            //bc[0].Add(bc[2]);
            //bc[0].Add(bc[4]);

            //bc[1].Add(bc[3]);

            //bc[3].Add(bc[4]);
            //bc[3].Add(bc[2]);

            //bc[2].Add(bc[4]);
            //bc[2].Add(bc[0]);

            //bc[2].Add(bc[3]);
            //bc[2].Add(bc[4]);

            //bc[4].Add(bc[2]);
            //bc[4].Add(bc[3]);





            return bc;
        }

        public void Console()
        {
            Obj[] bc = CreatTree();

            List<List<Vector3>> polygons = new List<List<Vector3>>();
            bc[0].traverse2(bc[0].pos, polygons, null, true);

            Debug.Log("polys found:" + polygons.Count);

            foreach (List<Vector3> item in polygons)
            {
                Debug.Log("\n");
                foreach (Vector3 index in item)
                {
                    Debug.Log(index.x + ",");
                }
            }
            
        }


    }


    public class Obj
    {
        List<Obj> children;
        public Vector3 pos;

        public Obj()
        {
            children = new List<Obj>();
        }

        public void traverse(Vector3 initID, bool first = true)
        {

           //Debug.Log(string.Format("id={0}, iniID={1}, first={2}", id, initID, first));
            if (pos == initID && first == false) return;

            foreach (Obj o in children)
            {
                o.traverse(initID, false);
            }

        }
        int counter = 0;
        public void traverse2(Vector3 initID, List<List<Vector3>> polygons, List<Vector3> history = null, bool first = false)
        {

            counter++;
            if (counter > 20) return;

            //建立历史
            if (history == null) history = new List<Vector3>();
            if (history.Contains(pos))
            {
                int index = history.IndexOf(pos);
                history = history.GetRange(index, history.Count - index);
                history.Add(pos);
                if (history.Count > 3)
                    polygons.Add(history.ToList());
                return;
            }

            //记录历史
            history.Add(pos);

            Debug.Log(string.Format("id={0}, iniID={1}, first={2}", pos, initID, first));
            //结束条件
            if ((pos == initID && first == false) || children.Count == 0)
            {
                return;
            }

            //遍历子树
            foreach (Obj o in children)
            {
                o.traverse2(initID, polygons, history.ToList(), false);
            }

        }
        public void Add(Obj o)
        {
            children.Add(o);
        }
    }

}

