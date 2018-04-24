using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Utility
{
    //检测两点是否靠近
    public static bool IsClose(Vector3 P1, Vector3 P2, float r = 0.015f)
    {
        Vector2 p1 = Camera.main.WorldToViewportPoint(P1);
        Vector2 p2 = Camera.main.WorldToViewportPoint(P2);
        if ((p1.x - p2.x < r && p1.x - p2.x > -r) && (p1.y - p2.y < r && p1.y - p2.y > -r))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //将鼠标点击位置 转换成世界坐标
    public static Vector3 MouseToWorld(Plane workPlane)
    {
        float d;
        Ray r = Camera.main.ScreenPointToRay(Input.mousePosition);
        workPlane.Raycast(r, out d);
        Vector3 xp = r.GetPoint(d);
        return xp;
    }

    //向量偏移
    public static List<Vector3[]> CountVectorDeviation(Vector3 p1, Vector3 p2, float width)
    {
        float dis = Vector3.Distance(p1, p2);
        float a = p2.z - p1.z;
        float b = p2.x - p1.x;
        float c = Math.Abs(width * (a / dis));
        float d = Math.Abs(width * (b / dis));
        Vector3 P1, P2, P3, P4;
        List<Vector3[]> V = new List<Vector3[]>();
        if ((a > 0 && b > 0) || (a < 0 && b < 0))
        {
            //下方
            P1 = new Vector3(p1.x + c, 0, p1.z - d);
            P2 = new Vector3(p2.x + c, 0, p2.z - d);
            //上方
            P3 = new Vector3(p1.x - c, 0, p1.z + d);
            P4 = new Vector3(p2.x - c, 0, p2.z + d);

            V.Add(new Vector3[] { P1, P2 });
            V.Add(new Vector3[] { P3, P4 });
        }
        else if ((a > 0 && b < 0) || (a < 0 && b > 0))
        {
            //下方
            P1 = new Vector3(p1.x - c, 0, p1.z - d);
            P2 = new Vector3(p2.x - c, 0, p2.z - d);
            //上方
            P3 = new Vector3(p1.x + c, 0, p1.z + d);
            P4 = new Vector3(p2.x + c, 0, p2.z + d);

            V.Add(new Vector3[] { P1, P2 });
            V.Add(new Vector3[] { P3, P4 });
        }

        return V;
    }


    //对同一个点发出的多个向量进行顺时针排序
    public static Vector3[] ColockwiseSortVector(Vector3[] vector3s)
    {
        List<Vector3> area1 = new List<Vector3>();
        List<Vector3> area2 = new List<Vector3>();

        foreach (var item in vector3s)
        {
            if (item.x >= 0)
            {
                area1.Add(item);
            }
            else if (item.x < 0)
            {
                area2.Add(item);
            }
        }

        Vector3 ReferenceVector = new Vector3(0, 0, 1);
        Dictionary<float, Vector3> dic = new Dictionary<float, Vector3>();

        float[] idex1 = new float[area1.Count];
        for (int i = 0; i < area1.Count; i++)
        {
            idex1[i] = Vector3.Dot(area1[i].normalized, ReferenceVector);
            float angle = Mathf.Acos(idex1[i]) * Mathf.Rad2Deg;
            dic.Add(angle, area1[i]);

        }

        float[] idex2 = new float[area2.Count];
        for (int i = 0; i < area2.Count; i++)
        {
            idex2[i] = Vector3.Dot(area2[i].normalized, ReferenceVector);
            float angle = 360 - Mathf.Acos(idex2[i]) * Mathf.Rad2Deg;
            dic.Add(angle, area2[i]);
        }

        List<float> array = new List<float>();
        foreach (var item in dic)
        {
            array.Add(item.Key);
        }

        float[] NewArray = ArraySorting(array.ToArray());
        Vector3[] NewVector = new Vector3[NewArray.Length];
        for (int i = 0; i < NewArray.Length; i++)
        {
           //Debug.Log(NewArray[i]);
            NewVector[i] = dic[NewArray[i]];
        }

        return NewVector;
    }


    //数组排序（升序）
    public static float[] ArraySorting(float[] array)
    {
        for (int i = 0; i < array.Length; i++)
        {
            for (int j = i + 1; j < array.Length; j++)
            {
                if (array[i] > array[j])
                {
                    float id = array[i];
                    array[i] = array[j];
                    array[j] = id;
                }
            }
        }

        return array;

    }
}
