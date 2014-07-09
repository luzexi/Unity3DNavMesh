using System;
using System.Collections.Generic;
using UnityEngine;




namespace Game.NavMesh
{
	public class NavNode
    {
        public Vector2 vertex;  //顶点
        public bool passed;     //是否被访问过
        public bool isMain;     //是否主多边形顶点
        public bool o;          //是否输出点
        public bool isIns;      //是否交点
        public NavNode other;   //交点用，另个多边形上的节点
        public NavNode next;    //后面一个点

        public NavNode(Vector2 point, bool isin, bool bMain)
        {
            vertex = point;
            isIns = isin;
            isMain = bMain;
            passed = false;
            o = false;
        }
    }
}
