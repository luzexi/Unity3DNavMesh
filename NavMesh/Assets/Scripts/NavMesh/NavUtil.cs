using System;
using System.Collections.Generic;
using UnityEngine;


namespace Game.NavMesh
{
	public class NavUtil
	{
        /// <summary>
        /// 用于合并传进来的多边形数组，返回合并后的多边形数组，
        /// 如果生成了孤岛，则孤岛的tag标志递增
        /// </summary>
        /// <param name="polys"></param>
        /// <returns></returns>
        public static PolyResCode UnionAllPolygon(ref List<Polygon> polys)
        {
            int tag = 1;

            for (int i = 0; i < polys.Count; i++)
                polys[i].CW();

            for (int i = 1; i < polys.Count; i++)
            {
                Polygon p1 = polys[i];
                for (int j = 1; j < polys.Count; j++)
                {
                    Polygon p2 = polys[j];
                    if ( i!=j && !p1.Equals(p2))
                    {
                        List<Polygon> polyResult = new List<Polygon>();
                        PolyResCode result = p1.Union(p2, ref polyResult);

                        if (result == PolyResCode.Success && polyResult.Count > 0)
                        {
                            polys.Remove(p1);
                            polys.Remove(p2);

                            for (int k = 0; k < polyResult.Count; k++)
                            {
                                Polygon poly = polyResult[k];
                                if (/*polyResult.Count > 1 &&*/ !poly.IsCW())
                                    poly.SetTag(tag++);//如果逆时针说明这个多边形是孤岛

                                polys.Add(poly);
                            }
                            i = -1;
                            break;
                        }
                    }
                }
            }
            return PolyResCode.Success;
        }

        /// <summary>
        /// 合并两个节点列表，生成交点，并按顺时针序插入到顶点表中
        /// </summary>
        /// <param name="c0">主多边形顶点表，并返回插入交点后的顶点表</param>
        /// <param name="c1">合并多边形顶点表，并返回插入交点后的顶点表</param>
        /// <param name="nInsCnt">交点个数</param>
        /// <returns></returns>
        public static PolyResCode NavNodeIntersectPoint(List<NavNode> c0, List<NavNode> c1, out int nInsCnt)
        {
            nInsCnt = 0;

            NavNode startNode0 = c0[0];
            NavNode startNode1 = null;
            Line2D line0, line1;
            Vector2 insPoint;
            bool hasIns = false;

            while (startNode0 != null)
            {
                // 判断是否到末点了
                if (startNode0.next == null)
                    line0 = new Line2D(startNode0.vertex, c0[0].vertex);
                else
                    line0 = new Line2D(startNode0.vertex, startNode0.next.vertex);

                startNode1 = c1[0];
                hasIns = false;

                while (startNode1 != null)
                {
                    if (startNode1.next == null)
                        line1 = new Line2D(startNode1.vertex, c1[0].vertex);
                    else
                        line1 = new Line2D(startNode1.vertex, startNode1.next.vertex);

                    if (line0.Intersection(line1, out insPoint) == LineCrossState.CROSS)
                    {
                        int insPotIndex = -1;
                        //如果交点不在多边形的节点上
                        if (NavUtil.NavNodeGetNodeIndex(c0, insPoint, out insPotIndex) == PolyResCode.ErrNotInside)
                        {
                            nInsCnt++;
                            NavNode node0 = new NavNode(insPoint, true, true);
                            NavNode node1 = new NavNode(insPoint, true, false);

                            c0.Add(node0);
                            c1.Add(node1);

                            node0.other = node1;
                            node1.other = node0;

                            //插入顶点列表
                            node0.next = startNode0.next;
                            startNode0.next = node0;
                            node1.next = startNode1.next;
                            startNode1.next = node1;

                            if (line0.ClassifyPoint(line1.GetEndPoint()) == PointSide.RIGHT_SIDE)
                            {
                                node0.o = true;
                                node1.o = true;
                            }

                            hasIns = true;
                            break;
                        }
                    }
                    startNode1 = startNode1.next;

                }
                if (!hasIns)
                    startNode0 = startNode0.next;

            }

            return PolyResCode.Success;
        }

        /// <summary>
        /// 查找point是否在节点列表里面
        /// </summary>
        /// <param name="nodeList">节点列表</param>
        /// <param name="point">用于查找的节点</param>
        /// <param name="pIndex">返回节点索引</param>
        /// <returns>if inside,return sucess,else return not inside</returns>
        public static PolyResCode NavNodeGetNodeIndex(List<NavNode> nodeList, Vector2 point, out int pIndex)
        {
            pIndex = -1;
            for (int i = 0; i < nodeList.Count; i++)
            {
                NavNode node = nodeList[i];
                if (NMath.Equals(node.vertex, point))
                {
                    pIndex = i;
                    return PolyResCode.Success;
                }
            }
            return PolyResCode.ErrNotInside;
        }

        /// <summary>
        /// 合并两个节点列表为一个多边形，结果为顺时针序( 生成的内部孔洞多边形为逆时针序)
        /// </summary>
        /// <param name="mainNode"></param>
        /// <param name="subNode"></param>
        /// <param name="polyRes"></param>
        /// <returns></returns>
        public static PolyResCode LinkToPolygon(List<NavNode> mainNode, List<NavNode> subNode, ref List<Polygon> polyRes)
        {
            polyRes.Clear();
            for (int i = 0; i < mainNode.Count; i++)
            {
                NavNode currNode = mainNode[i];

                // 选择一个没有访问过的交点做起始点
                if (currNode.isIns && !currNode.passed)
                {
                    List<Vector2> points = new List<Vector2>();
                    while (currNode != null)
                    {
                        currNode.passed = true;

                        //交点转换
                        if (currNode.isIns)
                        {
                            currNode.other.passed = true;

                            if (!currNode.o)//该交点为进点（跟踪裁剪多边形边界）
                            {
                                if (currNode.isMain)//当前点在主多边形中
                                    currNode = currNode.other;//切换到裁剪多边形中
                            }
                            else
                            {
                                //该交点为出点（跟踪主多边形边界）
                                if (!currNode.isMain)//当前点在裁剪多边形中
                                    currNode = currNode.other;//切换到主多边形中
                            }
                        }

                        points.Add(currNode.vertex);

                        if (currNode.next == null)
                        {
                            if (currNode.isMain)
                                currNode = mainNode[0];
                            else
                                currNode = subNode[0];
                        }
                        else
                            currNode = currNode.next;

                        if (currNode.vertex == points[0])
                            break;
                    }

                    // 删除重复顶点
                    Polygon poly = new Polygon(points);
                    poly.DelRepeatPoint();
                    polyRes.Add(poly);
                }
            }
            return PolyResCode.Success;
        }
	}
}
