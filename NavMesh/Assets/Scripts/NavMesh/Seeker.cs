


using UnityEngine;
using System;
using System.Collections.Generic;

//  Seeker.cs
//  Author: Lu Zexi
//  2013-10-05



namespace Game.NavMesh
{

    public enum PathResCode
    {
        Success = 0,    //寻路成功
        Failed = -1,    //寻路失败
        NoMeshData = -2,    //没有数据
        NoStartTriOrEndTri = -3,    //没有起始点或终点
        NavIDNotMatch = -4, //导航网格的索引和id不匹配
        NotFoundPath = -5,  //找不到路径
        CanNotGetNextWayPoint = -6,//找不到下一个拐点信息
        GroupNotMatch, //起点和中点在不同的孤岛之间，无法到达
        NoCrossPoint,
        FixPointFailed, //修复点失败
    }

    /// <summary>
    /// 寻路者
    /// </summary>
    public class Seeker
    {
        private const int START_POS_EXTEND_LENGTH = 10; //起始位置容错长度
        private const int END_POS_EXTEND_LENGTH = 2;    //终点位置容错长度

        private List<NavTriangle> m_lstTriangle;    //地图数据
		public List<NavTriangle> NavMeshData
		{
			set{ this.m_lstTriangle = value;}
		}

        private static Seeker s_cInstance;  //静态实例

        public Seeker()
        {
            this.m_lstTriangle = new List<NavTriangle>();
        }

        /// <summary>
        /// 获取静态实例
        /// </summary>
        /// <returns></returns>
        public static Seeker GetInstance()
        {
            if (s_cInstance == null)
                s_cInstance = new Seeker();
            return s_cInstance;
        }

        /// <summary>
        /// 寻路
        /// </summary>
        /// <param name="strPos">起始位置</param>
        /// <param name="endPos">终点位置</param>
        /// <param name="path">输出路径点</param>
        /// <param name="offset">移动物体大小</param>
        /// <returns>寻路结果</returns>
        public PathResCode Seek(Vector2 startPos, Vector2 endPos, out List<Vector2> path, int offset)
        {
            path = new List<Vector2>();

            if (this.m_lstTriangle == null || this.m_lstTriangle.Count <= 0)
                return PathResCode.NoMeshData;

            ResetData();


            List<NavTriangle> pathTri;
            PathResCode res = SeekTrianglePath(ref startPos, ref endPos, out pathTri, offset);
            if (res != PathResCode.Success)
                return res;

			res = CreateWayPoints(startPos, endPos, pathTri, out path, offset);
            if (res != PathResCode.Success)
                return res;


            return res;
        }

        /// <summary>
        /// 寻路路径三角形
        /// </summary>
        /// <param name="strPos">起始点</param>
        /// <param name="endPos">终点</param>
        /// <param name="pathTriangle">输出三角形</param>
        /// <param name="offset">移动物品大小</param>
        /// <returns>结果</returns>
        private PathResCode SeekTrianglePath(ref Vector2 startPos, ref Vector2 endPos, out List<NavTriangle> pathList, int offset)
        {
            pathList = new List<NavTriangle>();
            NavTriangle startTri = null, endTri = null;

            //获得起始与终点三角形
            foreach (NavTriangle navTri in this.m_lstTriangle)
            {
                if (startTri == null)
                    if (navTri.IsPointIn(startPos))
                        startTri = navTri;

                if (endTri == null)
                    if (navTri.IsPointIn(endPos))
                        endTri = navTri;

                if (startTri != null && endTri != null)
                    break;
            }

            //检查和修复位置
            PathResCode posErr = CheckAndFixPos(ref startTri, ref startPos, ref endTri, ref endPos);
            if (posErr != PathResCode.Success)
                return posErr;

            //////////////////////////////////// A*算法 ///////////////////////////////////////

            int pathSessionId = 1;
            bool foundPath = false;
            List<NavTriangle> openList = new List<NavTriangle>();   //开放列表
            List<NavTriangle> closeList = new List<NavTriangle>();

            startTri.SetSessionID(pathSessionId);

            openList.Add(startTri);
            while (openList.Count > 0)
            {
                // 1. 把当前节点从开放列表删除, 加入到封闭列表
                NavTriangle currNode;
                currNode = openList[openList.Count - 1];
                openList.Remove(currNode);
                closeList.Add(currNode);

                //已经找到目的地
                if (currNode.GetID() == endTri.GetID())
                {
                    foundPath = true;
                    break;
                }

                // 2. 对当前节点相邻的每一个节点依次执行以下步骤:
                // 遍历所有邻接三角型
                for (int i = 0; i < 3; i++)
                {
                    int neighborID = currNode.GetNeighbor(i);
                    NavTriangle neighborTri;

                    // 3. 如果该相邻节点不可通行,则什么操作也不执行,继续检验下一个节点;
                    if (neighborID < 0)
                    {
                        //没有该邻居节点
                        continue;
                    }
                    else
                    {
                        neighborTri = this.m_lstTriangle[neighborID];

                        if (neighborTri == null || neighborTri.GetID() != neighborID)
                            return PathResCode.NavIDNotMatch;
                    }
                    if (neighborTri.GetGroupID() == startTri.GetGroupID() )
                    {
                        if (neighborTri.GetSessionID() != pathSessionId)
                        {
                            // 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
                            //    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
                            neighborTri.SetSessionID(pathSessionId);
                            neighborTri.SetParentID(currNode.GetID());
                            neighborTri.SetOpen(true);

                            // 计算启发值h
                            neighborTri.CalcHeuristic(endPos);
                            // 计算三角形花费g
                            neighborTri.SetGValue(currNode.GetGValue() + currNode.GetCost(neighborTri.GetID()) );

                            //放入开放列表并排序
                            openList.Add(neighborTri);
                            openList.Sort(CompareTriWithGValue);

                            //保存穿入边
                            neighborTri.SetArrivalWall(currNode.GetID());
                        }
                        else
                        {
                            // 5. 如果该相邻节点在开放列表中, 
                            //    则判断若经由当前节点到达该相邻节点的G值是否小于原来保存的G值,
                            //    若小于,则将该相邻节点的父节点设为当前节点,并重新设置该相邻节点的G和F值
                            if (neighborTri.GetOpen())
                            {
                                if (neighborTri.GetGValue() + neighborTri.GetCost(currNode.GetID()) < currNode.GetGValue())
                                {
                                    currNode.SetGValue(neighborTri.GetGValue() + neighborTri.GetCost(currNode.GetID()));
                                    currNode.SetParentID(neighborTri.GetID());
                                    currNode.SetArrivalWall(neighborTri.GetID());
                                }
                            }
                            else
                            {
                                neighborTri = null;
                                continue;
                            }

                        }
                    }
                }
            }

            if (closeList.Count != 0)
            {
                NavTriangle path = closeList[closeList.Count - 1];
                pathList.Add(path);
                while (path.GetParentID() != -1)
                {
                    pathList.Add(this.m_lstTriangle[path.GetParentID()]);
                    path = this.m_lstTriangle[path.GetParentID()];
                }
            }

            if (!foundPath)
                return PathResCode.NotFoundPath;
            else
                return PathResCode.Success;
        }

        /// <summary>
        /// 根据f和h实现排序，A*算法
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        private int CompareTriWithGValue(NavTriangle x, NavTriangle y)
        {
            double xFvalue = x.GetHValue() /*+ x.GValue*/;
            double yFvalue = y.GetHValue() /*+ y.GValue*/;

            if (xFvalue == yFvalue)
                return 0;
            else if (xFvalue < yFvalue)
                return 1;
            else
                return -1;
        }

        /// <summary>
        /// 检查和修复所有错误路径点
        /// </summary>
        /// <param name="sTri"></param>
        /// <param name="startPos"></param>
        /// <param name="eTri"></param>
        /// <param name="endPos"></param>
        /// <returns></returns>
        private PathResCode CheckAndFixPos(ref NavTriangle startTri, ref Vector2 startPos, ref NavTriangle endTri, ref Vector2 endPos)
        {
            if ( startTri != null && endTri != null && startTri.GetGroupID() != endTri.GetGroupID())
                return PathResCode.GroupNotMatch;

            if (endTri == null)
            {
                if (FixPos(ref endTri, ref endPos, startTri, startPos, END_POS_EXTEND_LENGTH) != PathResCode.Success)
                {
                    return PathResCode.Failed;
                }
            }

            if (startTri == null)
            {
                PathResCode fixRet = FixPos(ref startTri, ref startPos, endTri, endPos, START_POS_EXTEND_LENGTH);
            }

            if (startTri == null || endTri == null)
                return PathResCode.Failed;

            if (startTri.GetGroupID() != endTri.GetGroupID())
                return PathResCode.GroupNotMatch;

            return PathResCode.Success;
        }

        /// <summary>
        /// 根据原始点计算出正确的落在导航区内的位置,修复无效点和三角形
        /// </summary>
        /// <param name="orgTarTri">起源三角形</param>
        /// <param name="orgTarPos">起源点</param>
        /// <param name="otherTri">参考方向三角形</param>
        /// <param name="otherPos">参考方向点</param>
        /// <returns>结果</returns>
        private PathResCode FixPos(ref NavTriangle orgTarTri, ref Vector2 orgTarPos, NavTriangle otherTri, Vector2 otherPos, int extendLength)
        {
            Vector2 tarPos = orgTarPos;
            //////////////////////////////////////////////////////////////////////////
            //为了精确判断,需要逆向延长线段一定长度
            if (extendLength > 0)
            {
                Vector2 newTarPos = NMath.ExtendPos(otherPos, tarPos, extendLength);
                tarPos = newTarPos;
            }
            //////////////////////////////////////////////////////////////////////////

            Line2D linePath = new Line2D(tarPos, otherPos); //参考线段
            Rect lineRect = NMath.LineRect(linePath);   //获取线段矩形包围盒

            //1)找到所有与参考线段矩形相交的三角形,并判断是否groupID相等
            List<NavTriangle> crossNavTris = new List<NavTriangle>();
            foreach (NavTriangle tri in this.m_lstTriangle)
            {
                if (NMath.CheckCross(lineRect, tri.GetBoxCollider()))
                {
                    if (otherTri != null && otherTri.GetGroupID() != tri.GetGroupID())
                        continue;
                    crossNavTris.Add(tri);
                }
            }

            //2)找出所有与参考线段相交的三角形,并记录相交点
            List<Vector2> crossPoints = new List<Vector2>();    //相交点列表
            List<int> triIndex = new List<int>();   //相交三角形索引列表
            for (int index = 0; index < crossNavTris.Count; index++)
            {
                NavTriangle crossTri = crossNavTris[index];
                Line2D triLine;
                for (int i = 0; i < 3; i++)
                {
                    Vector2 insPoint;
                    triLine = new Line2D(crossTri.GetPoint(i), crossTri.GetPoint((i + 1)%3));
                    if (linePath.Intersection(triLine, out insPoint) == LineCrossState.CROSS)
                    {
                        crossPoints.Add(insPoint);
                        triIndex.Add(index);
                    }
                }
            }

            if (crossPoints.Count == 0)
                return PathResCode.NoCrossPoint;


            //3)找到最接近起源点的点
            Vector2 lastPos = crossPoints[0];
            int lastTriIndex = triIndex[0];
            double lastLength = Math.Pow(lastPos.x - orgTarPos.x, 2.0) + Math.Pow(lastPos.y - orgTarPos.y, 2.0);
            for (int i = 1; i < crossPoints.Count; i++)
            {
                double newLength = Math.Pow(crossPoints[i].x - orgTarPos.x, 2.0) + Math.Pow(crossPoints[i].y - orgTarPos.y, 2.0);
                if (newLength < lastLength)
                {
                    lastPos = crossPoints[i];
                    lastTriIndex = triIndex[i];
                    lastLength = newLength;
                }
            }

            //4)保存目标
            orgTarPos = lastPos;
            orgTarTri = crossNavTris[lastTriIndex];

            return PathResCode.Success;
        }


        /// <summary>
        /// 生成最终的路径点
        /// </summary>
        /// <param name="startPos">起始点</param>
        /// <param name="endPos">终点</param>
        /// <param name="triPathList">三角形路径列表</param>
        /// <param name="wayPoints">路径点</param>
        /// <param name="offSet">移动物体宽度</param>
        /// <returns></returns>
        private PathResCode CreateWayPoints(Vector2 startPos, Vector2 endPos
            , List<NavTriangle> triPathList, out List<Vector2> wayPoints, int offSet)
        {
            wayPoints = new List<Vector2>();
            if (triPathList.Count == 0 || startPos == null || endPos == null)
                return PathResCode.Failed;

            // 保证从起点到终点的顺序
            triPathList.Reverse();

            // 保存出边编号
            for (int i = 0; i < triPathList.Count; i++)
            {
                NavTriangle tri = triPathList[i];
                if (i != triPathList.Count - 1)
                {
                    NavTriangle nextTri = triPathList[i + 1];
                    tri.SetOutWallIndex(tri.GetWallIndex(nextTri.GetID()));
                }
            }

            wayPoints.Add(startPos);

            //起点与终点在同一三角形中
            if (triPathList.Count == 1)
            {
                wayPoints.Add(endPos);
                return PathResCode.Success;
            }

            WayPoint way = new WayPoint(startPos, triPathList[0]);
            while (!NMath.IsEqualZero(way.GetPoint() - endPos))
            {
                way = GetFurthestWayPoint(way, triPathList, endPos, offSet);
                if (way == null)
                    return PathResCode.CanNotGetNextWayPoint;
                wayPoints.Add(way.GetPoint());
            }

            return PathResCode.Success;
        }

        /// <summary>
        /// 根据拐点计算法获得导航网格的下一个拐点
        /// </summary>
        /// <param name="way"></param>
        /// <param name="triPathList"></param>
        /// <param name="endPos"></param>
        /// <param name="offSet"></param>
        /// <returns></returns>
        private WayPoint GetFurthestWayPoint(WayPoint way, List<NavTriangle> triPathList, Vector2 endPos, int offSet)
        {
            WayPoint nextWay = null;
            Vector2 currPnt = way.GetPoint();
            NavTriangle currTri = way.GetTriangle();
            NavTriangle lastTriA = currTri;
            NavTriangle lastTriB = currTri;
            int startIndex = triPathList.IndexOf(currTri);//开始路点所在的网格索引
            Line2D outSide = currTri.GetSide(currTri.GetOutWallIndex());//路径线在网格中的穿出边?
            Vector2 lastPntA = outSide.GetStartPoint();
            Vector2 lastPntB = outSide.GetEndPoint();
            Line2D lastLineA = new Line2D(currPnt, lastPntA);
            Line2D lastLineB = new Line2D(currPnt, lastPntB);
            Vector2 testPntA, testPntB;

            for (int i = startIndex + 1; i < triPathList.Count; i++)
            {
                currTri = triPathList[i];
                outSide = currTri.GetSide(currTri.GetOutWallIndex());
                if (i == triPathList.Count - 1)
                {
                    testPntA = endPos;
                    testPntB = endPos;
                }
                else
                {
                    testPntA = outSide.GetStartPoint();
                    testPntB = outSide.GetEndPoint();
                }

                if (lastPntA != testPntA)
                {
                    if (lastLineB.ClassifyPoint(testPntA) == PointSide.RIGHT_SIDE)
                    {
                        nextWay = new WayPoint(lastPntB, lastTriB);
                        return nextWay;
                    }
                    else if (lastLineA.ClassifyPoint(testPntA) != PointSide.LEFT_SIDE)
                    {
                        lastPntA = testPntA;
                        lastTriA = currTri;
                        //重设直线
                        lastLineA = new Line2D(lastLineA.GetStartPoint(), lastPntA);
                    }
                }

                if (lastPntB != testPntB)
                {
                    if (lastLineA.ClassifyPoint(testPntB) == PointSide.LEFT_SIDE)
                    {
                        nextWay = new WayPoint(lastPntA, lastTriA);
                        return nextWay;
                    }
                    else if (lastLineB.ClassifyPoint(testPntB) != PointSide.RIGHT_SIDE)
                    {
                        lastPntB = testPntB;
                        lastTriB = currTri;
                        //重设直线
                        lastLineB = new Line2D(lastLineB.GetStartPoint(), lastPntB);
                    }
                }
            }

            //到达终点
            nextWay = new WayPoint(endPos, triPathList[triPathList.Count - 1]);

            return nextWay;
        }

        /// <summary>
        /// 重置寻路数据
        /// </summary>
        private void ResetData()
        {
            foreach (NavTriangle item in this.m_lstTriangle)
            {
                item.Reset();
            }
        }


    }

}

