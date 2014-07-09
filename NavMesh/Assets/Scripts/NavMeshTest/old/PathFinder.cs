/********************************************************************
	created:	2011/12/19
	created:	19:12:2011   13:30
	filename: 	SGWeb\DependProjects\NavMesh\PathFinder.cs
	file path:	SGWeb\DependProjects\NavMesh
	file base:	PathFinder
	file ext:	cs
	author:		Ivan
	
	purpose:	这个类只用来寻路，使用生成后的导航网格数据
                请加载NavMeshTestProject项目来运行单元测试，保证方法的正确。
                默认开启容错机制，如果目标点或者当前点在不可行走区域，会
                自动查找附近的可行走点
*********************************************************************/
using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace NavMesh
{
    public sealed class PathFinder
    {
        // 更改为单例模式
        static readonly PathFinder instance = new PathFinder();

        static PathFinder() { }

        public static PathFinder Instance
        {
            get
            {
                return instance;
            }
        }

        // 导航网格数据
        public List<Triangle> NavMeshData { get; set; }

        // 增加容错机制，允许玩家点击到不可行走区域，系统会自动行走到最近的地点
        private bool AllowUnwalkPos { get; set; }

        /// <summary>
        /// 开启容错,支持点击到不可行走区域
        /// </summary>
        public void TurnOnAllowUnwalk() { AllowUnwalkPos = true; }
        /// <summary>
        /// 关闭容错,支持点击到不可行走区域
        /// </summary>
        public void TurnOffAllowUnwalk() { AllowUnwalkPos = false; }

        public PathFinder()
        {
            InitData();
        }

        /// <summary>
        /// init original data
        /// </summary>
        private void InitData()
        {
            NavMeshData = null;
            TurnOnAllowUnwalk();
        }

        /// <summary>
        /// 对指定点寻路
        /// </summary>
        /// <param name="startPos"></param>
        /// <param name="endPos"></param>
        /// <param name="wayPoints">返回节点列表</param>
        /// <param name="offSet">移动物体宽度</param>
        /// <returns></returns>
        public PathResCode FindPath(Vector2 startPos, Vector2 endPos
            , out List<Vector2> wayPoints, int offSet)
        {
            wayPoints = new List<Vector2>();
            if (NavMeshData == null)
                return PathResCode.NoMeshData;

            // 由于重复使用导航网格，需要重置数据
            ResetMeshData();

            List<Triangle> triPathList;
            PathResCode findResult = FindTriPath(ref startPos, ref endPos, NavMeshData, out triPathList);
            if (findResult != PathResCode.Success)
                return findResult;

            PathResCode CreateWayRes = CreateWayPoints(startPos, endPos, triPathList, out wayPoints, offSet);
            if (CreateWayRes != PathResCode.Success)
                return CreateWayRes;

            return PathResCode.Success;
        }


        /// <summary>
        /// 根据f和h实现排序，A*算法
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        private static int CompareTriWithGValue(Triangle x, Triangle y)
        {
            double xFvalue = x.HValue /*+ x.GValue*/;
            double yFvalue = y.HValue /*+ y.GValue*/;

            if (xFvalue == yFvalue)
                return 0;
            else if (xFvalue < yFvalue)
                return 1;
            else
                return -1;
        }

        /// <summary>
        /// 查找三角形路径
        /// </summary>
        /// <param name="startPos"></param>
        /// <param name="endPos"></param>
        /// <param name="navMeshData"></param>
        /// <param name="triPathList"></param>
        /// <returns></returns>
        private PathResCode FindTriPath(ref Vector2 startPos, ref Vector2 endPos
           , List<Triangle> navMeshData, out List<Triangle> pathList)
        {
            pathList = new List<Triangle>();
            Triangle sTri = null, eTri = null;
            // 先获得起始三角形
            foreach (Triangle navTri in navMeshData)
            {
                if (sTri == null)
                    if (navTri.IsPointIn(startPos))
                        sTri = navTri;

                if (eTri == null)
                    if (navTri.IsPointIn(endPos))
                        eTri = navTri;

                if (sTri != null && eTri != null)
                    break;
            }

            // 检查和修复位置
            PathResCode posErr = CheckAndFixPos(ref sTri, ref startPos, ref eTri, ref endPos);
            if (posErr != PathResCode.Success)
                return posErr;

            int pathSessionId = 1;
            bool foundPath = false;
            Triangle currNode;
            Triangle adjacentTmp;
            List<Triangle> openList = new List<Triangle>();
            List<Triangle> closeList = new List<Triangle>();

            sTri.SessionID = pathSessionId;
            openList.Add(sTri);

            while (openList.Count > 0)
            {
                // 1. 把当前节点从开放列表删除, 加入到封闭列表
                currNode = openList[openList.Count - 1];
                openList.Remove(currNode);
                closeList.Add(currNode);

                // 已经找到目的地
                if (currNode.ID == eTri.ID)
                {
                    foundPath = true;
                    break;
                }
                // 2. 对当前节点相邻的每一个节点依次执行以下步骤:
                // 遍历所有邻接三角型
                int adjacentId;
                for (int i = 0; i < 3; i++)
                {
                    adjacentId = currNode.Neighbors[i];

                    // 3. 如果该相邻节点不可通行,则什么操作也不执行,继续检验下一个节点;
                    if (adjacentId < 0)
                        continue;
                    else
                    {
                        adjacentTmp = navMeshData[adjacentId];

                        if (adjacentTmp == null || adjacentTmp.ID != adjacentId)
                            return PathResCode.NavIdNotMatch;
                    }

                    if (adjacentTmp.Group == sTri.Group)
                    {
                        if (adjacentTmp.SessionID != pathSessionId)
                        {
                            // 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
                            //    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
                            adjacentTmp.SessionID = pathSessionId;
                            adjacentTmp.ParentId = currNode.ID;
                            adjacentTmp.IsOpen = true;

                            // 计算启发值h
                            adjacentTmp.calcHeuristic(endPos);
                            // 计算三角形花费g
                            adjacentTmp.GValue = currNode.GValue + currNode.GetCost(adjacentTmp.ID);

                            //放入开放列表并排序
                            openList.Add(adjacentTmp);
                            openList.Sort(CompareTriWithGValue);

                            //保存穿入边
                            adjacentTmp.SetArrivalWall(currNode.ID);
                        }
                        else
                        {
                            // 5. 如果该相邻节点在开放列表中, 
                            //    则判断若经由当前节点到达该相邻节点的G值是否小于原来保存的G值,
                            //    若小于,则将该相邻节点的父节点设为当前节点,并重新设置该相邻节点的G和F值
                            if (adjacentTmp.IsOpen)
                            {
                                if (adjacentTmp.GValue + adjacentTmp.GetCost(currNode.ID) < currNode.GValue)
                                {
                                    currNode.GValue = adjacentTmp.GValue + adjacentTmp.GetCost(currNode.ID);
                                    currNode.ParentId = adjacentTmp.ID;
                                    currNode.SetArrivalWall(adjacentTmp.ID);
                                }
                            }
                            else
                            {
                                adjacentTmp = null;
                                continue;
                            }

                        }
                    }
                }
            }

            if (closeList.Count != 0)
            {
                Triangle path = closeList[closeList.Count - 1];
                pathList.Add(path);
                while (path.ParentId != -1)
                {
                    pathList.Add(navMeshData[path.ParentId]);
                    path = navMeshData[path.ParentId];
                }
            }

            if (!foundPath)
                return PathResCode.NotFoundPath;
            else
                return PathResCode.Success;
        }

        /// <summary>
        /// 检查和修复所有错误路径点
        /// </summary>
        /// <param name="sTri"></param>
        /// <param name="startPos"></param>
        /// <param name="eTri"></param>
        /// <param name="endPos"></param>
        /// <returns></returns>
        private PathResCode CheckAndFixPos(ref Triangle sTri, ref Vector2 startPos, ref Triangle eTri, ref Vector2 endPos)
        {
            //////////////////////////////////////////////////////////////////////////
            // check
            PathResCode posErr = PathResCode.Success;

            if (sTri == null || eTri == null)
                posErr = PathResCode.NoStartTriOrEndTri;
            else if (sTri.Group != eTri.Group)
                posErr = PathResCode.GroupNotMatch;

            //没有错误
            if (posErr == PathResCode.Success)
                return posErr;

            //////////////////////////////////////////////////////////////////////////
            // fix
            // 不需要修复
            if (!AllowUnwalkPos)
            {
                return posErr;
            }
            else
            {
                // 修正所有错误点
                PathResCode fixRet = FixUnwalkPos(ref sTri, ref startPos, ref eTri, ref endPos);
                if (fixRet != PathResCode.Success)
                    return fixRet;
            }

            return PathResCode.Success;
        }

        /// <summary>
        /// 修复错误位置
        /// </summary>
        /// <param name="sTri"></param>
        /// <param name="startPos"></param>
        /// <param name="eTri"></param>
        /// <param name="endPos"></param>
        /// <returns></returns>
        private PathResCode FixUnwalkPos(ref Triangle sTri, ref Vector2 startPos, ref Triangle eTri, ref Vector2 endPos)
        {
            if (eTri == null)
            {
                PathResCode fixRet = FixEndPos(ref eTri, ref endPos, sTri, startPos);
                if (fixRet != PathResCode.Success)
                    return fixRet;
            }
            if (sTri == null)
            {
                PathResCode fixRet = FixStartPos(ref sTri, ref startPos, eTri, endPos);
                if (fixRet != PathResCode.Success)
                    return fixRet;
            }
            if (sTri.Group != eTri.Group)
                return PathResCode.GroupNotMatch;

            return PathResCode.Success;
        }

        /// <summary>
        /// 修复错误的终点
        /// </summary>
        /// <param name="eTri"></param>
        /// <param name="endPos"></param>
        /// <param name="startTri"></param>
        /// <param name="startPos"></param>
        /// <returns></returns>
        private PathResCode FixEndPos(ref Triangle eTri, ref Vector2 endPos,Triangle startTri, Vector2 startPos)
        {
            if (NavMeshData == null)
                return PathResCode.NoMeshData;

            int endExtendLength = 2;
            PathResCode fixRet = GetFixPos(ref eTri, ref endPos, startTri, startPos, endExtendLength);

            return fixRet;
        }

        /// <summary>
        /// 修复错误的起点
        /// </summary>
        /// <param name="sTri"></param>
        /// <param name="startPos"></param>
        /// <param name="endTri"></param>
        /// <param name="endPos"></param>
        /// <returns></returns>
        private PathResCode FixStartPos(ref Triangle sTri, ref Vector2 startPos, Triangle endTri, Vector2 endPos)
        {
            if (NavMeshData == null)
                return PathResCode.NoMeshData;

            int startExtendLength = 10;
            PathResCode fixRet = GetFixPos(ref sTri, ref startPos, endTri, endPos, startExtendLength);

            return fixRet;
        }

        /// <summary>
        /// 生成最终的路径点
        /// </summary>
        /// <param name="startPos"></param>
        /// <param name="endPos"></param>
        /// <param name="triPathList"></param>
        /// <param name="wayPoints"></param>
        /// <param name="offSet">移动物体宽度</param>
        /// <returns></returns>
        private PathResCode CreateWayPoints(Vector2 startPos, Vector2 endPos
            , List<Triangle> triPathList, out List<Vector2> wayPoints, int offSet)
        {
            wayPoints = new List<Vector2>();
            if (triPathList.Count == 0 || startPos == null || endPos == null)
                return PathResCode.Failed;
            // 保证从起点到终点的顺序
            triPathList.Reverse();
            // 保存出边编号
            for (int i = 0; i < triPathList.Count; i++)
            {
                Triangle tri = triPathList[i];
                if (i != triPathList.Count - 1)
                {
                    Triangle nextTri = triPathList[i + 1];
                    tri.OutWallIndex = tri.GetWallIndex(nextTri.ID);
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
            while (!SGMath.IsEqualZero(way.Position - endPos))
            {
                way = GetFurthestWayPoint(way, triPathList, endPos, offSet);
                if (way == null)
                    return PathResCode.CanNotGetNextWayPoint;
                wayPoints.Add(way.Position);
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
        private WayPoint GetFurthestWayPoint(WayPoint way, List<Triangle> triPathList, Vector2 endPos, int offSet)
        {
            WayPoint nextWay = null;
            Vector2 currPnt = way.Position;
            Triangle currTri = way.Triangle;
            Triangle lastTriA = currTri;
            Triangle lastTriB = currTri;
            int startIndex = triPathList.IndexOf(currTri);//开始路点所在的网格索引
            Line2D outSide = currTri.GetSide(currTri.OutWallIndex);//路径线在网格中的穿出边?
            Vector2 lastPntA = outSide.PointStart;
            Vector2 lastPntB = outSide.PointEnd;
            Line2D lastLineA = new Line2D(currPnt, lastPntA);
            Line2D lastLineB = new Line2D(currPnt, lastPntB);
            Vector2 testPntA, testPntB;

            for (int i = startIndex + 1; i < triPathList.Count; i++)
            {
                currTri = triPathList[i];
                outSide = currTri.GetSide(currTri.OutWallIndex);
                if (i == triPathList.Count - 1)
                {
                    testPntA = endPos;
                    testPntB = endPos;
                }
                else
                {
                    testPntA = outSide.PointStart;
                    testPntB = outSide.PointEnd;
                }

                if (lastPntA != testPntA)
                {
                    if (lastLineB.classifyPoint(testPntA) == PointSide.RIGHT_SIDE)
                    {
                        nextWay = new WayPoint(lastPntB, lastTriB);
                        return nextWay;
                    }
                    else if (lastLineA.classifyPoint(testPntA) != PointSide.LEFT_SIDE)
                    {
                        lastPntA = testPntA;
                        lastTriA = currTri;
                        //重设直线
                        lastLineA.PointEnd = lastPntA;
                    }
                }

                if (lastPntB != testPntB)
                {
                    if (lastLineA.classifyPoint(testPntB) == PointSide.LEFT_SIDE)
                    {
                        nextWay = new WayPoint(lastPntA, lastTriA);
                        return nextWay;
                    }
                    else if (lastLineB.classifyPoint(testPntB) != PointSide.RIGHT_SIDE)
                    {
                        lastPntB = testPntB;
                        lastTriB = currTri;
                        //重设直线
                        lastLineB.PointEnd = lastPntB;
                    }
                }
            }

            //到达终点
            nextWay = new WayPoint(endPos, triPathList[triPathList.Count - 1]);

            return nextWay;
        }

        private void ResetMeshData()
        {
            foreach (Triangle mesh in NavMeshData)
            {
                mesh.ResetData();
            }
        }

        /// <summary>
        /// 根据起始点，计算出正确的落在导航区内的位置
        /// </summary>
        /// <param name="tarTri"></param>
        /// <param name="tarPos"></param>
        /// <param name="otherTri"></param>
        /// <param name="otherPos"></param>
        /// <returns></returns>
        private PathResCode GetFixPos(ref  Triangle orgTarTri, ref Vector2 orgTarPos, Triangle otherTri, Vector2 otherPos,int extendLength)
        {
            Vector2 tarPos = orgTarPos;
            //////////////////////////////////////////////////////////////////////////
            //为了精确判断，需要逆向延长线段一定长度
            if (extendLength > 0)
            {
                Vector2 newTarPos = SGMath.GetExtendPos(otherPos, tarPos, extendLength);
                tarPos = newTarPos;
            }
            //////////////////////////////////////////////////////////////////////////

            Line2D linePath = new Line2D(tarPos, otherPos);
            Rect lineRect = SGMath.GetRect(linePath);
            //1)找到所有矩形相交的nav triangle,判断是否group相等
            List<Triangle> crossNavTris = new List<Triangle>();
            foreach (Triangle tri in NavMeshData)
            {
                if (SGMath.CheckCross(lineRect, tri.BoxCollider))
                {
                    if (otherTri != null && otherTri.Group != tri.Group)
                        continue;
                    crossNavTris.Add(tri);
                }
            }
            //2)找到所有点
            List<Vector2> crossPoints = new List<Vector2>();
            //记录点所在的三角形
            List<int> triIndex = new List<int>();
            Vector2 insPoint;
            //foreach (Triangle crossTri in crossNavTris)
            for (int index = 0; index < crossNavTris.Count; index++)
            {
                Triangle crossTri = crossNavTris[index];
                Line2D triLine;
                for (int i = 0; i < 2; i++)
                {
                    triLine = new Line2D(crossTri.Points[i], crossTri.Points[i + 1]);
                    if (linePath.intersection(triLine, out insPoint) == LineCrossState.CROSS)
                    {
                        crossPoints.Add(insPoint);
                        triIndex.Add(index);
                    }
                }
                triLine = new Line2D(crossTri.Points[2], crossTri.Points[0]);
                if (linePath.intersection(triLine, out insPoint) == LineCrossState.CROSS)
                {
                    crossPoints.Add(insPoint);
                    triIndex.Add(index);
                }
            }
            if (crossPoints.Count == 0)
                return PathResCode.NoCrossPoint;

            //3)找到最接近终点的点
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

    }
}
