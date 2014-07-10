/********************************************************************
	created:	2011/12/19
	created:	19:12:2011   13:31
	filename: 	SGWeb\DependProjects\NavMesh\NavMeshGen.cs
	file path:	SGWeb\DependProjects\NavMesh
	file base:	NavMeshGen
	file ext:	cs
	author:		Ivan
	
	purpose:	这个类用来生成导航网格(保存，加载)，必须传入不可行走区域。
                请加载NavMeshTestProject项目来运行单元测试，保证方法的正确
*********************************************************************/
using System;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.IO;

namespace Game.NavMesh
{
    /// <summary>
    /// 生成结果
    /// </summary>
    public enum NavResCode
    {
        Success = 0,
        Failed = -1,
        NotFindDt = -2,
        FileNotExist = -3,
        VersionNotMatch = -4,
    }

    public sealed class NavMeshGen
    {
        // 使用者必须实现自己的加载数据函数，否则使用默认的C#的文件加载 [1/10/2012 Ivan]
        public delegate UnityEngine.Object GetResources(string fileName);
        // 更改为单例模式
        static readonly NavMeshGen instance = new NavMeshGen();

        static NavMeshGen() { }

        public static NavMeshGen Instance
        {
            get
            {
                return instance;
            }
        }

        // version info
        private const string NAVMESH_VERSION = "SGNAVMESH_01";

        public List<Line2D> allEdges { get; set; }  //所有阻挡区域的边
        public List<Vector2> allPoints { get; set; }//所有顶点列表
        public List<Line2D> allStartEdges { get; set; } //所有起始边

        public NavMeshGen()
        {
            allEdges = new List<Line2D>();
            allStartEdges = new List<Line2D>();
            allPoints = new List<Vector2>();
        }

        /// <summary>
        /// 保存导航网格信息到文件里面
        /// </summary>
        /// <param name="filePath">文件全路径</param>
        /// <returns></returns>
        public NavResCode SaveNavMeshToFile(string filePath, List<Triangle> navTriangles)
        {
            if (navTriangles == null || navTriangles.Count == 0)
                return NavResCode.Failed;

            UTF8Encoding utf8 = new UTF8Encoding();

            if (!Directory.Exists(Path.GetDirectoryName(filePath)))
                Directory.CreateDirectory(Path.GetDirectoryName(filePath));

            FileStream fs = File.Create(filePath);
            BinaryWriter binWriter = new BinaryWriter(fs);

            // save version
            binWriter.Write(utf8.GetBytes(NAVMESH_VERSION));
            // save triangle count
            binWriter.Write(navTriangles.Count);

            // 遍历导航网格并保存
            foreach (Triangle navTri in navTriangles)
            {
                if (navTri != null)
                {
                    //保存网格ID
                    binWriter.Write(navTri.GetID());
                    //保存网格的三角形顶点
                    for (int i = 0; i < 3; i++)
                    {
                        binWriter.Write(navTri.GetPoint(i).x);
                        binWriter.Write(navTri.GetPoint(i).y);
                    }

                    // 保存所有邻居边
                    for (int i = 0; i < 3; i++)
                    {
                        binWriter.Write(navTri.GetNeighbor(i));
                    }

                    // 保存每条边中点距离
                    for (int i = 0; i < 3; i++)
                    {
                        binWriter.Write(navTri.GetWallDis(i));
                    }

                    // 保存中心点位置
                    binWriter.Write(navTri.GetCenter().x);
                    binWriter.Write(navTri.GetCenter().y);

                    //保存区域id
                    binWriter.Write(navTri.GetGroupID());
                }
            }

            // close file
            binWriter.Close();
            fs.Close();

            return NavResCode.Success;
        }

        /// <summary>
        /// 从文件中读取导航网格信息
        /// </summary>
        /// <param name="filePath">文件全路径</param>
        /// <param name="navTriangles">读取的导航网格</param>
        /// <returns></returns>
        public NavResCode LoadNavMeshFromResource(string filePath, out List<Triangle> navTriangles, GetResources GetRes)
        {
            navTriangles = new List<Triangle>();
            BinaryReader binReader = null;
            MemoryStream stream = null;

            if (GetRes == null)
            {
                if (!File.Exists(filePath))
                    return NavResCode.FileNotExist;

                // open file
                FileStream fs = File.Open(filePath, FileMode.Open);
                binReader = new BinaryReader(fs);
            }
            else
            {

                UnityEngine.Object FileObject = GetRes(filePath);
                if (FileObject == null)
                {
                    return NavResCode.FileNotExist;
                }

                TextAsset asset = (TextAsset)FileObject;

                stream = new MemoryStream(asset.bytes);
                binReader = new BinaryReader(stream);
            }
            

            NavResCode res = NavResCode.Failed;
            try
            {
                res = LoadNavMeshFromFile(ref navTriangles, binReader);
            }
            catch
            {
            }
            finally
            {
                binReader.Close();
                if( GetRes != null )
                    stream.Close();
            }


            return res;
        }

        /// <summary>
        /// 从文件中读取导航网格信息(这个函数使用C#的文件加载)
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="navTriangles"></param>
        /// <returns></returns>
        public NavResCode LoadNavMeshFromFile(string filePath, out List<Triangle> navTriangles)
        {
            //navTriangles = new List<Triangle>();
            //// check file exist
            //if (!File.Exists(filePath))
            //    return NavResCode.FileNotExist;

            //// open file
            //FileStream fs = File.Open(filePath, FileMode.Open);
            //BinaryReader binReader = new BinaryReader(fs);

            //NavResCode res = NavResCode.Failed;
            //try
            //{
            //    res = LoadNavMeshFromFile(ref navTriangles, binReader);
            //}
            //catch
            //{
            //}
            //finally
            //{
            //    binReader.Close();
            //    fs.Close();
            //}


            //return res;
            return LoadNavMeshFromResource(filePath, out navTriangles, null);
        }

        /// <summary>
        /// 根据传进来BinaryReader读取数据
        /// </summary>
        /// <param name="navTriangles"></param>
        /// <param name="binReader"></param>
        /// <returns></returns>
        public NavResCode LoadNavMeshFromFile(ref List<Triangle> navTriangles, BinaryReader binReader)
        {
            try
            {
                // 读取版本号
                string fileVersion = new string(binReader.ReadChars(NAVMESH_VERSION.Length));
                if (fileVersion != NAVMESH_VERSION)
                    return NavResCode.VersionNotMatch;
                // 读取导航三角形数量
                int navCount = binReader.ReadInt32();
                Triangle currTri;
                for (int i = 0; i < navCount; i++)
                {
                    currTri = new Triangle();
                    currTri.Read(binReader);
                    navTriangles.Add(currTri);
                }
            }
            catch
            {
                //Debug.LogError(e.Message);
                return NavResCode.Failed;
            }
            finally
            {
            }

            return NavResCode.Success;
        }

        /// <summary>
        /// 创建导航网格
        /// </summary>
        /// <param name="polyAll">所有阻挡区域</param>
        /// <param name="triAll">输出的导航网格</param>
        /// <returns></returns>
        public NavResCode CreateNavMesh(List<Polygon> polyAll, ref List<Triangle> triAll)
        {
            triAll.Clear();
            List<Line2D> allLines = new List<Line2D>();  //线段堆栈

            //Step1 保存顶点和边
            NavResCode initRes = InitData(polyAll);
            if (initRes != NavResCode.Success)
                return initRes;

			Debug.Log(allStartEdges.Count + " edges");
            int index = 0;
            int lastNeighborId = -1;
            Triangle lastTri = null;

            //Step2.遍历边界边作为起点
            for (int edgeIndex = 0; edgeIndex < allStartEdges.Count; edgeIndex++)
            {
                Line2D sEdge = allStartEdges[edgeIndex];
                allLines.Add(sEdge);
                Line2D edge = null;

                do
                {
                    //Step3.选出计算出边的DT点，构成约束Delaunay三角形
                    edge = allLines[allLines.Count - 1];
                    allLines.Remove(edge);

                    Vector2 dtPoint;
                    bool isFindDt = FindDT(edge, out dtPoint);
                    if (!isFindDt)
                        continue;
					//Debug.Log(edge.GetStartPoint().ToString() + " - " + edge.GetEndPoint().ToString() + " - " + dtPoint.ToString());
                    Line2D lAD = new Line2D(edge.GetStartPoint(), dtPoint);
                    Line2D lDB = new Line2D(dtPoint, edge.GetEndPoint());

                    //创建三角形
                    Triangle delaunayTri = new Triangle(edge.GetStartPoint(), edge.GetEndPoint(), dtPoint, index++ , edgeIndex);
                    // 保存邻居节点
                    //                     if (lastNeighborId != -1)
                    //                     {
                    //                         delaunayTri.SetNeighbor(lastNeighborId);
                    //                         if(lastTri != null)
                    //                             lastTri.SetNeighbor(delaunayTri.ID);
                    //                     }
                    //save result triangle
                    triAll.Add(delaunayTri);

                    // 保存上一次的id和三角形
                    lastNeighborId = delaunayTri.GetID();
                    lastTri = delaunayTri;

                    int lineIndex;
                    //Step4.检测刚创建的的线段ad,db；如果如果它们不是约束边
                    //并且在线段堆栈中，则将其删除，如果不在其中，那么将其放入
                    if (!Line2D.CheckLineIn(allEdges, lAD, out lineIndex))
                    {
                        if (!Line2D.CheckLineIn(allLines, lAD, out lineIndex))
                            allLines.Add(lAD);
                        else
                            allLines.RemoveAt(lineIndex);
                    }

                    if (!Line2D.CheckLineIn(allEdges, lDB, out lineIndex))
                    {
                        if (!Line2D.CheckLineIn(allLines, lDB, out lineIndex))
                            allLines.Add(lDB);
                        else
                            allLines.RemoveAt(lineIndex);
                    }

                    //Step5.如果堆栈不为空，则转到第Step3.否则结束循环 
                } while (allLines.Count > 0);
            }

            // 计算邻接边和每边中点距离
            for (int i = 0; i < triAll.Count; i++)
            {
                Triangle tri = triAll[i];
                //// 计算每个三角形每边中点距离
                //tri.calcWallDistance();

                // 计算邻居边
                for (int j = 0; j < triAll.Count; j++)
                {
                    Triangle triNext = triAll[j];
                    if (tri.GetID() == triNext.GetID())
                        continue;

                    int result = tri.isNeighbor(triNext);
                    if (result != -1)
                    {
                        tri.SetNeighbor(result , triNext.GetID() );
                    }
                }
            }

            return NavResCode.Success;
        }

        bool needSplitBig = false;
        // 判断是否需要拆分大三角形
        public bool NeedSplitBig
        {
            get { return needSplitBig; }
            set { needSplitBig = value; }
        }
        int needSplitSize = 50;
        // 拆分的尺寸
        public int NeedSplitSize
        {
            get { return needSplitSize; }
            set { needSplitSize = value; }
        }
        //         void SplitBigTriangle(ref List<Triangle> triangles)
        //         {
        //             // 将面积过大的三角形拆分成三个小三角形
        //             for (int i = 0; i < triangles.Count; i++)
        //             {
        //                 if (triangles[i].Area() > NeedSplitSize)
        //                 {
        //                 }
        //             }
        //         }

        /// <summary>
        /// 判断点是否是线段的可见点，组成的三角形没有和其他边相交
        /// </summary>
        /// <param name="line"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        private bool IsPointVisibleOfLine(Line2D line, Vector2 point)
        {
            if (line == null)
                return false;

            Vector2 sPnt = line.GetStartPoint();
            Vector2 ePnt = line.GetEndPoint();

            // 是否是线段端点
            if (point == sPnt || point == ePnt)
                return false;
            //点不在线段的右侧（多边形顶点顺序为顺时针）
            if (line.ClassifyPoint(point) != PointSide.RIGHT_SIDE)
                return false;

            if (!IsVisibleIn2Point(sPnt, point))
                return false;

            if (!IsVisibleIn2Point(ePnt, point))
                return false;

            return true;
        }

        /// <summary>
        /// 判断这条线段是否没有和其他的边相交
        /// </summary>
        /// <param name="sPnt"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        private bool IsVisibleIn2Point(Vector2 sPnt, Vector2 ePnt)
        {
            Line2D line = new Line2D(sPnt, ePnt);
            Vector2 interPos;

            foreach (Line2D edge in allEdges)
            {
                if (edge.Intersection(line, out interPos) == LineCrossState.CROSS)
                {
                    if (!NMath.IsEqualZero(sPnt - interPos)
                        && !NMath.IsEqualZero(ePnt - interPos))
                        return false;
                }
            }
            return true;
        }

        /// <summary>
        /// 找到指定边的约束边DT
        /// </summary>
        /// <param name="line"></param>
        /// <returns></returns>
        private bool FindDT(Line2D line, out Vector2 dtPoint)
        {
            dtPoint = new Vector2();
            if (line == null)
                return false;

            Vector2 ptA = line.GetStartPoint();
            Vector2 ptB = line.GetEndPoint();

            List<Vector2> visiblePnts = new List<Vector2>();
            foreach (Vector2 point in allPoints)
            {
                if (IsPointVisibleOfLine(line, point))
                    visiblePnts.Add(point);
            }

            if (visiblePnts.Count == 0)
                return false;

            bool bContinue = false;
            dtPoint = visiblePnts[0];

            do
            {
                bContinue = false;
                //Step1.构造三角形的外接圆，以及外接圆的包围盒
                Circle circle = NMath.CreateCircle(ptA, ptB, dtPoint);
                Rect boundBox = NMath.GetCircleBoundBox(circle);

                //Step2. 依次访问网格包围盒内的每个网格单元：
                //若某个网格单元中存在可见点 p, 并且 ∠p1pp2 > ∠p1p3p2，则令 p3=p，转Step1；
                //否则，转Step3.
                float angOld = (float)Math.Abs(NMath.LineRadian(ptA, dtPoint, ptB));
                foreach (Vector2 pnt in visiblePnts)
                {
                    if (pnt == ptA || pnt == ptB || pnt == dtPoint)
                        continue;
                    if (!boundBox.Contains(pnt))
                        continue;

                    float angNew = (float)Math.Abs(NMath.LineRadian(ptA, pnt, ptB));
                    if (angNew > angOld)
                    {
                        dtPoint = pnt;
                        bContinue = true;
                        break;
                    }
                }

                //false 转Step3
            } while (bContinue);

            //Step3. 若当前网格包围盒内所有网格单元都已被处理完，
            // 也即C（p1，p2，p3）内无可见点，则 p3 为的 p1p2 的 DT 点
            return true;
        }

        /// <summary>
        /// 初始化创建导航网格需要的数据
        /// </summary>
        /// <param name="polyAll">所有阻挡区域</param>
        /// <returns></returns>
        private NavResCode InitData(List<Polygon> polyAll)
        {
            allEdges = new List<Line2D>();
            allPoints = new List<Vector2>();
            allStartEdges = new List<Line2D>();

            PolyResCode resCode = NavUtil.UnionAllPolygon(ref polyAll);
            if (resCode != PolyResCode.Success)
                return NavResCode.Failed;
            // 保存所有点和边
            foreach (Polygon poly in polyAll)
            {
                if (poly.GetPoints().Count < 3)
                    continue;
                AddPoint(poly.GetPoints());
                AddEdge(poly.GetPoints());

                // 更改算法，初始边只用孤岛的内边，这就强制要求使用的时候必须把可以行走的区域全部包围起来
                //if (poly.GetTag() != 0)
                {
                    allStartEdges.Add(new Line2D(poly.GetPoints()[1], poly.GetPoints()[0]));
                }
            }
			Debug.Log(allStartEdges.Count + " alledges.");
            return NavResCode.Success;
        }

        /// <summary>
        /// 保存用到的顶点
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        private NavResCode AddPoint(List<Vector2> points)
        {
            foreach (Vector2 point in points)
            {
                allPoints.Add(point);
            }

            return NavResCode.Success;
        }

        /// <summary>
        /// 保存所有边
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        private NavResCode AddEdge(List<Vector2> points)
        {
            Vector2 pBegin = points[0];
            for (int i = 1; i < points.Count; i++)
            {
                Vector2 pEnd = points[i];
                Line2D line = new Line2D(pBegin, pEnd);
                allEdges.Add(line);
                pBegin = pEnd;
            }
            Line2D lineEnd = new Line2D(pBegin, points[0]);
            allEdges.Add(lineEnd);

            return NavResCode.Success;
        }

    }
}
