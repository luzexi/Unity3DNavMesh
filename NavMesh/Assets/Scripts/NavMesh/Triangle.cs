

using UnityEngine;
using System;
using System.IO;


//  Triangle.cs
//  Author: Lu Zexi
//  2013-10-04


namespace Game.NavMesh
{

    /// <summary>
    /// 三角形类
    /// </summary>
    public class Triangle
    {
        //基础数据
		protected Vector2[] m_vecPoints;  //三角形定点列表
		protected int m_iID;  //三角形ID
		protected int m_iGroupID;   //三角形组ID
		protected int[] m_vecNeighbors;   //三角形邻居节点ID
        
        //计算数据
        protected Vector2 m_cCenter;  //三角形中心点
        protected Rect m_cBoxCollider;    //三角形包围盒
        protected double[] m_vecWallDistance;  //三角形相邻两边的中点距离

        public Triangle()
        {
            this.m_vecPoints = new Vector2[3];
            this.m_vecWallDistance = new double[3];
            this.m_vecNeighbors = new int[] { -1, -1, -1 };
            this.m_cCenter = Vector2.zero;
        }

        public Triangle( Vector2 pos1 , Vector2 pos2 , Vector2 pos3 , int id , int groupid )
        {
            this.m_vecPoints = new Vector2[3];
            this.m_vecWallDistance = new double[3];
            this.m_vecNeighbors = new int[]{-1,-1,-1};

            this.m_iID = id;
            this.m_iGroupID = groupid;

            this.m_vecPoints[0] = pos1;
            this.m_vecPoints[1] = pos2;
            this.m_vecPoints[2] = pos3;


            //计算中心点
            Vector2 temp = new Vector2();
            temp.x = (this.m_vecPoints[0].x + this.m_vecPoints[1].x + this.m_vecPoints[2].x) / 3;
            temp.y = (this.m_vecPoints[0].y + this.m_vecPoints[1].y + this.m_vecPoints[2].y) / 3;
            this.m_cCenter = temp;

            //计算三角形相邻两边的中点距离
            Vector2[] wallMidPoint = new Vector2[3];
            wallMidPoint[0] = new Vector2((this.m_vecPoints[0].x + this.m_vecPoints[1].x) / 2, (this.m_vecPoints[0].y + this.m_vecPoints[1].y) / 2);
            wallMidPoint[1] = new Vector2((this.m_vecPoints[1].x + this.m_vecPoints[2].x) / 2, (this.m_vecPoints[1].y + this.m_vecPoints[2].y) / 2);
            wallMidPoint[2] = new Vector2((this.m_vecPoints[2].x + this.m_vecPoints[0].x) / 2, (this.m_vecPoints[2].y + this.m_vecPoints[0].y) / 2);

            this.m_vecWallDistance[0] = Math.Sqrt((wallMidPoint[0].x - wallMidPoint[1].x) * (wallMidPoint[0].x - wallMidPoint[1].x)
                + (wallMidPoint[0].y - wallMidPoint[1].y) * (wallMidPoint[0].y - wallMidPoint[1].y));
            this.m_vecWallDistance[1] = Math.Sqrt((wallMidPoint[1].x - wallMidPoint[2].x) * (wallMidPoint[1].x - wallMidPoint[2].x)
                + (wallMidPoint[1].y - wallMidPoint[2].y) * (wallMidPoint[1].y - wallMidPoint[2].y));
            this.m_vecWallDistance[2] = Math.Sqrt((wallMidPoint[2].x - wallMidPoint[0].x) * (wallMidPoint[2].x - wallMidPoint[0].x)
                + (wallMidPoint[2].y - wallMidPoint[0].y) * (wallMidPoint[2].y - wallMidPoint[0].y));


            //计算包围盒
            CalcCollider();
        }

		public NavTriangle CloneNavTriangle()
		{
			NavTriangle tri = new NavTriangle();
			//基础数据
			Array.Copy(this.m_vecPoints,tri.m_vecPoints,this.m_vecPoints.Length);  //三角形定点列表
			tri.m_iID = this.m_iID;  //三角形ID
			tri.m_iGroupID = this.m_iGroupID;   //三角形组ID
			Array.Copy( this.m_vecNeighbors , tri.m_vecNeighbors , this.m_vecNeighbors.Length);   //三角形邻居节点ID
			
			//计算数据
			tri.m_cCenter = this.m_cCenter;  //三角形中心点
			tri.m_cBoxCollider = this.m_cBoxCollider;    //三角形包围盒
			//tri.m_vecWallDistance = this.m_vecWallDistance;  //三角形相邻两边的中点距离
			Array.Copy(this.m_vecWallDistance , tri.m_vecWallDistance , this.m_vecWallDistance.Length);
			return tri;
		}

        /// <summary>
        /// 计算包围盒
        /// </summary>
        private void CalcCollider()
        {
            //计算包围盒
            if (this.m_vecPoints[0] == this.m_vecPoints[1] || this.m_vecPoints[1] == this.m_vecPoints[2] || this.m_vecPoints[0] == this.m_vecPoints[2])
            {
                DEBUG.ERROR("Triangle:This is not a triangle.");
                return;
            }

            Rect collider = new Rect();
            collider.xMin = collider.xMax = this.m_vecPoints[0].x;
            collider.yMin = collider.yMax = this.m_vecPoints[0].y;
            for (int i = 1; i < 3; i++)
            {
                if (this.m_vecPoints[i].x < collider.xMin)
                {
                    collider.xMin = this.m_vecPoints[i].x;
                }
                else if (this.m_vecPoints[i].x > collider.xMax)
                {
                    collider.xMax = this.m_vecPoints[i].x;
                }

                if (this.m_vecPoints[i].y < collider.yMin)
                {
                    collider.yMin = this.m_vecPoints[i].y;
                }
                else if (this.m_vecPoints[i].y > collider.yMax)
                {
                    collider.yMax = this.m_vecPoints[i].y;
                }
            }

            this.m_cBoxCollider = collider;
        }

        /// <summary>
        /// 计算邻居节点
        /// </summary>
        /// <param name="triNext"></param>
        /// <returns></returns>
        public int isNeighbor(Triangle triNext)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (GetSide(i).Equals(triNext.GetSide(j)))
                        return i;
                }
            }
            return -1;
        }

        /// <summary>
        /// 测试给定点是否在三角形中
        /// 点在三角形边上也算
        /// </summary>
        /// <param name="pt">指定点</param>
        /// <returns>是否在三角形中</returns>
        public bool IsPointIn(Vector2 pt)
        {
            if (this.m_cBoxCollider.xMin != this.m_cBoxCollider.xMax && !this.m_cBoxCollider.Contains(pt))
                return false;

            PointSide resultA = GetSide(0).ClassifyPoint(pt);
            PointSide resultB = GetSide(1).ClassifyPoint(pt);
            PointSide resultC = GetSide(2).ClassifyPoint(pt);

            if (resultA == PointSide.ON_LINE || resultB == PointSide.ON_LINE || resultC == PointSide.ON_LINE)
            {
                return true;
            }
            else if (resultA == PointSide.RIGHT_SIDE && resultB == PointSide.RIGHT_SIDE && resultC == PointSide.RIGHT_SIDE)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// 根据索引获得相应的边
        /// </summary>
        /// <param name="sideIndex"></param>
        /// <returns></returns>
        public Line2D GetSide(int sideIndex)
        {
            Line2D newSide;

            switch (sideIndex)
            {
                case 0:
                    newSide = new Line2D(this.m_vecPoints[0], this.m_vecPoints[1]);
                    break;
                case 1:
                    newSide = new Line2D(this.m_vecPoints[1], this.m_vecPoints[2]);
                    break;
                case 2:
                    newSide = new Line2D(this.m_vecPoints[2], this.m_vecPoints[0]);
                    break;
                default:
                    newSide = new Line2D(this.m_vecPoints[0], this.m_vecPoints[1]);
                    //DEBUG.ERROR("Triangle:GetSide 获取索引[" + sideIndex + "]错误");
                    break;
            }

            return newSide;
        }

        /// <summary>
        /// 获得邻居ID边的索引
        /// </summary>
        /// <param name="neighborID">邻居三角形ID</param>
        /// <returns></returns>
        public int GetWallIndex(int neighborID)
        {
            for (int i = 0; i < 3; i++)
            {
                if (this.m_vecNeighbors[i] != -1 && this.m_vecNeighbors[i] == neighborID)
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// 比较并获取三角形邻居边索引
        /// </summary>
        /// <param name="triNext">三角形</param>
        /// <returns>邻边索引</returns>
        public int GetNeighborWall(Triangle triNext)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (GetSide(i).Equals(triNext.GetSide(j)))
                        return i;
                }
            }
            return -1;
        }

        /// <summary>
        /// 获取三角形ID
        /// </summary>
        /// <returns></returns>
        public int GetID()
        {
            return this.m_iID;
        }

        /// <summary>
        /// 获取组ID
        /// </summary>
        /// <returns></returns>
        public int GetGroupID()
        {
            return this.m_iGroupID;
        }

        /// <summary>
        /// 获取中心点
        /// </summary>
        /// <returns></returns>
        public Vector2 GetCenter()
        {
            return this.m_cCenter;
        }

        /// <summary>
        /// 获取包围盒
        /// </summary>
        /// <returns></returns>
        public Rect GetBoxCollider()
        {
            return this.m_cBoxCollider;
        }

        /// <summary>
        /// 获取指定点
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public Vector2 GetPoint(int index)
        {
            if (index >= 3)
            {
                DEBUG.ERROR("GetPoint:The index is large than 3.");
                return Vector2.zero;
            }

            return this.m_vecPoints[index];
        }

        /// <summary>
        /// 获取邻居节点ID
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public int GetNeighbor(int index)
        {
            if (index >= 3)
            {
                DEBUG.ERROR("GetNeighbor:The index is large than 3.");
                return -1;
            }

            return this.m_vecNeighbors[index];
        }

        /// <summary>
        /// 设置邻居三角形ID
        /// </summary>
        /// <param name="index"></param>
        /// <param name="id"></param>
        public void SetNeighbor(int index, int id)
        {
            if (index >= 3)
            {
                DEBUG.ERROR("SetNeighbor:The index is large than 3.");
                return;
            }

            this.m_vecNeighbors[index] = id;
        }

        /// <summary>
        /// 获取三边中点距离
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public double GetWallDis(int index)
        {
            if (index >= 3)
            {
                DEBUG.ERROR("GetWallDis:The index is large than 3.");
                return -1;
            }

            return this.m_vecWallDistance[index];
        }

        /// <summary>
        /// 读取数据
        /// </summary>
        public virtual void Read( BinaryReader binReader)
        {
            // 读取id
            this.m_iID = binReader.ReadInt32();
            // 读取多边形的顶点
            for (int pNum = 0; pNum < 3; pNum++)
            {
                this.m_vecPoints[pNum].x = binReader.ReadSingle();
                this.m_vecPoints[pNum].y = binReader.ReadSingle();
            }

            // 计算包围盒
            CalcCollider();

            // 读取邻居节点
            for (int neighborId = 0; neighborId < 3; neighborId++)
            {
                this.m_vecNeighbors[neighborId] = binReader.ReadInt32();
            }
            // 读取每条边中点距离
            for (int wall = 0; wall < 3; wall++)
            {
                this.m_vecWallDistance[wall] = binReader.ReadDouble();
            }

            // 读取中心点
            Vector2 tempCenter = this.m_cCenter;
            tempCenter.x = binReader.ReadSingle();
            tempCenter.y = binReader.ReadSingle();
            this.m_cCenter = tempCenter;

            // 读取区域id
            this.m_iGroupID = binReader.ReadInt32();
        }

    }

}
