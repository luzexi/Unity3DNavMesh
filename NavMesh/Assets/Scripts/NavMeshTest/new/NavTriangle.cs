

using UnityEngine;
using System;

//  NavTriangle.cs
//  Author: Lu Zexi
//  2013-10-04


namespace Game.NavMesh
{

    /// <summary>
    /// Nav三角形类
    /// </summary>
    public class NavTriangle : Triangle
    {
        // 寻路相关参数
        private int m_iSessionID;   //
        private int m_iParentID;    //父节点ID
        private bool m_bIsOpen;     //是否打开


        //评估相关
        private double m_dHValue;   //H评估值
        private double m_dGValue;   //G评估值
        private int m_iInWallIndex; //穿入边索引
        private int m_iOutWallIndex;    //穿出边索引

        public NavTriangle(Vector2 pos1, Vector2 pos2, Vector2 pos3, int id, int groupid)
            :base(pos1 , pos2 , pos3 , id , groupid)
        {
            Reset();
        }

        /// <summary>
        /// 重置
        /// </summary>
        public void Reset()
        {
            this.m_iSessionID = -1;
            this.m_iParentID = -1;
            this.m_bIsOpen = false;
            this.m_iOutWallIndex = -1;
            this.m_dHValue = 0;
            this.m_dGValue = 0;
            this.m_iInWallIndex = -1;
        }

        /// <summary>
        /// 设置当前三角形的穿入边
        /// </summary>
        /// <param name="arriId"></param>
        public void SetArrivalWall(int neighborID)
        {
            if (neighborID == -1)
                return;

            this.m_iInWallIndex = GetWallIndex(neighborID);
        }

        /// <summary>
        /// 获得通过当前三角形的花费
        /// </summary>
        /// <param name="neighborID"></param>
        /// <returns></returns>
        public double GetCost(int neighborID)
        {
            int outWallIndex = GetWallIndex(neighborID);
            if (this.m_iInWallIndex == -1)
                return 0;
            else if (this.m_iInWallIndex != 0)
                return this.m_vecWallDistance[1];
            else if (outWallIndex == 1)
                return this.m_vecWallDistance[0];
            else
                return this.m_vecWallDistance[2];
        }

        /// <summary>
        /// 计算三角形估价函数（h值）
        /// 使用该三角形的中心点（3个顶点的平均值）到路径终点的x和y方向的距离。
        /// </summary>
        /// <param name="endPos">终点</param>
        public void CalcHeuristic(Vector2 endPos)
        {
            double xDelta = Math.Abs(this.m_cCenter.x - endPos.x);
            double yDelta = Math.Abs(this.m_cCenter.y - endPos.y);
            this.m_dHValue = Math.Sqrt(xDelta * xDelta + yDelta * yDelta);
        }

        /// <summary>
        /// 获取SESSIONID
        /// </summary>
        /// <returns></returns>
        public int GetSessionID()
        {
            return this.m_iSessionID;
        }

        /// <summary>
        /// 设置SESSIONID
        /// </summary>
        /// <param name="id"></param>
        public void SetSessionID(int id)
        {
            this.m_iSessionID = id;
        }

        /// <summary>
        /// 获取父节点ID
        /// </summary>
        /// <returns></returns>
        public int GetParentID()
        {
            return this.m_iParentID;
        }

        /// <summary>
        /// 设置父节点
        /// </summary>
        /// <param name="id"></param>
        public void SetParentID( int id )
        {
            this.m_iParentID = id;
        }

        /// <summary>
        /// 获取是否打开
        /// </summary>
        /// <returns></returns>
        public bool GetOpen()
        {
            return this.m_bIsOpen;
        }

        /// <summary>
        /// 设置打开状态
        /// </summary>
        /// <param name="val"></param>
        public void SetOpen(bool val)
        {
            this.m_bIsOpen = val;
        }

        /// <summary>
        /// 获取H评估值
        /// </summary>
        /// <returns></returns>
        public double GetHValue()
        {
            return this.m_dHValue;
        }

        /// <summary>
        /// 获取G评估值
        /// </summary>
        /// <returns></returns>
        public double GetGValue()
        {
            return this.m_dGValue;
        }

        /// <summary>
        /// 设置G评估值
        /// </summary>
        /// <param name="val"></param>
        public void SetGValue(double val)
        {
            this.m_dGValue = val;
        }

        /// <summary>
        /// 获取穿入边索引
        /// </summary>
        /// <returns></returns>
        public int InWallIndex()
        {
            return this.m_iInWallIndex;
        }

        /// <summary>
        /// 获取穿出边索引
        /// </summary>
        /// <returns></returns>
        public int GetOutWallIndex()
        {
            return this.m_iOutWallIndex;
        }

        /// <summary>
        /// 设置穿出边索引
        /// </summary>
        /// <param name="index"></param>
        public void SetOutWallIndex(int index)
        {
            this.m_iOutWallIndex = index;
        }
    }

}
