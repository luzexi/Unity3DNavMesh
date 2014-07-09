
using UnityEngine;
using System;
using System.Collections.Generic;


//  Line2D.cs
//  Author: Lu Zexi
//  2013-10-04



namespace Game.NavMesh
{
    /// <summary>
    /// 点与线段所在位置
    /// </summary>
    public enum PointSide
    {
        ON_LINE = 0,    //在线段上
        LEFT_SIDE = 1,  //在线段左边
        RIGHT_SIDE = 2, //在线段右边
    };

    /// <summary>
    /// 两线段交叉状态
    /// </summary>
    public enum LineCrossState
    {
        COLINE = 0, //外线口
        PARALLEL,   //平行线
        CROSS,      //相交
        NOT_CROSS   //无相交
    }
    
    /// <summary>
    /// 2D线段类
    /// </summary>
    public class Line2D
    {
        private Vector2 m_cStartPoint;  //起始点
        private Vector2 m_cEndPoint;    //结束点

        public Line2D(Vector2 ps, Vector2 pe)
        {
            this.m_cStartPoint = ps;
            this.m_cEndPoint = pe;
        }

        /// <summary>
        /// 检测线段是否在给定线段列表里面
        /// </summary>
        /// <param name="allLines">线段列表</param>
        /// <param name="chkLine">检查线段</param>
        /// <param name="index">如果在，返回索引</param>
        /// <returns></returns>
        public static bool CheckLineIn(List<Line2D> allLines, Line2D chkLine, out int index)
        {
            index = -1;
            for (int i = 0; i < allLines.Count; i++)
            {
                Line2D line = allLines[i];
                if (line.Equals(chkLine))
                {
                    index = i;
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// 判断点与直线的关系，假设你站在a点朝向b点， 
        /// 则输入点与直线的关系分为：Left, Right or Centered on the line
        /// </summary>
        /// <param name="point">判断点</param>
        /// <returns>判断结果</returns>
        public PointSide ClassifyPoint(Vector2 point)
        {
            if (point == this.m_cStartPoint || point == this.m_cEndPoint)
                return PointSide.ON_LINE;
            //向量a
            Vector2 vectorA = this.m_cEndPoint - this.m_cStartPoint;
            //向量b
            Vector2 vectorB = point - this.m_cStartPoint;

            float crossResult = NMath.CrossProduct(vectorA, vectorB);
            if (NMath.IsEqualZero(crossResult))
                return PointSide.ON_LINE;
            else if (crossResult < 0)
                return PointSide.RIGHT_SIDE;
            else
                return PointSide.LEFT_SIDE;
        }

        /// <summary>
        /// 计算两条二维线段的交点
        /// </summary>
        /// <param name="other">Other line</param>
        /// <param name="intersectPoint">输出的线段交点</param>
        /// <returns>返回值说明了两条线段的位置关系(COLINE,PARALLEL,CROSS,NOT_CROSS) </returns>
        public LineCrossState Intersection(Line2D other, out Vector2 intersectPoint)
        {
            intersectPoint.x = intersectPoint.y = float.NaN;
            if (!NMath.CheckCross(this.m_cStartPoint, this.m_cEndPoint, other.m_cStartPoint, other.m_cEndPoint))
                return LineCrossState.NOT_CROSS;

            double A1, B1, C1, A2, B2, C2;

            A1 = this.m_cEndPoint.y - this.m_cStartPoint.y;
            B1 = this.m_cStartPoint.x - this.m_cEndPoint.x;
            C1 = this.m_cEndPoint.x * this.m_cStartPoint.y - this.m_cStartPoint.x * this.m_cEndPoint.y;

            A2 = other.m_cEndPoint.y - other.m_cStartPoint.y;
            B2 = other.m_cStartPoint.x - other.m_cEndPoint.x;
            C2 = other.m_cEndPoint.x * other.m_cStartPoint.y - other.m_cStartPoint.x * other.m_cEndPoint.y;

            if (NMath.IsEqualZero(A1 * B2 - B1 * A2))
            {
                if (NMath.IsEqualZero((A1 + B1) * C2 - (A2 + B2) * C1))
                {
                    return LineCrossState.COLINE;
                }
                else
                {
                    return LineCrossState.PARALLEL;
                }
            }
            else
            {
                intersectPoint.x = (float)((B2 * C1 - B1 * C2) / (A2 * B1 - A1 * B2));
                intersectPoint.y = (float)((A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2));
                return LineCrossState.CROSS;
            }
        }

        /// <summary>
        /// 获得直线方向
        /// </summary>
        /// <returns>矢量</returns>
        public Vector2 GetDirection()
        {
            Vector2 dir = this.m_cEndPoint - this.m_cStartPoint;
            return dir;
        }

        /// <summary>
        /// 选段长度
        /// </summary>
        /// <returns></returns>
        public float GetLength()
        {
            return (float)Math.Sqrt(Math.Pow(this.m_cStartPoint.x - this.m_cEndPoint.x, 2.0) + Math.Pow(this.m_cStartPoint.y - this.m_cEndPoint.y, 2.0));
        }

        /// <summary>
        /// 两条线段是否相等
        /// </summary>
        /// <param name="lineTemp">判断对象</param>
        /// <returns>是否相等</returns>
        public bool Equals(Line2D line)
        {
			//只是一个点
			if (SGMath.IsEqualZero(line.m_cStartPoint - line.m_cEndPoint) ||
			    SGMath.IsEqualZero(m_cStartPoint - m_cEndPoint))
				return false;
			
			bool bEquals = NMath.IsEqualZero(m_cStartPoint - line.m_cStartPoint) ? true : NMath.IsEqualZero(m_cStartPoint - line.m_cEndPoint);
			if (bEquals)
			{
				bEquals = NMath.IsEqualZero(m_cEndPoint - line.m_cStartPoint) ? true : NMath.IsEqualZero(m_cEndPoint - line.m_cEndPoint);
			}
			return bEquals;
			//            Line2D line = (Line2D)lineTemp;
			//            if (line == null)
			//            {
//                return false;
//            }
//
//            return (NMath.IsEqual(this.m_cStartPoint, line.m_cStartPoint) && NMath.IsEqual(this.m_cEndPoint, line.m_cEndPoint)); ;
        }

        /// <summary>
        /// 获取起始点
        /// </summary>
        /// <returns>起始点</returns>
        public Vector2 GetStartPoint()
        {
            return this.m_cStartPoint;
        }

        /// <summary>
        /// 获取结束点
        /// </summary>
        /// <returns>结束点</returns>
        public Vector2 GetEndPoint()
        {
            return this.m_cEndPoint;
        }

    }

}
