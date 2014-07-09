


using UnityEngine;
using System.Collections.Generic;


//  Polygon.cs
//  Author: Lu Zexi
//  2013-10-04




namespace Game.NavMesh
{
    public enum PolyResCode
    {
        Success = 0,
        ErrEmpty = -1,      //空多边形
        ErrNotCross = -2,   //没有相交
        ErrCrossNum = -3,   // 多边形交点数量错误
        ErrNotInside = -4   //不在多边形上
    }

    /// <summary>
    /// 多边形类
    /// </summary>
    public class Polygon
    {
        private int m_iTag; //标志
        private List<Vector2> m_lstPoints;  //列表点

        public Polygon()
        {
            this.m_lstPoints = new List<Vector2>();
            this.m_iTag = 0;
        }

        public Polygon(List<Vector2> points)
        {
            this.m_lstPoints = points;
            this.m_iTag = 0;
        }

        /// <summary>
        /// 删除重复顶点
        /// </summary>
        /// <returns></returns>
        public void DelRepeatPoint()
        {
            for (int i = 0; i < this.m_lstPoints.Count; i++)
            {
                for (int j = i + 1; j < this.m_lstPoints.Count; j++)
                {
                    if (NMath.IsEqualZero(this.m_lstPoints[i] - this.m_lstPoints[j]))
                    {
                        this.m_lstPoints.Remove(this.m_lstPoints[j]);
                        j = i;
                    }
                }
            }
        }


        /// <summary>
        /// 顺时针排序
        /// </summary>
        /// <returns></returns>
        public void CW()
        {
            if (!IsCW())
            {
                this.m_lstPoints.Reverse();
            }
        }


        /// <summary>
        /// 判断是否是顺时针
        /// </summary>
        /// <returns></returns>
        public bool IsCW()
        {
            if (this.m_lstPoints.Count <= 2)
                return false;

            //最上（y最小）最左（x最小）点， 肯定是一个凸点
            //寻找最上点
            Vector2 topPoint = this.m_lstPoints[0];
            int topIndex = 0;
            for (int i = 1; i < this.m_lstPoints.Count; i++)
            {
                Vector2 currPoint = this.m_lstPoints[i];
                if ((topPoint.y > currPoint.y)
                    || ((topPoint.y == currPoint.y) && (topPoint.x > currPoint.x)))
                {
                    topPoint = currPoint;
                    topIndex = i;
                }
            }

            //寻找左右邻居
            int preIndex = (topIndex - 1) >= 0 ? (topIndex - 1) : (this.m_lstPoints.Count - 1);
            int nextIndex = (topIndex + 1) < this.m_lstPoints.Count ? (topIndex + 1) : 0;

            Vector2 prePoint = this.m_lstPoints[preIndex];
            Vector2 nextPoint = this.m_lstPoints[nextIndex];

            //三点共线情况不存在，若三点共线则说明必有一点的y（斜线）或x（水平线）小于topPt
            float r = NMath.CrossProduct((prePoint - topPoint), (nextPoint - topPoint));
            if (r > 0)
                return true;

            return false;
        }

        // ========================================== 合并 ============================================= //

        /// <summary>
        /// 合并两个交叉的多边形(多边形必须先转换为顺时针方向，调用CW()函数!)
        /// </summary>
        /// <param name="other"></param>
        /// <param name="polyRes"></param>
        /// <returns></returns>
        public PolyResCode Union(Polygon other, ref List<Polygon> polyRes)
        {
            if (this.m_lstPoints.Count == 0 || other.m_lstPoints.Count == 0)
                return PolyResCode.ErrEmpty;
            else if (!NMath.CheckCross(GetCoverRect(), other.GetCoverRect()))
                return PolyResCode.ErrNotCross;

            // 转换为顺时针方向
            //this.CW();
            //other.CW();

            List<NavNode> mainNode = new List<NavNode>();     //主多边形顶点
            List<NavNode> subNode = new List<NavNode>();      //需要合并的多边形顶点

            // init main nodes
            for (int i = 0; i < this.m_lstPoints.Count; i++)
            {
                NavNode currNode = new NavNode(this.m_lstPoints[i], false, true);
                if (i > 0)
                {
                    NavNode preNode = mainNode[i - 1];
                    preNode.next = currNode;
                }
                mainNode.Add(currNode);
            }

            // init sub nodes
            for (int j = 0; j < other.m_lstPoints.Count; j++)
            {
                NavNode currNode = new NavNode(other.m_lstPoints[j], false, false);
                if (j > 0)
                {
                    NavNode preNode = subNode[j - 1];
                    preNode.next = currNode;
                }
                subNode.Add(currNode);
            }

            int insCnt = 0;
            PolyResCode result = NavUtil.NavNodeIntersectPoint(mainNode, subNode, out insCnt);
            if (result == PolyResCode.Success && insCnt > 0)
            {
                if (insCnt % 2 != 0)
                {
                    return PolyResCode.ErrCrossNum;
                }
                else
                {
                    PolyResCode linkRes = NavUtil.LinkToPolygon(mainNode, subNode, ref polyRes);
                    return linkRes;
                }
            }

            return PolyResCode.ErrCrossNum;
        }

        // ======================================== 合并结束 ============================================ //




        /// <summary>
        /// 返回多边形包围盒
        /// </summary>
        /// <returns></returns>
        public Rect GetCoverRect()
        {
            Rect rect = new Rect(0, 0, 0, 0);

            for (int i = 0; i < this.m_lstPoints.Count; i++)
            {
                if (rect.xMin > this.m_lstPoints[i].x)
                    rect.xMin = this.m_lstPoints[i].x;
                if (rect.xMax < this.m_lstPoints[i].x)
                    rect.xMax = this.m_lstPoints[i].x;
                if (rect.yMin > this.m_lstPoints[i].y)
                    rect.yMin = this.m_lstPoints[i].y;
                if (rect.yMax < this.m_lstPoints[i].y)
                    rect.yMax = this.m_lstPoints[i].y;
            }
            return rect;
        }


        /// <summary>
        /// 获取标识
        /// </summary>
        /// <returns></returns>
        public int GetTag()
        {
            return this.m_iTag;
        }

        /// <summary>
        /// 设置标签
        /// </summary>
        /// <param name="tag"></param>
        public void SetTag( int tag )
        {
            this.m_iTag = tag;
        }

        /// <summary>
        /// 获取点集
        /// </summary>
        /// <returns></returns>
        public List<Vector2> GetPoints()
        {
            return new List<Vector2>(this.m_lstPoints);
        }

        /// <summary>
        /// 增加点位置
        /// </summary>
        /// <param name="pos"></param>
        public void AddPoints(Vector2 pos)
        {
            this.m_lstPoints.Add(pos);
        }

    }

}
