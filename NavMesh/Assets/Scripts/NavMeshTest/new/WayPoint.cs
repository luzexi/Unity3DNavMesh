

using UnityEngine;



namespace Game.NavMesh
{

    /// <summary>
    /// 路径点
    /// </summary>
    public class WayPoint
    {
        private Vector2 m_cPoint;       //位置点
        private NavTriangle m_cTriangle;    //所在三角形

        public WayPoint(Vector2 pos, NavTriangle tri)
        {
            this.m_cPoint = pos;
            this.m_cTriangle = tri;
        }

        /// <summary>
        /// 获取路径点
        /// </summary>
        /// <returns></returns>
        public Vector2 GetPoint()
        {
            return this.m_cPoint;
        }

        /// <summary>
        /// 获取路径三角形
        /// </summary>
        /// <returns></returns>
        public NavTriangle GetTriangle()
        {
            return this.m_cTriangle;
        }

    }

}
