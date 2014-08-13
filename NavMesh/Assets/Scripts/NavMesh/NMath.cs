
using UnityEngine;
using System;


//  NMath.cs
//  Author: Lu Zexi
//  2013-10-04



namespace Game.NavMesh
{

    /// <summary>
    /// NavMesh专用数学类
    /// </summary>
    public class NMath
    {
        private const float EPSILON = 1e-005f;  //最小常量

        /// <summary>
        /// 返回顶角在o点，起始边为os，终止边为oe的夹角, 即∠soe (单位：弧度) 
        /// 矢量os 在矢量 oe的顺时针方向,返回正值;否则返回负值 
        /// </summary>
        /// <param name="s"></param>
        /// <param name="o"></param>
        /// <param name="e"></param>
        /// <returns>返回弧度</returns>
        public static double LineRadian(Vector2 s, Vector2 o, Vector2 e)
        {
            float dx12 = s.x - o.x;
            float dy12 = s.y - o.y;
            float dx32 = e.x - o.x;
            float dy32 = e.y - o.y;

            //角度计算公式s→ * e→ = |s|*|e|*cosθ

            double cosfi = dx12 * dx32 + dy12 * dy32;

            float norm = (dx12 * dx12 + dy12 * dy12) * (dx32 * dx32 + dy32 * dy32);

            cosfi /= Math.Sqrt(norm);

            if (cosfi >= 1.0) return 0;
            if (cosfi <= -1.0) return -Math.PI;

            double angleRadian = Math.Acos(cosfi);

            // 说明矢量os 在矢量 oe的顺时针方向 
            if (dx12 * dy32 - dy12 * dx32 > 0)
                return angleRadian;

            return -angleRadian;

        }

        /// <summary>
        /// 获得外接圆的包围盒
        /// </summary>
        /// <param name="circle"></param>
        /// <returns></returns>
        public static Rect GetCircleBoundBox(Circle circle)
        {
            Rect bBox = new Rect();
            bBox.xMin = circle.center.x - circle.radius;
            bBox.xMax = circle.center.x + circle.radius;
            bBox.yMin = circle.center.y - circle.radius;
            bBox.yMax = circle.center.y + circle.radius;
            return bBox;
        }

        /// <summary>
        /// 返回三角形的外接圆
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <returns></returns>
        public static Circle CreateCircle(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            if (!NMath.IsEqualZero(p1.y - p2.y) ||
                !NMath.IsEqualZero(p2.y - p3.y))
            {
                double yc, xc;
                float m1 = -(p2.x - p1.x) / (p2.y - p1.y);
                float m2 = -(p3.x - p2.x) / (p3.y - p2.y);
                double mx1 = (p1.x + p2.x) / 2.0;
                double mx2 = (p2.x + p3.x) / 2.0;
                double my1 = (p1.y + p2.y) / 2.0;
                double my2 = (p2.y + p3.y) / 2.0;

                if (NMath.IsEqualZero(p1.y - p2.y))
                {
                    xc = (p2.x + p1.x) / 2.0;
                    yc = m2 * (xc - mx2) + my2;
                }
                else if (NMath.IsEqualZero(p3.y - p2.y))
                {
                    xc = (p3.x + p2.x) / 2.0;
                    yc = m1 * (xc - mx1) + my1;
                }
                else
                {
                    xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
                    yc = m1 * (xc - mx1) + my1;
                }

                double dx = p2.x - xc;
                double dy = p2.y - yc;
                double rsqr = dx * dx + dy * dy;
                double r = Math.Sqrt(rsqr);

                return new Circle(new Vector2((float)xc, (float)yc), (float)r);
            }
            return new Circle(new Vector2(0, 0), 0);
        }

		/// <summary>
		/// Calculate the circle tangency.
		/// </summary>
		/// <returns>The circle tangency.</returns>
		/// <param name="c">C.</param>
		/// <param name="st">St.</param>
		public static Vector2[] CalCircleTangency( Circle c , Vector2 st )
		{
			Vector2 dir;
			float dis = (c.center - st).magnitude;
			float temp= Mathf.Sqrt(dis*dis-c.radius*c.radius);
			float sina=temp/dis;
			float cosa=c.radius/dis;
			dir.x=(st.x-c.center.x)/dis*c.radius;
			dir.y=(st.y-c.center.y)/dis*c.radius;

			Vector2[] res = new Vector2[2];

			res[0].x = c.center.x+(dir.x*cosa-dir.y*sina);
			res[0].y = c.center.y+(dir.x*sina+dir.y*cosa);
			
			res[1].x = c.center.x+(dir.x*cosa+dir.y*sina);
			res[1].y = c.center.y+(-dir.x*sina+dir.y*cosa);

			return res;
		}

        /// <summary>
        /// 将线段终点延长
        /// </summary>
        /// <param name="startPos">起始点</param>
        /// <param name="tarPos">终点</param>
        /// <param name="length">延长长度</param>
        /// <returns></returns>
        public static Vector2 ExtendPos(Vector2 startPos, Vector2 tarPos, int length)
        {
            Vector2 newPos = tarPos;
            float slopeRate = Math.Abs((tarPos.y - startPos.y) / (tarPos.x - startPos.x));
            float xLength, yLength;
            if (slopeRate < 1)
            {
                yLength = length;
                xLength = length / slopeRate;
            }
            else
            {
                xLength = length;
                yLength = length * slopeRate;
            }

            if (tarPos.x > startPos.x)
                newPos.x += xLength;
            else
                newPos.x -= xLength;

            if (tarPos.y > startPos.y)
                newPos.y += yLength;
            else
                newPos.y -= yLength;

            return newPos;
        }

        /// <summary>
        /// 判断矩形是否相交
        /// </summary>
        /// <param name="rec1">矩形1</param>
        /// <param name="rec2">矩形2</param>
        /// <returns>是否相交</returns>
        public static bool CheckCross(Rect rec1, Rect rec2)
        {
            Rect ret = new Rect();
            ret.xMin = Mathf.Max(rec1.xMin, rec2.xMin);
            ret.xMax = Mathf.Min(rec1.xMax, rec2.xMax);
            ret.yMin = Mathf.Max(rec1.yMin, rec2.yMin);
            ret.yMax = Mathf.Min(rec1.yMax, rec2.yMax);

            if (ret.xMin > ret.xMax || ret.yMin > ret.yMax)
            {
                // no intersection, return empty
                return false;
            }
            return true;
        }

        /// <summary>
        /// 根据线段生成矩形
        /// </summary>
        /// <param name="linePath">线段</param>
        /// <returns>矩形</returns>
        public static Rect LineRect(Line2D linePath)
        {
            Rect lineRect = new Rect();

            if (linePath.GetStartPoint().x < linePath.GetEndPoint().x)
                lineRect.xMin = linePath.GetStartPoint().x;
            else
                lineRect.xMin = linePath.GetEndPoint().x;

            if (linePath.GetStartPoint().y < linePath.GetEndPoint().y)
                lineRect.yMin = linePath.GetStartPoint().y;
            else
                lineRect.yMin = linePath.GetEndPoint().y;

            lineRect.width = Math.Abs(linePath.GetEndPoint().x - linePath.GetStartPoint().x);
            lineRect.height = Math.Abs(linePath.GetEndPoint().y - linePath.GetStartPoint().y);

            return lineRect;
        }

        /// <summary>
        /// 检查线段是否相交
        /// </summary>
        /// <param name="sp1">line1 start point</param>
        /// <param name="ep1">line1 end point</param>
        /// <param name="sp2">line2 start point</param>
        /// <param name="ep2">line2 end point</param>
        /// <returns>is cross</returns>
        public static bool CheckCross(Vector2 sp1, Vector2 ep1, Vector2 sp2, Vector2 ep2)
        {
            if (Math.Max(sp1.x, ep1.x) < Math.Min(sp2.x, ep2.x))
            {
                return false;
            }
            if (Math.Min(sp1.x, ep1.x) > Math.Max(sp2.x, ep2.x))
            {
                return false;
            }
            if (Math.Max(sp1.y, ep1.y) < Math.Min(sp2.y, ep2.y))
            {
                return false;
            }
            if (Math.Min(sp1.y, ep1.y) > Math.Max(sp2.y, ep2.y))
            {
                return false;
            }

            float temp1 = CrossProduct((sp1 - sp2), (ep2 - sp2)) * CrossProduct((ep2 - sp2), (ep1 - sp2));
            float temp2 = CrossProduct((sp2 - sp1), (ep1 - sp1)) * CrossProduct((ep1 - sp1), (ep2 - sp1));

            if ((temp1 >= 0) && (temp2 >= 0))
            {
                return true;
            }

            return false;
        }

		/// <summary>
		/// Checks the cross.
		/// </summary>
		/// <returns><c>true</c>, if cross was checked, <c>false</c> otherwise.</returns>
		/// <param name="line">Line.</param>
		/// <param name="tri">Tri.</param>
		public static bool CheckCross( Line2D line , Triangle tri )
		{
			for(int i = 0 ; i < 3 ; i++ )
			{
				Line2D lineTri = tri.GetSide(i);
				if( CheckCross(line.GetStartPoint() , line.GetEndPoint() ,
				               lineTri.GetStartPoint() , lineTri.GetEndPoint())
				   )
				{
					return true;
				}
			}
			return false;
		}

		/// <summary>
		/// Checks the cross.
		/// </summary>
		/// <returns><c>true</c>, if cross was checked, <c>false</c> otherwise.</returns>
		/// <param name="line">Line.</param>
		/// <param name="cir">Cir.</param>
		public static bool CheckCross( Line2D line , Circle cir )
		{
			CheckCross(line , cir.center , cir.radius);
		}

		/// <summary>
		/// Checks the cross.
		/// </summary>
		/// <returns><c>true</c>, if cross was checked, <c>false</c> otherwise.</returns>
		/// <param name="line">Line.</param>
		/// <param name="pos">Position.</param>
		/// <param name="radius">Radius.</param>
		public static bool CheckCross( Line2D line , Vector2 pos , int radius )
		{
			float fDis = line.GetLength();
			
			Vector2 d;
			d.x = (line.GetEndPoint().x - line.GetStartPoint().x) / fDis;
			d.y = (line.GetEndPoint().y - line.GetStartPoint().y) / fDis;
			
			Vector2 E;
			E.x = pos.x - line.GetStartPoint().x;
			E.y = pos.y - line.GetStartPoint().y;
			
			float a = E.x * d.x + E.y * d.y;
			float a2 = a * a;
			
			float e2 = E.x * E.x + E.y * E.y;
			
			float r2 = radius * radius;
			
			if ((r2 - e2 + a2) < 0)  
			{  
				return false;  
			}
			return true;
		}

        /// <summary>
        /// r=multiply(sp,ep,op),得到(sp-op)*(ep-op)的叉积 
        ///	r>0:ep在矢量opsp的逆时针方向； 
        ///	r=0：opspep三点共线； 
        ///	r<0:ep在矢量opsp的顺时针方向 
        /// </summary>
        /// <param name="p1">opsp</param>
        /// <param name="p2">opep</param>
        /// <returns></returns>
        public static float CrossProduct(Vector2 p1, Vector2 p2)
        {
            return (p1.x * p2.y - p1.y * p2.x);
        }

        /// <summary>
        /// 两个矢量是否相同
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static bool IsEqual(Vector2 v1, Vector2 v2)
        {
            return IsEqualZero(v1 - v2);
        }

        /// <summary>
        /// 点是否等于0
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static bool IsEqualZero(Vector2 data)
        {
            if (IsEqualZero(data.x) && IsEqualZero(data.y))
                return true;
            else
                return false;
        }

        /// <summary>
        /// 检查浮点数误差
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static bool IsEqualZero(double data)
        {
            if (Math.Abs(data) <= EPSILON)
                return true;
            else
                return false;
        }
    }
}
