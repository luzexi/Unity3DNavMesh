using System;
using System.Collections.Generic;
using UnityEngine;


namespace Game.NavMesh
{

    /// <summary>
    /// 圆
    /// </summary>
    public class Circle
    {
        //圆心
        public Vector2 center;

        //半径
        public float radius;

        public Circle(Vector2 cen, float r)
        {
            center = cen;
            radius = r;
        }
    }
}
