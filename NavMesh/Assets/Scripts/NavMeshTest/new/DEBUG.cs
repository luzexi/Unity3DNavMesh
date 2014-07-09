
using UnityEngine;
using System;


//  DEBGU.cs
//  Author: Lu Zexi
//  2013-10-04


namespace Game.NavMesh
{
    public class DEBUG
    {
        /// <summary>
        /// 记录打印
        /// </summary>
        /// <param name="str"></param>
        public static void LOG(string str)
        {
            Debug.Log(str);
        }

        /// <summary>
        /// 错误记录打印
        /// </summary>
        /// <param name="str"></param>
        public static void ERROR(string str)
        {
            Debug.LogError(str);
        }

        /// <summary>
        /// 异常记录打印
        /// </summary>
        /// <param name="ex"></param>
        public static void ERROR(Exception ex)
        {
            Debug.LogError(ex.StackTrace);
        }
    }
}
