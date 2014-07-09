using UnityEngine;
using System.Collections;
using System.Collections.Generic;

//	NavEditArea.cs
//	Author: Lu Zexi
//	2014-07-08


namespace Game.NavMesh
{
	/// <summary>
	/// Nav edit area.
	/// </summary>
	public class NavEditArea
	{
		//public int m_iAreaID;	//area id
		public List<GameObject> m_lstPoints = new List<GameObject>();	//the points

		/// <summary>
		/// Adds the point.
		/// </summary>
		/// <param name="obj">Object.</param>
		public void Add( GameObject obj )
		{
			Debug.Log(obj.name);
			this.m_lstPoints.Add(obj);
			Debug.Log(obj.name);
		}

		/// <summary>
		/// Inserts the point.
		/// </summary>
		/// <param name="obj">Object.</param>
		/// <param name="index">Index.</param>
		public void Insert( GameObject obj , int index )
		{
			if( index >= 0 && index < this.m_lstPoints.Count)
			{
				this.m_lstPoints.Insert(index ,obj);
			}
		}

		/// <summary>
		/// Removes at index.
		/// </summary>
		/// <param name="index">Index.</param>
		public bool RemoveAt(int index)
		{
			if( index >= 0 && index < this.m_lstPoints.Count )
			{
				GameObject.DestroyImmediate(this.m_lstPoints[index]);
				this.m_lstPoints.RemoveAt(index);
				return true;
			}
			return false;
		}

		/// <summary>
		/// Checks the line cross.
		/// </summary>
		/// <returns><c>true</c>, if line cross was checked, <c>false</c> otherwise.</returns>
		public bool CheckLineCross()
		{
			Vector2 p1s, p1e, p2s, p2e;
			
			int iPointCount = this.m_lstPoints.Count;
			
			// 如果点的列表中只有两个点, 可以添加新点.
			if (iPointCount <= 2)
			{
				return true;
			}
			
			for (int i = 0; i < iPointCount - 2; i++)
			{
				p1s.x = this.m_lstPoints[i].transform.position.x;
				p1s.y = this.m_lstPoints[i].transform.position.z;
				p1e.x = this.m_lstPoints[i + 1].transform.position.x;
				p1e.y = this.m_lstPoints[i + 1].transform.position.z;
				
				for (int j = i + 2; j < iPointCount; j++)
				{
					if (j != iPointCount - 1)
					{
						p2s.x = this.m_lstPoints[j].transform.position.x;
						p2s.y = this.m_lstPoints[j].transform.position.z;
						p2e.x = this.m_lstPoints[j + 1].transform.position.x;
						p2e.y = this.m_lstPoints[j + 1].transform.position.z;
					}
					else
					{
						p2s.x = this.m_lstPoints[j].transform.position.x;
						p2s.y = this.m_lstPoints[j].transform.position.z;
						p2e.x = this.m_lstPoints[0].transform.position.x;
						p2e.y = this.m_lstPoints[0].transform.position.z;
						
						if (0 == i)
						{
							continue;
						}
					}
					
					if (NMath.CheckCross(p1s, p1e, p2s, p2e))
					{
						return false;
					}
					
				}
			}
			return true;
		}

		/// <summary>
		/// Destroy this instance.
		/// </summary>
		public void Destroy()
		{
			foreach( GameObject item in this.m_lstPoints )
			{
				GameObject.DestroyImmediate(item);
			}
			this.m_lstPoints.Clear();
		}
	}

}