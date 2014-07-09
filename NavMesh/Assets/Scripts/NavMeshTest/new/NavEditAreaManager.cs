using UnityEngine;
using System.Collections;
using System.Collections.Generic;


//	NavEditAreaManager.cs
//	Author: Lu Zexi
//	2014-07-08


namespace Game.NavMesh
{
	/// <summary>
	/// Nav edit area manager.
	/// </summary>
	public class NavEditAreaManager
	{
		public int m_iLastGroupID = 0;
		//public List<NavEditArea> m_lstArea = new List<NavEditArea>();
		public List<NavEditAreaGroup> m_lstAreaGroup = new List<NavEditAreaGroup>();

		private static NavEditAreaManager s_cInstance;
		public static NavEditAreaManager sInstance
		{
			get
			{
				if( s_cInstance == null )
					s_cInstance = new NavEditAreaManager();
				return s_cInstance;
			}
		}

		public NavEditAreaManager()
		{
			//
		}

		/// <summary>
		/// Gets the group.
		/// </summary>
		/// <returns>The group.</returns>
		/// <param name="groupIndex">Group index.</param>
		public NavEditAreaGroup GetGroup( int groupIndex )
		{
			if( groupIndex >= 0 && groupIndex < this.m_lstAreaGroup.Count )
				return this.m_lstAreaGroup[groupIndex];
			return null;
		}

		/// <summary>
		/// Adds the group.
		/// </summary>
		public void AddGroup()
		{
			NavEditAreaGroup group = new NavEditAreaGroup();
			//group.m_iID = this.m_iLastGroupID++;
			this.m_lstAreaGroup.Add(group);
		}

		/// <summary>
		/// Removes the group.
		/// </summary>
		/// <param name="groupid">Groupid.</param>
		public void RemoveGroup( int groupindex )
		{
			this.m_lstAreaGroup[groupindex].Destroy();
			this.m_lstAreaGroup.RemoveAt(groupindex);
		}

		/// <summary>
		/// Gets the area.
		/// </summary>
		/// <returns>The area.</returns>
		/// <param name="id">Identifier.</param>
		public NavEditArea GetArea( int groupIndex , int areaIndex )
		{
			if( groupIndex >=0 && groupIndex < this.m_lstAreaGroup.Count)
			{
				NavEditAreaGroup group = this.m_lstAreaGroup[groupIndex];
				return group.GetArea(areaIndex);
			}
			return null;
		}

		/// <summary>
		/// Adds the frame area.
		/// </summary>
		/// <returns>The frame area.</returns>
		/// <param name="groupIndex">Group index.</param>
		public NavEditArea AddFrameArea( int groupIndex )
		{
			return this.m_lstAreaGroup[groupIndex].CreateFrameArea();
		}

		/// <summary>
		/// Adds the new area.
		/// </summary>
		public NavEditArea AddNewArea( int groupIndex )
		{
			return this.m_lstAreaGroup[groupIndex].CreateArea();
		}

		/// <summary>
		/// Removes the area.
		/// </summary>
		/// <param name="areaid">Areaid.</param>
		public void RemoveArea( int groupIndex , int areaIndex )
		{
			this.m_lstAreaGroup[groupIndex].RemoveArea(areaIndex);
		}

		/// <summary>
		/// Adds the new point.
		/// </summary>
		/// <param name="areaid">Areaid.</param>
		/// <param name="obj">Object.</param>
		public void AddNewPoint( int groupIndex , int areaIndex , GameObject obj )
		{
			this.m_lstAreaGroup[groupIndex].m_lstArea[areaIndex].Add(obj);
		}

		/// <summary>
		/// Inserts the point.
		/// </summary>
		/// <param name="areaid">Areaid.</param>
		/// <param name="pointIndex">Point index.</param>
		/// <param name="obj">Object.</param>
		public void InsertPoint( int groupIndex , int areaIndex , int pointIndex , GameObject obj )
		{
			this.m_lstAreaGroup[groupIndex].m_lstArea[areaIndex].Insert(obj ,pointIndex);
		}

		/// <summary>
		/// Removes the point.
		/// </summary>
		/// <param name="areaid">Areaid.</param>
		/// <param name="pointIndex">Point index.</param>
		public void RemovePoint( int groupIndex, int areaIndex , int pointIndex )
		{
			this.m_lstAreaGroup[groupIndex].m_lstArea[areaIndex].RemoveAt(pointIndex);
		}

		/// <summary>
		/// Checks all points.
		/// </summary>
		public void CheckAllPoints()
		{
			foreach( NavEditAreaGroup group in this.m_lstAreaGroup )
			{
				foreach (NavEditArea item in group.m_lstArea )
				{
					int pointNum = item.m_lstPoints.Count;
					for (int i = 0; i < pointNum; i++)
					{
						if (item.m_lstPoints[i] == null)
						{
							item.RemoveAt(i);
							i--;
							pointNum--;
						}
					}
				}
			}
		}

		/// <summary>
		/// Saves the area.
		/// </summary>
		/// <param name="path">Path.</param>
		public void SaveArea( string path )
		{
			//
		}

		/// <summary>
		/// Loads the area.
		/// </summary>
		/// <param name="path">Path.</param>
		/// <param name="parent">Parent.</param>
		public void LoadArea( string path , GameObject parent )
		{
			//
		}
	}
}
