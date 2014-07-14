using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;


//	NavEditAreaGroup.cs
//	Author: Lu Zexi
//	2014-07-08



namespace Game.NavMesh
{
	/// <summary>
	/// Nav edit area group.
	/// </summary>
	public class NavEditAreaGroup
	{
		//public int m_iID;
		public NavEditArea m_cFrameArea;
		public List<NavEditArea> m_lstArea = new List<NavEditArea>();
		//public int m_iLastAreaID = 1;

		/// <summary>
		/// Gets the area.
		/// </summary>
		/// <returns>The area.</returns>
		/// <param name="areaid">Areaid.</param>
		public NavEditArea GetArea( int areaIndex )
		{
			if(areaIndex >=0 && areaIndex < this.m_lstArea.Count )
				return this.m_lstArea[areaIndex];
			return null;
		}

		/// <summary>
		/// Adds the new area.
		/// </summary>
		public NavEditArea CreateArea()
		{
			NavEditArea area = new NavEditArea();
			this.m_lstArea.Add(area);
			return area;
		}

		/// <summary>
		/// Adds the frame area.
		/// </summary>
		/// <returns>The frame area.</returns>
		public NavEditArea CreateFrameArea()
		{
			if(this.m_cFrameArea != null )
			{
				this.m_lstArea.Remove(this.m_cFrameArea);
				this.m_cFrameArea.Destroy();
				this.m_cFrameArea = null;
			}
			NavEditArea area = new NavEditArea();
			this.m_cFrameArea = area;
			this.m_lstArea.Insert(0,this.m_cFrameArea);
			return this.m_cFrameArea;
		}

		/// <summary>
		/// Removes the frame area.
		/// </summary>
		public void RemoveFrameArea()
		{
			this.m_lstArea.Remove(this.m_cFrameArea);
			this.m_cFrameArea.Destroy();
			this.m_cFrameArea = null;
		}

		/// <summary>
		/// Removes the area.
		/// </summary>
		/// <param name="index">Index.</param>
		public void RemoveArea( int areaIndex )
		{
			this.m_lstArea[areaIndex].Destroy();
			this.m_lstArea.RemoveAt(areaIndex);
		}

		/// <summary>
		/// Destroy this instance.
		/// </summary>
		public void Destroy()
		{
			foreach( NavEditArea item in this.m_lstArea )
			{
				item.Destroy();
			}
			this.m_lstArea.Clear();
			this.m_cFrameArea = null;
		}

	}

}