using UnityEngine;
using System.Collections;
using System.Collections.Generic;


//	NavMonoEditor.cs
//	Author: Lu Zexi
//	2014-07-08



namespace Game.NavMesh
{
	/// <summary>
	/// Nav mono editor.
	/// </summary>
	public class NavMonoEditor : MonoBehaviour
	{
		private List<Triangle> m_lstTriangle = new List<Triangle>();

		public bool m_bShowMesh = true;	//is show mesh
		public int m_iSelGroup;	//the selected group
		public int m_iSelArea;	//the selected area
		public int m_iSelPoint;	//the selected point

		// Use this for initialization
		void Start ()
		{
			//
		}
	
		// Update is called once per frame
		void Update ()
		{
			//
		}

		void OnDrawGizmos()
		{
			DrawAllAreas();
			DrawSelectPoint();
			DrawNavMesh();
		}

//========================== draw area ========================================

		/// <summary>
		/// Draws all areas.
		/// </summary>
		private void DrawAllAreas()
		{
			for( int i = 0 ; i<NavEditAreaManager.sInstance.m_lstAreaGroup.Count ; i++ )
			{
				NavEditAreaGroup group = NavEditAreaManager.sInstance.GetGroup(i);
				bool selectGroup = this.m_iSelGroup == i;
				for(int j = 0 ; j< group.m_lstArea.Count ; j++ )
				{
					NavEditArea area = group.GetArea(j);
					bool selectArea = false;
					if(selectGroup)
						selectArea = this.m_iSelArea == j;
					DrawArea(area , selectArea);
				}
			}
		}

		/// <summary>
		/// Draws the area.
		/// </summary>
		/// <param name="areaid">Areaid.</param>
		private void DrawArea( NavEditArea area , bool selectarea )
		{	
			if(area == null )
				return;

			List<GameObject> allPoints = area.m_lstPoints;
			if (allPoints.Count <= 0)
				return;
			
			if ( selectarea )
				Gizmos.color = Color.red;
			else
				Gizmos.color = Color.green;
			
			for (int i = 0; i < allPoints.Count; i++)
			{
				if (allPoints[i] == null)
				{
					NavEditAreaManager.sInstance.CheckAllPoints();
					Debug.LogError("there a null in the point gameobj lst." +i);
					return;
				}
				else
				{
					if (i != allPoints.Count - 1)
					{
						if (allPoints[i + 1] == null)
						{
							NavEditAreaManager.sInstance.CheckAllPoints();
							Debug.LogError("there a null in the point gameobj lst.");
							return;
						}
						Gizmos.DrawLine(allPoints[i].transform.position, allPoints[i + 1].transform.position);
					}
					else
					{
						Gizmos.DrawLine(allPoints[i].transform.position, allPoints[0].transform.position);
					}
				}
			}
		}

//========================= draw select point ========================
		/// <summary>
		/// Draws the select point.
		/// </summary>
		private void DrawSelectPoint()
		{
			NavEditArea area = NavEditAreaManager.sInstance.GetArea( this.m_iSelGroup , this.m_iSelArea);
			if( area == null )
				return;

			if( this.m_iSelPoint < 0 || this.m_iSelPoint >= area.m_lstPoints.Count )
				return;
			
			Gizmos.DrawIcon(area.m_lstPoints[this.m_iSelPoint].transform.position, "point.tif");
		}

//======================= draw NavMesh ======================================

		/// <summary>
		/// Draws the nav mesh.
		/// </summary>
		private void DrawNavMesh()
		{
			if (this.m_bShowMesh)
			{
				if (this.m_lstTriangle.Count != 0)
				{
					foreach (Triangle tri in m_lstTriangle)
					{
						if (tri.GetGroupID() == 0)
						{
							Gizmos.color = Color.blue;
							//continue;
						}
						else if (tri.GetGroupID() == 1)
						{
							Gizmos.color = Color.grey;
							//continue;
						}
						else if (tri.GetGroupID() == 2)
							Gizmos.color = Color.black;

						Vector3 p1 = new Vector3(tri.GetPoint(0).x, 0, tri.GetPoint(0).y);
						Vector3 p2 = new Vector3(tri.GetPoint(1).x, 0, tri.GetPoint(1).y);
						Vector3 p3 = new Vector3(tri.GetPoint(2).x, 0, tri.GetPoint(2).y);
						Gizmos.DrawLine(p1, p2);
						Gizmos.DrawLine(p2, p3);
						Gizmos.DrawLine(p3, p1);
					}
				}
			}
		}

//======================== create navmesh ==========================

		/// <summary>
		/// Gets the un walk areas.
		/// </summary>
		/// <returns>The un walk areas.</returns>
		private List<Polygon> GetUnWalkAreas( List<NavEditArea> lst )
		{
			List<Polygon> areas = new List<Polygon>();
			for (int i = 0; i < lst.Count; i++)
			{
				List<GameObject> allPoints = lst[i].m_lstPoints;
				List<Vector2> allVecPnts = new List<Vector2>();
				for (int j = 0; j < allPoints.Count; j++)
				{
					Vector2 pos = new Vector2(allPoints[j].transform.position.x, allPoints[j].transform.position.z);
					allVecPnts.Add(pos);
				}
				areas.Add(new Polygon(allVecPnts));
			}
			
			return areas;
		}
		
		/// <summary>
		/// 创建导航网格
		/// </summary>
		public void CreateNavMesh()
		{
			Debug.Log("start...");

			List<Triangle> lstTri = new List<Triangle>();
			this.m_lstTriangle.Clear();
			foreach( NavEditAreaGroup group in NavEditAreaManager.sInstance.m_lstAreaGroup )
			{
				lstTri.Clear();
				List<Polygon> areas = GetUnWalkAreas(group.m_lstArea);
				Debug.Log(areas.Count + " polygon count");
				NavResCode genResult = NavMeshGen.Instance.CreateNavMesh(areas , ref lstTri );
				foreach( Triangle item in lstTri )
				{
					this.m_lstTriangle.Add(item);
				}
				if (genResult != NavResCode.Success)
					Debug.LogError("faile");
			}

			Debug.Log("triangle count "+ this.m_lstTriangle.Count);
			foreach (Triangle item in this.m_lstTriangle)
				Debug.Log(item.GetPoint(0) + " -- " + item.GetPoint(1) + " -- " + item.GetPoint(2));

			Debug.Log("end!");
		}

//========================= save or load navmesh =========================

		public void SaveNavMesh( string path )
		{
			//
		}

		public void LoadNavMesh( string path )
		{
			//
		}
	}
}