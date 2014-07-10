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
		private List<Triangle> m_lstTriangle = new List<Triangle>();	//navmesh triangle
		private List<Vector2> m_lstFindPath = new List<Vector2>();	//findPath;

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
			DrawFindPath();
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

			Vector3 pos = area.m_lstPoints[this.m_iSelPoint].transform.position;
			pos.y += 1;
			Gizmos.DrawIcon( pos , "010.tif" );
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
			Debug.Log("end!");
		}

//========================= save or load navmesh =========================

		/// <summary>
		/// Saves the nav mesh.
		/// </summary>
		/// <param name="path">Path.</param>
		public void SaveNavMesh( string path )
		{
			NavResCode code = NavMeshGen.Instance.SaveNavMeshToFile(path , this.m_lstTriangle);
			if( code != NavResCode.Success )
			{
				Debug.LogError( "save navmesh error: " + code.ToString());
			}
		}

		/// <summary>
		/// Loads the nav mesh.
		/// </summary>
		/// <param name="path">Path.</param>
		public void LoadNavMesh( string path )
		{
			List<Triangle> lst;
			NavResCode code = NavMeshGen.Instance.LoadNavMeshFromFile(path , out lst);
			this.m_lstTriangle = lst;

			if(code != NavResCode.Success)
			{
				Debug.LogError("load navmesh error: " + code.ToString());
			}
		}

//=============================== seeker test =================================		
		/// <summary>
		/// Draws the find path.
		/// </summary>
		public void DrawFindPath()
		{
			for( int i = 1 ; i < this.m_lstFindPath.Count ; i++)
			{
				Gizmos.color = Color.blue;
				Vector3 spos = new Vector3(this.m_lstFindPath[i].x , 0 , this.m_lstFindPath[i].y);
				Vector3 epos = new Vector3(this.m_lstFindPath[i-1].x , 0 , this.m_lstFindPath[i-1].y);
				Gizmos.DrawLine(spos , epos);
			}
		}

		/// <summary>
		/// Seek the specified sPos and ePos.
		/// </summary>
		/// <param name="sPos">S position.</param>
		/// <param name="ePos">E position.</param>
		public void Seek( Vector3 sPos , Vector3 ePos )
		{
			//
			List<NavTriangle> lst = new List<NavTriangle>();
			foreach( Triangle item in this.m_lstTriangle )
			{
				NavTriangle navTri = item.CloneNavTriangle();
				lst.Add(navTri);
				//Debug.Log( "src " + item.GetGroupID() + " :" + item.GetNeighbor(0)+"--" + item.GetNeighbor(1) + "--" + item.GetNeighbor(2) );
				//Debug.Log( "tar " + navTri.GetGroupID() + " :" + navTri.GetNeighbor(0)+"--" + navTri.GetNeighbor(1) + "--" + navTri.GetNeighbor(2) );
			}
			Seeker.GetInstance().NavMeshData = lst;
			List<Vector2> lstpath;
			Vector2 ssPos = new Vector2(sPos.x , sPos.z);
			Vector2 eePos = new Vector2(ePos.x , ePos.z);
			Seeker.GetInstance().Seek(ssPos ,eePos ,out lstpath , 1);
			Debug.Log(lstpath.Count + " lstpath");
			this.m_lstFindPath = lstpath;
		}
	}

}