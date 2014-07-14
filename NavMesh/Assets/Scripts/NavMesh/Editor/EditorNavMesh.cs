using UnityEngine;
using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;
using Game.NavMesh;


//	EditorNavMesh.cs
//	Author: Lu  Zexi
//	2014-07-08


namespace Game.NavMesh
{
	[CustomEditor(typeof(NavMonoEditor))]
	public class EditorNavMesh : Editor
	{
		private const string UNWALK_EXTENSION = "unwalk";
		private const string NAVMESH_EXTENSION = "navmesh";
		private NavMonoEditor m_cNavMono = null;
		public static EditState m_eState = EditState.StateOther;
		public static GameObject m_cParent = null;

		private static int m_iLastSelGroup = -1;
		private static int m_iLastSelArea = -1;
		private static int m_iLastSelPoint = -1;

		public enum EditState
		{
			StateEditArea,
			StateFindArea,
			StateFinishArea,
			StateOther
		}

		// 焦点转移到编辑窗口
		private void FocusEditPanel()
		{
			if (SceneView.sceneViews.Count > 0)
			{
				SceneView myView = (SceneView)SceneView.sceneViews[0];
				myView.Focus();
			}
		}

		void OnEnable()
		{
			if (m_cNavMono == null)
			{
				if (target)
				{
					m_cNavMono = (NavMonoEditor)target;
				}
				else
					Debug.Log("需要一个脚本对象，必须挂载脚本上面");
			}
			if( m_cParent == null )
			{
				GameObject.DestroyImmediate(GameObject.Find("NavMeshParent"));
				m_cParent = new GameObject("NavMeshParent");
				m_cParent.transform.parent = this.m_cNavMono.transform;
			}
		}

		void OnDisable()
		{
			if (m_eState == EditState.StateEditArea || m_eState == EditState.StateFindArea)
			{
				if (this.m_cNavMono.gameObject != null)
				{
					Selection.activeGameObject = this.m_cNavMono.gameObject;
					
					//Debug.Log("请先关闭编辑状态(点击FinishArea按钮)，才能选择别的对象");
				}
			}
		}

		void OnDestroy()
		{
			this.m_cNavMono = null;
		}

		private static Vector3 sPos;
		private static Vector3 ePos;
		private static int m_iIndex = 0;
		void OnSceneGUI()
		{
			if (Event.current == null)
				return;
			
			Event e = Event.current;
			
			if (this.m_cNavMono == null)
				return;
			
			// only in edit mode will auto add point with mouse click;
			if (m_eState == EditState.StateEditArea)
			{
				if (e.button == 0 && e.type == EventType.MouseDown)
				{
					NavEditAreaGroup group = NavEditAreaManager.sInstance.GetGroup(this.m_cNavMono.m_iSelGroup);
					if (group == null)
						return;
					NavEditArea area = group.GetArea(this.m_cNavMono.m_iSelArea);
					if(area == null)
						return;
					
					Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
					//int layerMask = 1 << 9;
					RaycastHit hit;
					if (Physics.Raycast(ray.origin, ray.direction, out hit, Mathf.Infinity))
					{
						Vector3 placePos = hit.point;
						//placePos.y = pointHeight;

						// generate obj
						GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
						point.transform.position = placePos;
						point.transform.parent = m_cParent.transform;
						point.transform.localScale /= 5f;
						point.name = "point";
						area.Insert(point , this.m_cNavMono.m_iSelPoint);
						this.m_cNavMono.m_iSelPoint++;
						if( this.m_cNavMono.m_iSelPoint >= area.m_lstPoints.Count )
							this.m_cNavMono.m_iSelPoint = area.m_lstPoints.Count -1;
					}
					e.Use();
				}
			}
			else if( m_eState == EditState.StateFindArea )
			{
				if (e.button == 0 && e.type == EventType.MouseDown)
				{
					Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
					//int layerMask = 1 << 9;
					RaycastHit hit;
					if (Physics.Raycast(ray.origin, ray.direction, out hit, Mathf.Infinity))
					{
						if( m_iIndex == 0 )
						{
							sPos = hit.point;
							m_iIndex++;
						}
						else
						{
							m_iIndex = 0;
							ePos = hit.point;
							this.m_cNavMono.Seek(sPos , ePos);
						}

					}
				}
			}
		}

		private Vector2 groupUIPos;
		private Vector2 areaUIPos;
		private Vector2 pointUIPos;
		override public void OnInspectorGUI()
		{
			EditorGUILayout.BeginVertical();
			{
				NavEditAreaGroup group = NavEditAreaManager.sInstance.GetGroup(this.m_cNavMono.m_iSelGroup);
				NavEditArea area = null;
				if( group != null)
				{
					area = group.GetArea(this.m_cNavMono.m_iSelArea);
				}
				
				//========================= groups =============================
				GUILayout.BeginVertical();
				{
					GUILayout.Label("All Groups");
					groupUIPos = GUILayout.BeginScrollView( groupUIPos ,GUILayout.Height(150));
					{
						List<string> lst = new List<string>();
						for(int i = 0 ; i<NavEditAreaManager.sInstance.m_lstAreaGroup.Count ; i++)
						{
							lst.Add( "Group(" + i.ToString() + ")" );
						}
						this.m_cNavMono.m_iSelGroup = GUILayout.SelectionGrid(
							this.m_cNavMono.m_iSelGroup,
							lst.ToArray(),1);
						if( m_iLastSelGroup != this.m_cNavMono.m_iSelGroup )
						{
							m_iLastSelGroup = this.m_cNavMono.m_iSelGroup;
							this.m_cNavMono.m_iSelArea = 0;
							this.m_cNavMono.m_iSelPoint = 0;
							//FocusEditPanel();
						}
					}
					GUILayout.EndScrollView();
					
					GUILayout.BeginHorizontal();
					{
						if( GUILayout.Button("Create Group"))
						{
							Debug.Log("create group");
							NavEditAreaManager.sInstance.AddGroup();
							this.m_cNavMono.m_iSelGroup = NavEditAreaManager.sInstance.m_lstAreaGroup.Count-1;
						}
						if( GUILayout.Button("Delete Group"))
						{
							Debug.Log("delete group");
							NavEditAreaManager.sInstance.RemoveGroup(this.m_cNavMono.m_iSelGroup);
							this.m_cNavMono.m_iSelGroup--;
							if(this.m_cNavMono.m_iSelGroup < 0)
								this.m_cNavMono.m_iSelGroup = 0;
							this.m_cNavMono.m_iSelArea = 0;
							this.m_cNavMono.m_iSelPoint = 0;
						}
					}
					GUILayout.EndHorizontal();
				}
				GUILayout.EndVertical();
				
				//======================== areas ========================
				GUILayout.BeginVertical();
				{
					GUILayout.Label("All Areas");
					areaUIPos = GUILayout.BeginScrollView( areaUIPos ,GUILayout.Height(150));
					{
						if(group != null )
						{
							List<string> lst = new List<string>();
							for(int i = 0 ; i< group.m_lstArea.Count ; i++)
							{
								lst.Add( "Area(" + i.ToString() + ")" );
							}
							this.m_cNavMono.m_iSelArea = GUILayout.SelectionGrid(
								this.m_cNavMono.m_iSelArea,
								lst.ToArray(),1);
							if( m_iLastSelArea != this.m_cNavMono.m_iSelArea )
							{
								m_iLastSelArea = this.m_cNavMono.m_iSelArea;
								this.m_cNavMono.m_iSelPoint = 0;
							}
//							FocusEditPanel();
						}
					}
					GUILayout.EndScrollView();
					
					GUILayout.BeginHorizontal();
					{
						GUI.enabled = group != null;
						if( GUILayout.Button("Create Frame Area"))
						{
							Debug.Log("create frame area");
							group.CreateFrameArea();
							this.m_cNavMono.m_iSelArea = 0;
							this.m_cNavMono.m_iSelPoint = 0;
						}
						if( GUILayout.Button("Create Area"))
						{
							Debug.Log("create area");
							group.CreateArea();
							this.m_cNavMono.m_iSelArea = group.m_lstArea.Count-1; 
							this.m_cNavMono.m_iSelPoint = 0;
						}
						if( GUILayout.Button("Delete Area"))
						{
							Debug.Log("delete area");
							group.RemoveArea(this.m_cNavMono.m_iSelArea);
							this.m_cNavMono.m_iSelArea--;
							if(this.m_cNavMono.m_iSelArea < 0 )
								this.m_cNavMono.m_iSelArea = 0;
							this.m_cNavMono.m_iSelPoint = 0;
						}
					}
					GUILayout.EndHorizontal();
				}
				GUILayout.EndVertical();
				
				//====================== points ==============================
				GUILayout.BeginVertical();
				{
					GUILayout.Label("All Points");
					pointUIPos = GUILayout.BeginScrollView( pointUIPos ,GUILayout.Height(150));
					{
						if(group != null && area != null)
						{
							List<string> lst = new List<string>();
							for(int i = 0 ; i< area.m_lstPoints.Count ; i++)
							{
								lst.Add( "Point(" + i + ")");
							}
							this.m_cNavMono.m_iSelPoint = GUILayout.SelectionGrid(
								this.m_cNavMono.m_iSelPoint,
								lst.ToArray(),1);
//							FocusEditPanel();
						}
					}
					GUILayout.EndScrollView();
					GUI.enabled = m_eState != EditState.StateEditArea;
					if(GUILayout.Button("Editor Point",GUILayout.Height(30)))
					{
						m_eState = EditState.StateEditArea;
					}
					GUI.enabled = m_eState != EditState.StateFindArea;
					if(GUILayout.Button("Find Path",GUILayout.Height(30)))
					{
						m_eState = EditState.StateFindArea;
					}
					GUI.enabled = (m_eState == EditState.StateEditArea || m_eState == EditState.StateFindArea);
					if(GUILayout.Button("Finish Point",GUILayout.Height(30)))
					{
						m_eState = EditState.StateFinishArea;
					}
					if(GUILayout.Button("Delete Point",GUILayout.Height(30)))
					{
						Debug.Log("delete point");
						if(area != null )
						{
							area.RemoveAt(this.m_cNavMono.m_iSelPoint);
							this.m_cNavMono.m_iSelPoint--;
							if(this.m_cNavMono.m_iSelPoint < 0 )
								this.m_cNavMono.m_iSelPoint = 0;
						}
					}
					GUI.enabled = true;
				}
				GUILayout.EndVertical();
				
				//===================== ============================
			}
			EditorGUILayout.EndVertical();


			GUILayout.Label("NavMesh");
			this.m_cNavMono.m_bShowMesh = GUILayout.Toggle(this.m_cNavMono.m_bShowMesh , "NavMesh Show");
			this.m_cNavMono.m_bShowArea = GUILayout.Toggle(this.m_cNavMono.m_bShowArea , "Area Show");
			if( GUILayout.Button("Create NavMesh",GUILayout.Height(30)))
			{
				this.m_cNavMono.CreateNavMesh();
			}

			GUILayout.Label("Area Group Save/Load");
			GUILayout.BeginHorizontal();
			{
				if( GUILayout.Button("Save AreaGroup",GUILayout.Height(30)))
				{
					Debug.Log("save area group");
					string pathfile = EditorUtility.SaveFilePanel("Save Area Group" , Application.dataPath , "map" , UNWALK_EXTENSION);
					NavEditAreaManager.sInstance.SaveAreaGroup(pathfile);
				}
				if( GUILayout.Button("Load AreaGroup",GUILayout.Height(30)))
				{
					Debug.Log("load area group");
					string pathfile = EditorUtility.OpenFilePanel("Open Area Group" , Application.dataPath , UNWALK_EXTENSION);
					NavEditAreaManager.sInstance.LoadAreaGroup(pathfile , m_cParent);
				}
			}
			GUILayout.EndHorizontal();

			GUILayout.Label("NavMesh Save/Load");
			GUILayout.BeginHorizontal();
			{
				if(GUILayout.Button("Save NavMesh",GUILayout.Height(30)))
				{
					Debug.Log("save navmesh");
					string pathfile = EditorUtility.SaveFilePanel("Save NavMesh" , Application.dataPath , "map" , NAVMESH_EXTENSION);
					this.m_cNavMono.SaveNavMesh(pathfile);
				}
				if(GUILayout.Button("Load NavMesh",GUILayout.Height(30)))
				{
					Debug.Log("load navmesh");
					string pathfile = EditorUtility.OpenFilePanel("Open NavMesh" , Application.dataPath , NAVMESH_EXTENSION);
					this.m_cNavMono.LoadNavMesh(pathfile);
				}
			}
			GUILayout.EndHorizontal();

		}
	}

}
