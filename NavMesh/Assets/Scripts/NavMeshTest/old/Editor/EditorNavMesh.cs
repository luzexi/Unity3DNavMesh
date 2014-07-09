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
		private NavMonoEditor m_cNavMono = null;
		public static EditState m_eState = EditState.StateOther;
		public static GameObject m_cParent = null;

		public enum EditState
		{
			StateEditArea,
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
			if (m_eState == EditState.StateEditArea)
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
						point.transform.localScale /= 5;
						point.name = "point";
						area.Add(point);
						this.m_cNavMono.m_iSelPoint = area.m_lstPoints.Count-1;
					}
					e.Use();
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
						FocusEditPanel();
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
							this.m_cNavMono.m_iSelGroup = NavEditAreaManager.sInstance.m_lstAreaGroup.Count-1;
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
							FocusEditPanel();
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
						}
						if( GUILayout.Button("Create Area"))
						{
							Debug.Log("create area");
							group.CreateArea();
							this.m_cNavMono.m_iSelArea = group.m_lstArea.Count-1;
						}
						if( GUILayout.Button("Delete Area"))
						{
							Debug.Log("delete area");
							group.RemoveArea(this.m_cNavMono.m_iSelArea);
							this.m_cNavMono.m_iSelArea = group.m_lstArea.Count-1;
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
								lst.Add( "Point=>" + area.m_lstPoints[i].name);
							}
							this.m_cNavMono.m_iSelPoint = GUILayout.SelectionGrid(
								this.m_cNavMono.m_iSelPoint,
								lst.ToArray(),1);
							FocusEditPanel();
						}
					}
					GUILayout.EndScrollView();
					GUI.enabled = m_eState != EditState.StateEditArea;
					if(GUILayout.Button("Editor Point"))
					{
						m_eState = EditState.StateEditArea;
					}
					GUI.enabled = m_eState == EditState.StateEditArea;
					if(GUILayout.Button("Finish Point"))
					{
						m_eState = EditState.StateFinishArea;
					}
					if(GUILayout.Button("Delete Point"))
					{
						Debug.Log("delete point");
						if(area != null )
						{
							area.RemoveAt(this.m_cNavMono.m_iSelPoint);
						}
					}
					GUI.enabled = true;
				}
				GUILayout.EndVertical();
				
				//===================== ============================
			}
			EditorGUILayout.EndVertical();

			if( GUILayout.Button("Create NavMesh"))
			{
				this.m_cNavMono.CreateNavMesh();
			}
		}
	}

}
