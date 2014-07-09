using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System;

public enum EditState
{
    StateEditArea,
    StateFinishArea,
    StateOther
}

[CustomEditor(typeof(UnWalkEditor))]
public class EditorUnWalkable : Editor
{
    //////////////////////////////////////////////////////////////////////////
    UnWalkEditor UnWalkEditor;
    // sign the editor state
    public static EditState editState = EditState.StateOther;
    // parent obj, convenient for delete points
    public static GameObject parentPoint = null;

    string unWalkFilePath;
    string serverNavPath;
    //////////////////////////////////////////////////////////////////////////
    // draw variables
    bool lastShowServerNavGrid = false;
    //////////////////////////////////////////////////////////////////////////
    // NavMesh variables
    string navmeshFilePath;
    bool lastSelShowNav = true;

    //////////////////////////////////////////////////////////////////////////
    // The place to put the obj
    public Vector3 placePos;
    //public const int pointHeight = 10;

    // panel variables
    public Vector2 scrollAreaPos;
    public Vector2 scrollPointPos;
    public static bool toggleShowParent = true;

    // 焦点转移到编辑窗口
    private void FocusEditPanel()
    {
        if (SceneView.sceneViews.Count > 0)
        {
            SceneView myView = (SceneView)SceneView.sceneViews[0];
            myView.Focus();
        }
    }

    private void InitAllPanel()
    {
        //all panel
        {
            EditorGUILayout.BeginVertical();
            {
                // toggle showable
                GUILayout.Label("Show/Hide");
                toggleShowParent = GUILayout.Toggle(toggleShowParent, "Show All Points");
                if (parentPoint != null)
                {
                    if (toggleShowParent != parentPoint.active)
                    {
                        parentPoint.SetActiveRecursively(toggleShowParent);
                    }
                }

                // areas
                GUILayout.Label("All Areas");
                scrollAreaPos = EditorGUILayout.BeginScrollView(scrollAreaPos, GUILayout.Height(150));
                {
                    UnWalkEditor.selArea = GUILayout.SelectionGrid(UnWalkEditor.selArea, UnWalkEditor.dataManager.GetAllAreasName(), 1);
                    // active scene view for show unwalkable area
                    if (UnWalkEditor.selArea != UnWalkEditor.lastSelArea)
                    {
                        UnWalkEditor.lastSelArea = UnWalkEditor.selArea;
                        FocusEditPanel();
                    }
                }
                EditorGUILayout.EndScrollView();

                // points
                GUILayout.Label("All Points");
                scrollPointPos = EditorGUILayout.BeginScrollView(scrollPointPos, GUILayout.Height(150));
                {
                    UnWalkEditor.selPoint = GUILayout.SelectionGrid(UnWalkEditor.selPoint, UnWalkEditor.dataManager.GetAllPointsName(UnWalkEditor.selArea), 1);

                    if (UnWalkEditor.selPoint != UnWalkEditor.lastSelPoint)
                    {
                        UnWalkEditor.lastSelPoint = UnWalkEditor.selPoint;
                        FocusEditPanel();

                        // select the obj in scene window
                        //Selection.activeGameObject = UnWalkEditor.dataManager.allAreas[UnWalkEditor.selArea].points[UnWalkEditor.selPoint];
                    }
                }
                EditorGUILayout.EndScrollView();

                // space
                EditorGUILayout.Space();
                GUILayout.Label("Edit Buttons");
                // buttons
                if (GUILayout.Button("CreateNewArea"))
                {
                    CreateArea();
                }

                GUI.enabled = editState == EditState.StateEditArea ? false : true;
                if (GUILayout.Button("EditArea",GUILayout.Height(40)))
                {
                    EditArea();
                }

                GUI.enabled = editState == EditState.StateEditArea ? true : false;
                if (GUILayout.Button("FinishArea", GUILayout.Height(40)))
                {
                    FinishArea();
                }
                GUI.enabled = true;

                EditorGUILayout.BeginHorizontal();
                {
                    if (GUILayout.Button("DeleteArea", GUILayout.Width(80), GUILayout.Height(30)))
                    {
                        DeleteArea();
                    }
                    if (GUILayout.Button("InsertPoint", GUILayout.Width(80), GUILayout.Height(30)))
                    {
                        InsertPoint();
                    }
                    if (GUILayout.Button("DeletePoint", GUILayout.Width(80), GUILayout.Height(30)))
                    {
                        DeletePoint();
                    }
                    if (GUILayout.Button("CheckPoints", GUILayout.Width(80), GUILayout.Height(30)))
                    {
                        UnWalkEditor.dataManager.CheckAllPoints();
                    }
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space();
                GUILayout.Label("Save/Load file.");

                EditorGUILayout.BeginHorizontal();
                {
                    if (GUILayout.Button("LoadUnwalkFile", GUILayout.Width(150), GUILayout.Height(30)))
                    {
                        if (parentPoint != null)
                            UnWalkEditor.dataManager.LoadUnwalkArea(unWalkFilePath, parentPoint);
                        else
                            Debug.LogError("父节点不能为空");

                        UnWalkEditor.selArea = 0;
                        UnWalkEditor.selPoint = 0;
                    }
                    if (GUILayout.Button("SaveUnwalkFile", GUILayout.Width(150), GUILayout.Height(30)))
                    {
                        UnWalkEditor.dataManager.SaveUnwalkArea(unWalkFilePath);
                    }
                }
                EditorGUILayout.EndHorizontal();


                EditorGUILayout.Space();
                UnWalkEditor.showNavMesh = GUILayout.Toggle(UnWalkEditor.showNavMesh, "Show NavMeshs");
                if (lastSelShowNav != UnWalkEditor.showNavMesh)
                {
                    FocusEditPanel();
                    lastSelShowNav = UnWalkEditor.showNavMesh;
                }
                GUILayout.Label("Create/Save/Load NavMesh.");
                EditorGUILayout.BeginHorizontal();
                {
                    if (GUILayout.Button("CreateNavMesh", GUILayout.Width(150), GUILayout.Height(30)))
                    {
                        UnWalkEditor.CreateNavMesh();
                        FocusEditPanel();
                    }
                    if (GUILayout.Button("SaveNavMesh", GUILayout.Width(150), GUILayout.Height(30)))
                    {
                        SaveNavMesh();
                    }
                    if (GUILayout.Button("LoadNavMesh", GUILayout.Width(150), GUILayout.Height(30)))
                    {
                        LoadNavMesh();
                        FocusEditPanel();
                    }
                }
                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndVertical();

            //服务器导出
            GUILayout.Label("Server Export.");
            EditorGUILayout.Space();
            UnWalkEditor.showServerNavGrid = GUILayout.Toggle(UnWalkEditor.showServerNavGrid, "Show Server Nav Grid");
            if (lastShowServerNavGrid != UnWalkEditor.showServerNavGrid)
            {
                FocusEditPanel();
                lastShowServerNavGrid = UnWalkEditor.showServerNavGrid;
            }
            EditorGUILayout.BeginHorizontal();
            {
                if (GUILayout.Button("Export .Nav File", GUILayout.Width(150), GUILayout.Height(30)))
                {
                    ExportNavFile();
                }
            }
            UnWalkEditor.navMeshHeight = (int)parentPoint.transform.position.y;
            EditorGUILayout.EndHorizontal();
        }
    }

    private void InitObjects()
    {
        if (UnWalkEditor == null)
        {
            if (target)
            {
                UnWalkEditor = (UnWalkEditor)target;
            }
            else
                Debug.Log("需要一个脚本对象，必须挂载脚本上面");
        }

        if (parentPoint == null)
        {
            DestroyOldObjs();

            parentPoint = new GameObject();
            parentPoint.name = "MyEdit/ParentPoint";
            parentPoint.transform.position = UnWalkEditor.transform.position;
            parentPoint.transform.parent = UnWalkEditor.transform;
        }
    }

    private void InitVariables()
    {
        string allPath = EditorApplication.currentScene;
        string[] path = allPath.Split(char.Parse("/"));
        string[] fileName = path[path.Length - 1].Split(char.Parse("."));
		allPath = allPath.Replace(path[path.Length - 1], "");
        allPath = allPath + fileName[0];

        unWalkFilePath = allPath +  ".UnWalk";
        navmeshFilePath = allPath +  ".NavMesh.bytes";
        serverNavPath = allPath +  ".nav";
    }

    /// <summary>
    /// 导出服务器使用的导航文件
    /// </summary>
    private void ExportNavFile()
    {
        Vector3 terrainSize = Terrain.activeTerrain.terrainData.size;
        UnWalkEditor.dataManager.ExportNavForServer(serverNavPath,(int)terrainSize.x,(int)terrainSize.z);
    }

    private void SaveNavMesh()
    {
        UnWalkEditor.SaveNavMesh(navmeshFilePath);
    }

    private void LoadNavMesh()
    {
        UnWalkEditor.LoadNavMesh(navmeshFilePath);
    }

    private void DestroyOldObjs()
    {
        GameObject.DestroyImmediate(GameObject.Find("MyEdit/ParentPoint"));
    }

    override public void OnInspectorGUI()
    {
        InitAllPanel();
    }

    public void OnSceneGUI()
    {
        if (Event.current == null)
            return;

        Event e = Event.current;

        if (UnWalkEditor == null || UnWalkEditor.dataManager == null)
            return;

        // only in edit mode will auto add point with mouse click;
        if (editState == EditState.StateEditArea)
        {
            if (e.button == 0 && e.type == EventType.MouseDown)
            {
                if (UnWalkEditor.dataManager.allAreas.Count <= 0)
                    return;

                Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                //int layerMask = 1 << 9;
                RaycastHit hit;
                if (Physics.Raycast(ray.origin, ray.direction, out hit, Mathf.Infinity))
                {
                    placePos = hit.point;
                    //placePos.y = pointHeight;

                    // generate obj
                    GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    point.transform.position = placePos;
                    point.transform.parent = parentPoint.transform;
                    point.transform.localScale /= 5;

                    UnWalkEditor.dataManager.AddPoint(UnWalkEditor.selArea, point);
                }
                e.Use();

                // select the last one
                UnWalkEditor.selPoint = UnWalkEditor.dataManager.allAreas[UnWalkEditor.selArea].points.Count -1;
            }
        }

        // always draw unwalk area
//         for (int i = 0; i < UnWalkEditor.dataManager.allAreas.Count; i++)
//         {
//             DrawUnWalkArea(i);
//         }
    }

    void OnEnable()
    {
        InitVariables();
        InitObjects();

        //重新激活的时候需要检查一遍所有的点是否存在
        UnWalkEditor.dataManager.CheckAllPoints();
    }

    void OnDisable()
    {
        if (editState == EditState.StateEditArea)
        {
            if (UnWalkEditor.gameObject != null)
            {
                Selection.activeGameObject = UnWalkEditor.gameObject;

                //Debug.Log("请先关闭编辑状态(点击FinishArea按钮)，才能选择别的对象");
            }
        }
        //parentPoint.SetActiveRecursively(false);
    }

    void CreateArea()
    {
        UnWalkEditor.dataManager.AddNewArea();
        //select the last area
        UnWalkEditor.selArea = UnWalkEditor.dataManager.allAreas.Count - 1;

        // auto enter edit mode
        EditArea();
    }

    void EditArea()
    {
        if (UnWalkEditor.selArea < 0 || UnWalkEditor.dataManager.allAreas.Count == 0)
        {
            Debug.Log("请先选择一个区域");
            return;
        }

        editState = EditState.StateEditArea;
    }

    private void DeleteArea()
    {
        UnWalkEditor.dataManager.DelArea(UnWalkEditor.selArea);
        UnWalkEditor.selArea = UnWalkEditor.dataManager.allAreas.Count - 1;
    }

    private void FinishArea()
    {
        if (!UnWalkEditor.dataManager.allAreas[UnWalkEditor.selArea].CheckLineCross())
        {
            Debug.Log("当前区域含有不合法的点,请保证线段没有交叉");
            return;
        }
        else
        {
            editState = EditState.StateFinishArea;
        }
    }

    private void InsertPoint()
    {
        if (CheckAllSelect())
        {
            // generate obj
            GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point.transform.position = UnWalkEditor.dataManager.allAreas[UnWalkEditor.selArea].points[UnWalkEditor.selPoint].transform.position;
            point.transform.position += new Vector3(10, 0, 0);
            point.transform.parent = parentPoint.transform;
            point.transform.localScale /= 5;

            UnWalkEditor.dataManager.InsertPoint(UnWalkEditor.selArea, UnWalkEditor.selPoint, point);
        }
    }

    /// <summary>
    /// 检查当前选中索引是否正确
    /// </summary>
    /// <returns></returns>
    private  bool CheckAllSelect()
    {
        if (UnWalkEditor.selArea >= 0 && UnWalkEditor.selArea < UnWalkEditor.dataManager.allAreas.Count &&
            UnWalkEditor.selPoint >= 0 && UnWalkEditor.selPoint < UnWalkEditor.dataManager.allAreas[UnWalkEditor.selArea].points.Count)
            return true;
        return false;
    }

    private void DeletePoint()
    {
        UnWalkEditor.dataManager.DelPoint(UnWalkEditor.selArea, UnWalkEditor.selPoint);
        UnWalkEditor.selPoint = UnWalkEditor.dataManager.allAreas[UnWalkEditor.selArea].points.Count - 1;
    }

    /// <summary>
    /// 绘制不可行走区域
    /// </summary>
    /// <returns></returns>
    private void DrawUnWalkArea(int areaNum)
    {
        if (UnWalkEditor.dataManager == null)
            return;

        if (areaNum < UnWalkEditor.dataManager.allAreas.Count)
        {
            List<Vector3> positions = new List<Vector3>();
            List<GameObject> allPoints = UnWalkEditor.dataManager.allAreas[areaNum].points;
            if (allPoints.Count <= 0)
                return;

            for (int i = 0; i < allPoints.Count; i++)
            {
				if(allPoints[i] != null)
                	positions.Add(allPoints[i].transform.position);
            }
            // 保持首尾相接
            if (allPoints[0] != null)
                positions.Add(allPoints[0].transform.position);

            if (areaNum == UnWalkEditor.selArea)
                Handles.color = Color.red;
            else
                Handles.color = Color.green;
            Handles.DrawPolyLine(positions.ToArray());
        }
    }

}

public class UnwalkTools
{
    [MenuItem("SG Tools/UnwalkEditor")]
    static void Execute()
    {
        string scriptName = "UnWalkEditor";
        GameObject go = GameObject.Find("UnwalkEditor");
        if (go != null)
        {
            if (go.GetComponent(scriptName) != null)
            {
                Selection.activeGameObject = go;
                return;
            }
        }
        else
        {
            go = new GameObject();
            go.name = "UnWalkEditor";
        }

        go.AddComponent(scriptName);
        Selection.activeGameObject = go;
    }
}