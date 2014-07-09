using UnityEngine;
using System.Collections.Generic;
using NavMesh;

public class UnWalkEditor : MonoBehaviour
{
    public UnWalkDataManager dataManager = new UnWalkDataManager();
    List<Triangle> allNavMeshData = new List<Triangle>();
    public int navMeshHeight = 10;
    public bool showNavMesh = true;
    public int selArea = 0;
    public int lastSelArea = -1;
    public int selPoint = 0;
    public int lastSelPoint = -1;

    //绘制服务器的导航格子
    public bool showServerNavGrid = false;

    // Use this for initialization
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
    }

    void OnDrawGizmos()
    {
        DrawAllAreas();
        DrawSelectPoint();
        //绘制导航网格
        DrawNavMesh();

        DrawServerGridInfo();
    }

    private void DrawServerGridInfo()
    {
        if (showServerNavGrid)
        {
            if (dataManager.mWorld != null && dataManager.mWorld.Length != 0)
            {
                float y = 0;
                float x = 0;
                int max = 1000;
                int maxDelta = 0;
                int width = (int)Terrain.activeTerrain.terrainData.size.x * 2;
                int height = (int)Terrain.activeTerrain.terrainData.size.y;
                for (int i = 0; i < dataManager.mWorld.Length; i++)
                {
                    if (dataManager.mWorld[i] != 0)
                    {
                        x = i % width;
                        y = i / width;
                        //if (maxDelta++ > max)
                            //break;
                        Gizmos.color = Color.red;
                        Vector3 p1 = new Vector3(x / 2, navMeshHeight, y / 2);
                        Vector3 p2 = new Vector3(x / 2 + 0.5f, navMeshHeight, y / 2); ;
                        Vector3 p3 = new Vector3(x / 2 + 0.5f, navMeshHeight, y / 2 + 0.5f);
                        Vector3 p4 = new Vector3(x / 2, navMeshHeight, y / 2 + 0.5f);
                        Gizmos.DrawLine(p1, p2);
                        Gizmos.DrawLine(p2, p3);
                        Gizmos.DrawLine(p3, p4);
                        Gizmos.DrawLine(p4, p1);
                        //Gizmos.DrawCube(new Vector3(x/2, 0, y/2), new Vector3(1, 1, 1));
                    }
                }
            }
        }
    }

    private void DrawNavMesh()
    {
        if (showNavMesh)
        {
            if (allNavMeshData.Count != 0)
            {
                foreach (Triangle tri in allNavMeshData)
                {
                    if (tri.Group == 0)
                    {
                        Gizmos.color = Color.blue;
                        //continue;
                    }
                    else if (tri.Group == 1)
                    {
                        Gizmos.color = Color.grey;
                        //continue;
                    }
                    else if (tri.Group == 2)
                        Gizmos.color = Color.black;

                    Vector3 p1 = new Vector3(tri.Points[0].x, navMeshHeight, tri.Points[0].y);
                    Vector3 p2 = new Vector3(tri.Points[1].x, navMeshHeight, tri.Points[1].y);
                    Vector3 p3 = new Vector3(tri.Points[2].x, navMeshHeight, tri.Points[2].y);
                    Gizmos.DrawLine(p1, p2);
                    Gizmos.DrawLine(p2, p3);
                    Gizmos.DrawLine(p3, p1);
                }
            }
        }
    }

    private void DrawSelectPoint()
    {
        if (dataManager.allAreas.Count <= 0 || selArea >= dataManager.allAreas.Count ||
            dataManager.allAreas[selArea].points.Count <= 0 || selPoint >= dataManager.allAreas[selArea].points.Count)
            return;

        Gizmos.DrawIcon(dataManager.allAreas[selArea].points[selPoint].transform.position, "point.tif");
    }

    /// <summary>
    /// 绘制所有区域
    /// </summary>
    /// <returns></returns>
    private void DrawAllAreas()
    {
        // always draw unwalk area
        for (int i = 0; i < dataManager.allAreas.Count; i++)
        {
            DrawUnWalkArea(i);
        }
    }

    /// <summary>
    /// 绘制不可行走区域
    /// </summary>
    /// <returns></returns>
    private void DrawUnWalkArea(int areaNum)
    {
        if (dataManager == null)
            return;

        if (areaNum < dataManager.allAreas.Count)
        {
            List<GameObject> allPoints = dataManager.allAreas[areaNum].points;
            if (allPoints.Count <= 0)
                return;

            if (areaNum == selArea)
                Gizmos.color = Color.red;
            else
                Gizmos.color = Color.green;

            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i] == null)
                {
                    dataManager.CheckAllPoints();
                    return;
                }
                else
                {
                    if (i != allPoints.Count - 1)
                    {
                        if (allPoints[i + 1] == null)
                        {
                            dataManager.CheckAllPoints();
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
    }

    /// <summary>
    /// 获得不可行走区域
    /// </summary>
    /// <returns></returns>
    private List<Polygon> GetUnWalkAreas()
    {
        List<Polygon> areas = new List<Polygon>();
        for (int i = 0; i < dataManager.allAreas.Count; i++)
        {
            List<GameObject> allPoints = dataManager.allAreas[i].points;
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
        Debug.Log("开始创建导航网格...");
        List<Polygon> areas = GetUnWalkAreas();
        NavResCode genResult = NavMeshGen.Instance.CreateNavMesh(areas, ref allNavMeshData);
        Debug.Log(allNavMeshData.Count);
        foreach (Triangle item in allNavMeshData)
            Debug.Log(item.Points[0] + " -- " + item.Points[1] + " -- " + item.Points[2]);

        if (genResult != NavResCode.Success)
            Debug.LogError("创建导航网格失败");
        else
            Debug.Log("创建导航网格成功!");
    }

    /// <summary>
    /// 保存导航网格
    /// </summary>
    /// <param name="filePath"></param>
    public void SaveNavMesh(string filePath)
    {
        if (allNavMeshData.Count == 0)
        {
            Debug.LogError("必须先创建导航网格");
        }
        else if (filePath.Length == 0)
        {
            Debug.LogError("保存路径不能为空");
        }
        else
        {
            NavResCode saveResult = NavMeshGen.Instance.SaveNavMeshToFile(filePath, allNavMeshData);
            if (saveResult != NavResCode.Success)
            {
                Debug.LogError("保存导航网格失败");
            }
            else
            {
                Debug.Log("保存导航网格成功!");
            }
        }
    }

    /// <summary>
    /// 加载导航网格
    /// </summary>
    /// <param name="filePath"></param>
    public void LoadNavMesh(string filePath)
    {
        if (filePath.Length == 0)
        {
            Debug.LogError("加载路径不能为空");
        }
        else
        {
            NavResCode loadResult =
                NavMeshGen.Instance.LoadNavMeshFromFile(filePath, out allNavMeshData);
            if (loadResult != NavResCode.Success)
            {
                Debug.LogError("加载导航网格失败");
            }
            else
            {
                Debug.Log("加载导航网格成功!");
            }
        }
    }
}
