using UnityEngine;
using System.IO;
using System.Text;
using System.Collections.Generic;
using NavMesh;
using System;

// namespace SGUnWalk
// {
/// <summary>
/// 不可行走区域
/// </summary>
public class UnWalkData
{
    private int lastNum = 0;
    public int LastPointNum
    {
        get { return lastNum; }
        set { lastNum = value; }
    }
    private string areaName;
    public string AreaName
    {
        get { return areaName; }
        set { areaName = value; }
    }
    public int areaId = -1;
    // 区域的所有顶点
    public List<GameObject> points = new List<GameObject>();
    // 顶点的名字
    public List<string> pointNames = new List<string>();

    /// <summary>
    /// add new point
    /// </summary>
    /// <param name="point"></param>
    /// <returns></returns>
    public void AddPoint(GameObject point)
    {
        if (point)
        {
            AddPoint(point, point.name);
        }
    }

    /// <summary>
    /// add new point with name
    /// </summary>
    /// <param name="point"></param>
    /// <param name="pointName"></param>
    /// <returns></returns>
    public void AddPoint(GameObject point, string pointName)
    {
        if (point)
        {
            points.Add(point);
            pointNames.Add(pointName);
            LastPointNum++;
        }
    }

    public void InsertPoint(int preIndex, GameObject point)
    {
        if (point != null && preIndex >= 0 && preIndex < points.Count)
        {
            points.Insert(preIndex, point);
            pointNames.Insert(preIndex, point.name);
            LastPointNum++;
        }
    }

    public bool CheckLineCross()
    {
        Vector2 p1s, p1e, p2s, p2e;

        int iPointCount = points.Count;

        // 如果点的列表中只有两个点, 可以添加新点.
        if (iPointCount <= 2)
        {
            return true;
        }

        for (int i = 0; i < iPointCount - 2; i++)
        {
            p1s.x = points[i].transform.position.x;
            p1s.y = points[i].transform.position.z;
            p1e.x = points[i + 1].transform.position.x;
            p1e.y = points[i + 1].transform.position.z;

            for (int j = i + 2; j < iPointCount; j++)
            {
                if (j != iPointCount - 1)
                {
                    p2s.x = points[j].transform.position.x;
                    p2s.y = points[j].transform.position.z;
                    p2e.x = points[j + 1].transform.position.x;
                    p2e.y = points[j + 1].transform.position.z;
                }
                else
                {
                    p2s.x = points[j].transform.position.x;
                    p2s.y = points[j].transform.position.z;
                    p2e.x = points[0].transform.position.x;
                    p2e.y = points[0].transform.position.z;

                    if (0 == i)
                    {
                        continue;
                    }
                }

                if (SGMath.CheckCross(p1s, p1e, p2s, p2e))
                {
                    return false;
                }

            }
        }
        return true;
    }

    /// <summary>
    /// delete current point
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    public bool DelPoint(int index)
    {
        if (points.Count == 0)
            return false;
        if (index < points.Count)
        {
            if (points[index] != null)
                GameObject.DestroyImmediate(points[index]);

            points.RemoveAt(index);
            pointNames.RemoveAt(index);
            return true;
        }
        return false;
    }


    /// <summary>
    /// 获得所有顶点的名字，用于显示
    /// </summary>
    /// <returns></returns>
    public string[] GetAllPointNames()
    {
        return pointNames.ToArray();
    }
}

/// <summary>
/// 不可行走区域管理类
/// </summary>
public class UnWalkDataManager
{
    static int UnWalkFlag = 11;
    //服务器使用的世界格子
    public int[] mWorld;

    // file info
    private const string EditVersion = "REGION_EDIT_01";// "EditVersion_01";

    // data variables
    public List<UnWalkData> allAreas = new List<UnWalkData>();
    public List<string> areasName = new List<string>();
    // save last area num
    public static int lastAreaNum = 0;

    public string[] GetAllAreasName()
    {
        return areasName.ToArray();
    }

    public string[] GetAllPointsName(int areaIndex)
    {
        string[] emptyName = { };
        if (allAreas == null)
            return emptyName;
        if (areaIndex >= allAreas.Count || allAreas.Count <= 0)
        {
            //LogManager.Log("区域索引[" + areaIndex + "]不对");
            return emptyName;
        }
        UnWalkData walkData = allAreas[areaIndex];
        return walkData.GetAllPointNames();
    }


    /// <summary>
    /// 添加新不可行走区域
    /// </summary>
    /// <returns></returns>
    public void AddNewArea()
    {
        AddNewArea(++lastAreaNum);
    }

    public void AddNewArea(int areaId)
    {
        UnWalkData walkData = new UnWalkData();
        walkData.areaId = areaId;
        walkData.AreaName = "Area" + walkData.areaId;
        allAreas.Add(walkData);
        areasName.Add(walkData.AreaName);
    }

    /// <summary>
    /// 删除不可行走区域
    /// </summary>
    /// <param name="index">区域索引</param>
    /// <returns></returns>
    public void DelArea(int index)
    {
        if (allAreas.Count <= 0)
            return;
        if (index <= allAreas.Count)
        {
            UnWalkData walkData = allAreas[index];

            // 需要删除游戏里面的所有顶点对象
            foreach (GameObject obj in walkData.points)
            {
                GameObject.DestroyImmediate(obj);
            }

            allAreas.RemoveAt(index);
            areasName.RemoveAt(index);
        }
    }

    /// <summary>
    /// 添加一个新的不可行走区域顶点
    /// </summary>
    /// <param name="areaIndex">区域索引</param>
    /// <param name="obj">顶点对象</param>
    /// <returns></returns>
    public void AddPoint(int areaIndex, GameObject obj)
    {
        if (areaIndex < allAreas.Count)
        {
            UnWalkData walkData = allAreas[areaIndex];
            string pointName = "区域" + (areaIndex + 1) + "->点" + (walkData.LastPointNum);
            obj.name = pointName;
            walkData.AddPoint(obj, pointName);
        }
        else
        {
            Debug.Log("区域索引[" + areaIndex + "]不对，请选择区域先");
        }
    }

    public void InsertPoint(int areaIndex, int prePointIndex, GameObject obj)
    {
        if (areaIndex < allAreas.Count)
        {
            UnWalkData walkData = allAreas[areaIndex];
            string pointName = "区域" + (areaIndex + 1) + "->点" + (walkData.LastPointNum);
            obj.name = pointName;
            walkData.InsertPoint(prePointIndex, obj);
        }
        else
            Debug.Log("区域索引[" + areaIndex + "]不对，请选择区域先");
    }

    /// <summary>
    /// 根据索引删除顶点
    /// </summary>
    /// <param name="areaIndex">不可行走区域索引</param>
    /// <param name="pointIndex">顶点索引</param>
    /// <returns></returns>
    public void DelPoint(int areaIndex, int pointIndex)
    {
        if (areaIndex < allAreas.Count)
        {
            UnWalkData walkData = allAreas[areaIndex];
            if (walkData.points.Count <= 0)
            {
                Debug.Log("顶点已经全部删除");
            }
            else if (pointIndex < walkData.points.Count)
            {
                walkData.DelPoint(pointIndex);
            }
            else
                Debug.LogError("顶点索引[" + pointIndex + "]不对，请选择正确的顶点");
        }
        else
            Debug.LogError("区域索引[" + areaIndex + "]不对，请选择区域先");
    }

    public void DelAllAreas()
    {
        int allNum = allAreas.Count;
        for (int i = 0; i < allNum; i++)
        {
            DelArea(0);
        }
    }

    public void CheckAllPoints()
    {
        foreach (UnWalkData area in allAreas)
        {
            int pointNum = area.points.Count;
            for (int i = 0; i < pointNum; i++)
            {
                if (area.points[i] == null)
                {
                    area.DelPoint(i);

                    i--;
                    pointNum--;
                }
            }
        }
    }

    /// <summary>
    /// save whole unwalkable area to file
    /// </summary>
    /// <param name="filePath">unwalkable file path</param>
    /// <returns></returns>
    public void SaveUnwalkArea(string filePath)
    {
        UTF8Encoding utf8 = new UTF8Encoding();

        if (!Directory.Exists(Path.GetDirectoryName(filePath)))
            Directory.CreateDirectory(Path.GetDirectoryName(filePath));

        FileStream fs = File.Create(filePath);
        BinaryWriter binWriter = new BinaryWriter(fs);

        // write version
        binWriter.Write(utf8.GetBytes(EditVersion));
        // write count
        binWriter.Write(allAreas.Count);

        foreach (UnWalkData walkData in allAreas)
        {
            //save id
            binWriter.Write(walkData.areaId);
            //为了和以前的格式保存一直，这里需要存入一个区域等级的标记
            binWriter.Write(UnWalkFlag);
            //save point count
            binWriter.Write(walkData.points.Count);

            foreach (GameObject point in walkData.points)
            {
                Vector3 pos = point.transform.position;
                //save x z info
                binWriter.Write(pos.x);
                binWriter.Write(pos.z);
                //为了和以前的格式保存一直，这里需要存入y
                binWriter.Write(pos.y);
            }
        }

        binWriter.Close();
        fs.Close();

        Debug.Log("保存数据成功!");
    }

    //获取场景的高度（包括地形和建筑）
    float GetSceneHeight(float x, float z)
    {
        float SceneHeight = -100000.0f;
        Ray ray = new Ray();//构造射线
        ray.direction = -Vector3.up;
        ray.origin = new Vector3(x, 100000.0f, z);
        RaycastHit hitInfo;
        if (Physics.Raycast(ray, out hitInfo, Mathf.Infinity))//排除actor
        {
            SceneHeight = hitInfo.point.y;
        }
        return SceneHeight;
    }

    /// <summary>
    /// load unwalkable data
    /// </summary>
    /// <param name="filePath"></param>
    /// <returns></returns>
    public void LoadUnwalkArea(string filePath, GameObject parentPoint)
    {
        if (parentPoint == null)
        {
            Debug.LogError("父节点不能为空");
            return;
        }
        float pointHeight = parentPoint.transform.position.y;
        // check file exist
        if (!File.Exists(filePath))
            return;

        DelAllAreas();

        // open file
        FileStream fs = File.Open(filePath, FileMode.Open);
        BinaryReader binReader = new BinaryReader(fs);

        try
        {
            // read version
            string currVersion = new string(binReader.ReadChars(EditVersion.Length));
            if (currVersion == EditVersion)
            {
                // read areas count
                int areaCount = binReader.ReadInt32();

                for (int i = 0; i < areaCount; i++)
                {
                    // read id
                    int areaId = binReader.ReadInt32();
                    AddNewArea(areaId);

                    //为了和以前的格式保存一直，这里需要读取一个区域等级的标记
                    int areaFlag = binReader.ReadInt32();

                    //read point count
                    int pointCount = binReader.ReadInt32();

                    for (int j = 0; j < pointCount; j++)
                    {
                        // read pos
                        float x = binReader.ReadSingle();
                        //float z = binReader.ReadSingle();
                        float z = binReader.ReadSingle();
                        //为了和以前的格式保存一直，这里需要y
                        float y = binReader.ReadSingle();

                        // auto generate point gameobject
                        GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        float height = GetSceneHeight(x, z);
                        point.transform.position = new UnityEngine.Vector3(x, height, z);
                        point.transform.parent = parentPoint.transform;
                        point.transform.localScale /= 5;

                        AddPoint(allAreas.Count - 1, point);
                    }
                }
            }
            else
            {
                Debug.LogError("version not match");
            }
        }
        catch (EndOfStreamException e)
        {
            Debug.LogError(e.Message);
        }
        finally
        {
            binReader.Close();
            fs.Close();
        }

        Debug.Log("加载数据成功!");
    }

    const int ServerNavScale = 2;
    public void ExportNavForServer(string filePath, int length, int width)
    {
        length *= ServerNavScale;
        width *= ServerNavScale;
        mWorld = new int[length * width];

        List<Polygon> polys = new List<Polygon>();
        foreach (UnWalkData walkData in allAreas)
        {
            // 以后要支持行走等级
            //int Level = pRegion->GetFlyable();

            Polygon poly = new Polygon();
            for (int i = 0; i < walkData.points.Count; i++)
            {
                GameObject point = walkData.points[i];

                Vector2 pos = new Vector2();
                pos.x = point.transform.position.x;
                pos.y = point.transform.position.z;
                pos *= ServerNavScale;

                poly.allPoints.Add(pos);
            }
            polys.Add(poly);
            CalculateNavInfo(poly, UnWalkFlag, width);
        }

        //由于客户端使用导航网格寻路，会贴边走，所以服务器删除拐点处的阻挡信息
        foreach (Polygon poly in polys)
        {
            for (int i = 0; i < poly.allPoints.Count; i++)
            {
                Vector2 ptStart = poly.allPoints[i];
                int x = (int)Math.Floor(ptStart.x / 2) * 2;
                int y = (int)Math.Floor(ptStart.y / 2) * 2;
                mWorld[y * width + x] = 0;
                if ((y + 1) * width + x + 1 >= 0)
                    mWorld[(y + 1) * width + x + 1] = 0;
				if(y * width + x -1 >= 0)
                	mWorld[y * width + x -1] = 0;
				if((y-1) * width + x >= 0)
                	mWorld[(y-1) * width + x] = 0;
            }
        }

        SaveToNavMapFile(filePath,width,length);
    }

    private void SaveToNavMapFile(string filePath,int width,int length)
    {
        UTF8Encoding utf8 = new UTF8Encoding();

        if (!Directory.Exists(Path.GetDirectoryName(filePath)))
            Directory.CreateDirectory(Path.GetDirectoryName(filePath));

        FileStream fs = File.Create(filePath);
        BinaryWriter binWriter = new BinaryWriter(fs);

        // write width and length
        binWriter.Write((ushort)width);
        binWriter.Write((ushort)length);

        byte[] temp = new byte[mWorld.Length * sizeof(int)];
        Buffer.BlockCopy(mWorld, 0, temp, 0, temp.Length);

        binWriter.Write(temp);


        binWriter.Close();
        fs.Close();

        Debug.Log("保存" + filePath + "成功!");
    }

    private void CalculateNavInfo(Polygon poly, int UnWalkFlag, int width)
    {
        Rect rect = poly.GetCoverRect();

        for (int y = (int)rect.yMin; y < (int)rect.yMax; y++)
        {
            for (int x = (int)rect.xMin; x < (int)rect.xMax; x++)
            {
                //if (poly.IsPointIn(new Vector2(x, y)) && poly.IsPointIn(new Vector2(x + 1, y + 1)) 
				//	&& poly.IsPointIn(new Vector2(x - 1, y - 1)) && poly.IsPointIn(new Vector2(x, y + 1)) && poly.IsPointIn(new Vector2(x+ 1, y )))
                {
                    mWorld[y * width + x] = UnWalkFlag;
                }
            }
        }
    }

}
/*}*/