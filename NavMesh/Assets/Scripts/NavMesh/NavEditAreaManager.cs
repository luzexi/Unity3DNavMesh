using UnityEngine;
using System;
using System.Text;
using System.IO;
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
		public const string EDITVERSION = "NAV_AREA_GROUP_001";
		public int m_iLastGroupID = 0;
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
		/// Removes all group.
		/// </summary>
		public void RemoveAllGroup()
		{
			foreach( NavEditAreaGroup item in this.m_lstAreaGroup)
			{
				item.Destroy();
			}
			this.m_lstAreaGroup.Clear();
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
		/// Raies the height.
		/// </summary>
		/// <returns>The height.</returns>
		/// <param name="x">The x coordinate.</param>
		/// <param name="z">The z coordinate.</param>
		public float RayHeight( float x , float z )
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
		/// Saves the area.
		/// </summary>
		/// <param name="path">Path.</param>
		public void SaveAreaGroup( string filePath )
		{
			UTF8Encoding utf8 = new UTF8Encoding();
			
			if (!Directory.Exists(Path.GetDirectoryName(filePath)))
				Directory.CreateDirectory(Path.GetDirectoryName(filePath));
			
			FileStream fs = File.Create(filePath);
			BinaryWriter binWriter = new BinaryWriter(fs);
			
			// write version
			binWriter.Write(utf8.GetBytes(EDITVERSION));
			binWriter.Write(this.m_lstAreaGroup.Count);
			foreach( NavEditAreaGroup item in this.m_lstAreaGroup )
			{
				binWriter.Write(item.m_lstArea.Count);
				foreach (NavEditArea area in item.m_lstArea)
				{
					//save id
					//binWriter.Write(area.areaId);
					//为了和以前的格式保存一直，这里需要存入一个区域等级的标记
					//binWriter.Write(UnWalkFlag);
					//save point count
					binWriter.Write( area.m_lstPoints.Count );
					
					foreach (GameObject point in area.m_lstPoints)
					{
						Vector3 pos = point.transform.position;
						//save x z info
						binWriter.Write(pos.x);
						binWriter.Write(pos.z);
						//为了和以前的格式保存一直，这里需要存入y
						binWriter.Write(pos.y);
					}
				}
			}
			
			binWriter.Close();
			fs.Close();
			
			Debug.Log("保存数据成功!");
		}

		/// <summary>
		/// Loads the area.
		/// </summary>
		/// <param name="path">Path.</param>
		/// <param name="parent">Parent.</param>
		public void LoadAreaGroup( string filePath , GameObject parentPoint )
		{
			if (parentPoint == null)
			{
				Debug.LogError("the parent is null.");
				return;
			}
			float pointHeight = parentPoint.transform.position.y;
			// check file exist
			if (!File.Exists(filePath))
				return;

			RemoveAllGroup();

			// open file
			FileStream fs = File.Open(filePath, FileMode.Open);
			BinaryReader binReader = new BinaryReader(fs);
			
			try
			{
				// read version
				string currVersion = new string(binReader.ReadChars(EDITVERSION.Length));
				if (currVersion == EDITVERSION)
				{
					int areaGroupCount = binReader.ReadInt32();
					for( int k = 0 ; k < areaGroupCount ; k++ )
					{
						NavEditAreaGroup group = new NavEditAreaGroup();
						// read areas count
						int areaCount = binReader.ReadInt32();
						
						for (int i = 0; i < areaCount; i++)
						{
							NavEditArea area = new NavEditArea();
							
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
								float height = RayHeight(x, z);
								point.transform.position = new UnityEngine.Vector3(x, height, z);
								point.transform.parent = parentPoint.transform;
								point.transform.localScale /= 5;

								area.m_lstPoints.Add(point);
							}

							//the frame area.
							if( i == 0 )
								group.m_cFrameArea = area;
							group.m_lstArea.Add(area);
						}
						this.m_lstAreaGroup.Add(group);
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
	}
}
