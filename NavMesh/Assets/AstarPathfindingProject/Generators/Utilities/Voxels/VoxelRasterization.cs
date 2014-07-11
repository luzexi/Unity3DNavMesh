//#define ASTARDEBUG
//#define ASTAR_DEBUGREPLAY
#define ASTAR_RECAST_ARRAY_BASED_LINKED_LIST //Faster Recast scan and nicer to GC, but might be worse at handling very large tile sizes. Makes Recast use linked lists based on structs in arrays instead of classes and references.

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.Threading;
using Pathfinding.Voxels;

namespace Pathfinding.Voxels {
	/** Voxelizer for recast graphs.
	 * 
	 * In comments: units wu are World Units, vx are Voxels
	 * 
	 * \astarpro
	 */
	public partial class Voxelize {
		
		public List<ExtraMesh> inputExtraMeshes;
		
		protected Vector3[] inputVertices;
		protected int[] inputTriangles;
		
		/* Minimum floor to 'ceiling' height that will still allow the floor area to 
		  * be considered walkable. [Limit: > 0] [Units: wu] */
		//public float walkableHeight = 0.8F;
		
		/* Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: wu] */
		//public float walkableClimb = 0.8F;
		
		/** Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]  */
		public readonly int voxelWalkableClimb;
		
		/** Minimum floor to 'ceiling' height that will still allow the floor area to 
		  * be considered walkable. [Limit: >= 3] [Units: vx] */
		public readonly uint voxelWalkableHeight;
		
		/** The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] */
		public readonly float cellSize = 0.2F;
		
		/** The y-axis cell size to use for fields. [Limit: > 0] [Units: wu] */
		public readonly float cellHeight = 0.1F;
		
		public int minRegionSize = 100;
		
		/** The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx] */
		public int borderSize = 0;
		
		/** The maximum allowed length for contour edges along the border of the mesh. [Limit: >= 0] [Units: vx]  */
		public float maxEdgeLength = 20;
		
		/** The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] */
		public float maxSlope = 30;
		
		public RecastGraph.RelevantGraphSurfaceMode relevantGraphSurfaceMode;
		
		/** The world AABB to rasterize */
		public Bounds forcedBounds;
	
		public VoxelArea voxelArea;
		public VoxelContourSet countourSet;
		
		
		/** Width in voxels.
		 * Must match the #forcedBounds
		 */
		public int width;
		
		/** Depth in voxels.
		 * Must match the #forcedBounds
		 */
		public int depth;
		
#region Debug

		public Vector3 voxelOffset;
		
		public Vector3 CompactSpanToVector(int x, int z, int i) {
			return voxelOffset+new Vector3(x*cellSize,voxelArea.compactSpans[i].y*cellHeight,z*cellSize);
		}
		
		public void VectorToIndex(Vector3 p, out int x, out int z) {
			p -= voxelOffset;
			x = Mathf.RoundToInt (p.x / cellSize);
			z = Mathf.RoundToInt (p.z / cellSize);
		}
		
#endregion	
	
	#region Constants /** @name Constants @{ */
		
		public const uint NotConnected = 0x3f;
		
		/** Unmotivated variable, but let's clamp the layers at 65535 */
		public const int MaxLayers = 65535;
		
		/** \todo : Check up on this variable */
		public const int MaxRegions = 500;
		
		public const int UnwalkableArea = 0;
		
		/** If heightfield region ID has the following bit set, the region is on border area
		 * and excluded from many calculations. */
		public const ushort BorderReg = 0x8000;
		
		/** If contour region ID has the following bit set, the vertex will be later
		 * removed in order to match the segments and vertices at tile boundaries. */
		public const int RC_BORDER_VERTEX = 0x10000;
	
		public const int RC_AREA_BORDER = 0x20000;
	
		public const int VERTEX_BUCKET_COUNT = 1<<12;
		
		public const int RC_CONTOUR_TESS_WALL_EDGES = 0x01;	// Tessellate wall edges
		public const int RC_CONTOUR_TESS_AREA_EDGES = 0x02;	// Tessellate edges between areas.
		
		/** Mask used with contours to extract region id. */
		public const int ContourRegMask = 0xffff;
		
#endregion /** @} */
		
		public string debugString = "";
		
		
		
	#if !PhotonImplementation
		public void OnGUI () {
			GUI.Label (new Rect (5,5,200,Screen.height),debugString);
		}
	#endif
		
		public readonly Vector3 cellScale;
		public readonly Vector3 cellScaleDivision;
		
		public Voxelize (float ch, float cs, float wc, float wh, float ms) {
			cellSize = cs;
			cellHeight = ch;
			float walkableHeight = wh;
			float walkableClimb = wc;
			maxSlope = ms;
			
			cellScale = new Vector3 (cellSize,cellHeight,cellSize);
			cellScaleDivision = new Vector3 (1F/cellSize,1F/cellHeight,1F/cellSize);
			
			voxelWalkableHeight = (uint)(walkableHeight/cellHeight);
			voxelWalkableClimb = Mathf.RoundToInt (walkableClimb/cellHeight);
		}
		
		public void CollectMeshes () {
			CollectMeshes (inputExtraMeshes, forcedBounds,out inputVertices, out inputTriangles);
		}
		
		public static void CollectMeshes (List<ExtraMesh> extraMeshes, Bounds bounds, out Vector3[] verts, out int[] tris) {
			verts = null;
			tris = null;
			
		}
		
		public void Init () {
			
			//Initialize the voxel area
			if (voxelArea == null || voxelArea.width != width || voxelArea.depth != depth)
				voxelArea = new VoxelArea (width, depth);
			else voxelArea.Reset ();
		}
		
		public void VoxelizeInput () {
			
			AstarProfiler.StartProfile ("Build Navigation Mesh");
			
			AstarProfiler.StartProfile ("Voxelizing - Step 1");
			
			
			//Debug.DrawLine (forcedBounds.min,forcedBounds.max,Color.blue);
			
			//Vector3 center = bounds.center;
			Vector3 min = forcedBounds.min;
			voxelOffset = min;
			
			float ics = 1F/cellSize;
			float ich = 1F/cellHeight;
			
			AstarProfiler.EndProfile ("Voxelizing - Step 1");
			
			AstarProfiler.StartProfile ("Voxelizing - Step 2 - Init");
			
			float slopeLimit = Mathf.Cos (Mathf.Atan(Mathf.Tan(maxSlope*Mathf.Deg2Rad)*(ich*cellSize)));
			
			//Utility.StartTimerAdditive (true);
			
			float[] vTris = new float[3*3];
			float[] vOut = new float[7*3];
			float[] vRow = new float[7*3];
			float[] vCellOut = new float[7*3];
			float[] vCell = new float[7*3];
			
			if (inputExtraMeshes == null) throw new System.NullReferenceException ("inputExtraMeshes not set");
			
			//Find the largest lenghts of vertex arrays and check for meshes which can be skipped
			int maxVerts = 0;
			for (int m=0;m<inputExtraMeshes.Count;m++) {
				if (!inputExtraMeshes[m].bounds.Intersects (forcedBounds)) continue;
				maxVerts = System.Math.Max(inputExtraMeshes[m].vertices.Length, maxVerts);
			}
			
			//Create buffer, here vertices will be stored multiplied with the local-to-voxel-space matrix
			Vector3[] verts = new Vector3[maxVerts];
			
			Matrix4x4 voxelMatrix = Matrix4x4.Scale (new Vector3(ics,ich,ics)) * Matrix4x4.TRS(-min,Quaternion.identity, Vector3.one);
			
			AstarProfiler.EndProfile ("Voxelizing - Step 2 - Init");
			AstarProfiler.StartProfile ("Voxelizing - Step 2");
			
			for (int m=0;m<inputExtraMeshes.Count;m++) {
				ExtraMesh mesh = inputExtraMeshes[m];
				
				if (!mesh.bounds.Intersects (forcedBounds)) continue;
				
				Matrix4x4 matrix = mesh.matrix;
				matrix = voxelMatrix * matrix;
					
				Vector3[] vs = mesh.vertices;
				int[] tris = mesh.triangles;
				int trisLength = tris.Length;
				
				for (int i=0;i<vs.Length;i++) verts[i] = matrix.MultiplyPoint3x4(vs[i]);
				
				//AstarProfiler.StartFastProfile(0);
				
				int mesharea = mesh.area;
				
				for (int i=0;i<trisLength;i += 3) {
					Vector3 p1;
					Vector3 p2;
					Vector3 p3;
					
					int minX;
					int minZ;
					int maxX;
					int maxZ;
					
					p1 = verts[tris[i]];
					p2 = verts[tris[i+1]];
					p3 = verts[tris[i+2]];
					
					minX = (int)(Utility.Min (p1.x,p2.x,p3.x));
					// (Mathf.Min (Mathf.Min (p1.x,p2.x),p3.x));
					minZ = (int)(Utility.Min (p1.z,p2.z,p3.z));
					
					maxX = (int)System.Math.Ceiling (Utility.Max (p1.x,p2.x,p3.x));
					maxZ = (int)System.Math.Ceiling (Utility.Max (p1.z,p2.z,p3.z));
					
					minX = Mathf.Clamp (minX , 0 , voxelArea.width-1);
					maxX = Mathf.Clamp (maxX , 0 , voxelArea.width-1);
					minZ = Mathf.Clamp (minZ , 0 , voxelArea.depth-1);
					maxZ = Mathf.Clamp (maxZ , 0 , voxelArea.depth-1);
					
					
					
					if (minX >= voxelArea.width || minZ >= voxelArea.depth || maxX <= 0 || maxZ <= 0) continue;
					
					//Debug.DrawLine (p1*cellSize+min+Vector3.up*0.2F,p2*cellSize+voxelOffset+Vector3.up*0.1F,Color.red);
					//Debug.DrawLine (p2*cellSize+min+Vector3.up*0.1F,p3*cellSize+voxelOffset,Color.red);
					
					Vector3 normal;
					
					int area;
					
					//AstarProfiler.StartProfile ("Rasterize...");
					
					normal = Vector3.Cross (p2-p1,p3-p1);
					
					float dot = Vector3.Dot (normal.normalized,Vector3.up);
					
					if (dot < slopeLimit) {
						area = UnwalkableArea;
					} else {
						area = 1 + mesharea;
					}
					
					
					
					
					//Debug.DrawRay (((p1+p2+p3)/3.0F)*cellSize+voxelOffset,normal,Color.cyan);
					
					Utility.CopyVector (vTris,0,p1);
					Utility.CopyVector (vTris,3,p2);
					Utility.CopyVector (vTris,6,p3);
					
					//Utility.StartTimerAdditive (false);
					
					for (int x=minX;x<=maxX;x++) {
						
						int nrow = Utility.ClipPolygon (vTris , 3 , vOut , 1F , -x+0.5F,0);
						
						if (nrow < 3) {
							continue;
						}
						
						
						nrow = Utility.ClipPolygon (vOut,nrow,vRow,-1F,x+0.5F,0);
						
						if (nrow < 3) {
							continue;
						}
						
						float clampZ1 = vRow[2];
						float clampZ2 = vRow[2];
						for (int q=1; q < nrow;q++) {
							float val = vRow[q*3+2];
							clampZ1 = System.Math.Min (clampZ1,val);
							clampZ2 = System.Math.Max (clampZ2,val);
						}
						
						int clampZ1I = AstarMath.Clamp ((int)System.Math.Round (clampZ1),0, voxelArea.depth-1);
						int clampZ2I = AstarMath.Clamp ((int)System.Math.Round (clampZ2),0, voxelArea.depth-1);
						
						
						for (int z=clampZ1I;z<=clampZ2I;z++) {
							
							//AstarProfiler.StartFastProfile(1);
							int ncell = Utility.ClipPolygon (vRow , nrow , vCellOut , 1F , -z+0.5F,2);
							
							if (ncell < 3) {
								//AstarProfiler.EndFastProfile(1);
								continue;
							}
							
							ncell = Utility.ClipPolygonY (vCellOut , ncell , vCell , -1F , z+0.5F,2);
							
							if (ncell < 3) {
								//AstarProfiler.EndFastProfile(1);
								continue;
							}
							
							//AstarProfiler.EndFastProfile(1);
							/*
							for (int q=0, j = ncell-1; q < ncell; j=q, q++) {
								Debug.DrawLine (vCell[q]*cellSize+min,vCell[j]*cellSize+min,Color.cyan);
							}*/
							
							//AstarProfiler.StartFastProfile(2);
							float sMin = vCell[1];
							float sMax = vCell[1];
							for (int q=1; q < ncell;q++) {
								float val = vCell[q*3+1];
								sMin = System.Math.Min (sMin,val);
								sMax = System.Math.Max (sMax,val);
							}
							
							//AstarProfiler.EndFastProfile(2);
							
							//Debug.DrawLine (new Vector3(x,sMin,z)*cellSize+min,new Vector3(x,sMax,z)*cellSize+min,Color.cyan);
							//if (z < 0 || x < 0 || z >= voxelArea.depth || x >= voxelArea.width) {
								//Debug.DrawRay (new Vector3(x,sMin,z)*cellSize+min, Vector3.up, Color.red);
								//continue;
							//}
							
							int maxi = (int)System.Math.Ceiling(sMax);
							if (maxi >= 0) {
								int mini = (int)(sMin+1);
								voxelArea.AddLinkedSpan (z*voxelArea.width+x, (mini >= 0 ? (uint)mini : 0),(uint)maxi,area, voxelWalkableClimb);
								//voxelArea.cells[z*voxelArea.width+x].AddSpan ((mini >= 0 ? (uint)mini : 0),(uint)maxi,area, voxelWalkableClimb);
							}
							
						}
						
						
					}
					
					
				}
				//AstarProfiler.EndFastProfile(0);
				//AstarProfiler.EndProfile ("Rasterize...");
				//Utility.EndTimerAdditive ("",false);
			}
			AstarProfiler.EndProfile ("Voxelizing - Step 2");
			
			//Debug.Log ("Span Count " + voxelArea.maxStackSize + " " + voxelArea.GetSpanCountAll() + " " + voxelArea.GetSpanCountAll2 () + " " + voxelArea.linkedSpanCount + " " + (voxelArea.linkedSpanCount - voxelArea.removedStackCount) + " " + (voxelArea.width*voxelArea.depth));
			
			/*int wd = voxelArea.width*voxelArea.depth;
			for (int x=0;x<wd;x++) {
				VoxelCell c = voxelArea.cells[x];
				float cPos = (float)x/(float)voxelArea.width;
				float  cPos2 = Mathf.Floor (cPos);
				
				Vector3 p = new Vector3((cPos-cPos2)*voxelArea.width,0,cPos2);
				p *= cellSize;
				p += min;
				VoxelSpan span = c.firstSpan;
				
				int count =0;
				while (span != null) {
					Color col = count < Utility.colors.Length ? Utility.colors[count] : Color.white;
					//p.y = span.bottom*cellSize+min.y;
					Debug.DrawLine (p+new Vector3(0,span.bottom*cellSize,0),p+new Vector3(0,span.top*cellSize,0),col);
					span = span.next;
					count++;
				}
				
			}*/
			
			//return;
			//Step 2 - Navigable Space
			
			
			
			//ErodeWalkableArea (2);
			
			/*
			
			voxelOffset = min;
			
			AstarProfiler.StartProfile ("Build Distance Field");
			
			BuildDistanceField (min);
			
			AstarProfiler.EndProfile ("Build Distance Field");
			AstarProfiler.StartProfile ("Build Regions");
			
			BuildRegions (min);
			
			AstarProfiler.EndProfile ("Build Regions");
			
			AstarProfiler.StartProfile ("Build Contours");
			
			BuildContours ();
			
			AstarProfiler.EndProfile ("Build Contours");
			
			AstarProfiler.EndProfile ("Build Navigation Mesh");
			AstarProfiler.StartProfile ("Build Debug Mesh");
			
			int sCount = voxelArea.compactSpans.Length;
			Vector3[] debugPointsTop = new Vector3[sCount];
			Vector3[] debugPointsBottom = new Vector3[sCount];
			Color[] debugColors = new Color[sCount];
			
			int debugPointsCount = 0;
			
			//int wd = voxelArea.width*voxelArea.depth;
			
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					Vector3 p = new Vector3(x,0,pz)*cellSize+min;
					
					//CompactVoxelCell c = voxelArea.compactCells[x+z];
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					//if (c.count == 0) {
					//	Debug.DrawRay (p,Vector3.up,Color.red);
					//}
					
					for (int i=(int)c.index, ni = (int)(c.index+c.count);i<ni;i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						p.y = ((float)(s.y+0.1F))*cellHeight+min.y;
						
						debugPointsTop[debugPointsCount] = p;
						
						p.y = ((float)s.y)*cellHeight+min.y;
						debugPointsBottom[debugPointsCount] = p;
						
						debugColors[debugPointsCount] = //Color.Lerp (Color.black, Color.white , (float)voxelArea.dist[i] / (float)voxelArea.maxDistance);
						//Utility.GetColor ((int)s.area);
						Utility.IntToColor ((int)s.reg,0.8F);
						//Color.Lerp (Color.black, Color.white , (float)s.area / 10F);
						//(float)(Mathf.Abs(dst[i]-src[i])) / (float)5);//s.area == 1 ? Color.green : (s.area == 2 ? Color.yellow : Color.red);
						debugPointsCount++;
						
						//Debug.DrawRay (p,Vector3.up*0.5F,Color.green);
					}
				}
			}
	
			DebugUtility.DrawCubes (debugPointsTop,debugPointsBottom,debugColors, cellSize);
			
			AstarProfiler.EndProfile ("Build Debug Mesh");
			
			/*for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					Vector3 p = new Vector3(x,0,pz)*cellSize+min;
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					if (c.count == 0) {
						//Debug.DrawRay (p,Vector3.up,Color.red);
					}
					
					for (int i=(int)c.index, ni = (int)(c.index+c.count);i<ni;i++) {
						
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						p.y = ((float)s.y)*cellHeight+min.y;
						
						
						for (int d = 0; d<4;d++) {
							int conn = s.GetConnection (d);
							
							
							if (conn == NotConnected) {
								//Debug.DrawRay (p,Vector3.up*0.2F,Color.red);
								Debug.DrawRay (p,voxelArea.VectorDirection[d]*cellSize*0.5F,Color.red);
							} else {
								Debug.DrawRay (p,voxelArea.VectorDirection[d]*cellSize*0.5F,Color.green);
							}
						}
					}
				}
			}*/
			
			
			/*int sCount = voxelArea.compactSpans.Length;
			Vector3[] debugPointsTop = new Vector3[sCount];
			Vector3[] debugPointsBottom = new Vector3[sCount];
			Color[] debugColors = new Color[sCount];
			
			int debugPointsCount = 0;
			
			//int wd = voxelArea.width*voxelArea.depth;
			
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					Vector3 p = new Vector3(x,0,pz)*cellSize+min;
					
					//CompactVoxelCell c = voxelArea.compactCells[x+z];
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					//if (c.count == 0) {
					//	Debug.DrawRay (p,Vector3.up,Color.red);
					//}
					
					for (int i=(int)c.index, ni = (int)(c.index+c.count);i<ni;i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						p.y = ((float)(s.y+0.1F))*cellHeight+min.y;
						
						debugPointsTop[debugPointsCount] = p;
						
						p.y = ((float)s.y)*cellHeight+min.y;
						debugPointsBottom[debugPointsCount] = p;
						
						Color col = Color.black;
						
						switch (s.area) {
							case 0:
								col = Color.red;
								break;
							case 1:
								col = Color.green;
								break;
							case 2:
								col = Color.yellow;
								break;
							case 3:
								col = Color.magenta;
								break;
						}
						
						debugColors[debugPointsCount] = col;//Color.Lerp (Color.black, Color.white , (float)dst[i] / (float)voxelArea.maxDistance);//(float)(Mathf.Abs(dst[i]-src[i])) / (float)5);//s.area == 1 ? Color.green : (s.area == 2 ? Color.yellow : Color.red);
						debugPointsCount++;
						
						//Debug.DrawRay (p,Vector3.up*0.5F,Color.green);
					}
				}
			}
	
			DebugUtility.DrawCubes (debugPointsTop,debugPointsBottom,debugColors, cellSize);*/
			
			//AstarProfiler.PrintResults ();
			
			//firstStep = false;
			
		}
		
		public void BuildCompactField () {
			AstarProfiler.StartProfile ("Build Compact Voxel Field");
			
			//Build compact representation
			int spanCount = voxelArea.GetSpanCount ();
			
			voxelArea.compactSpanCount = spanCount;
			if (voxelArea.compactSpans == null || voxelArea.compactSpans.Length < spanCount) {
				voxelArea.compactSpans = new CompactVoxelSpan[spanCount];
				voxelArea.areaTypes = new int[spanCount];
			}
			
			uint idx = 0;
			
			int w = voxelArea.width;
			int d = voxelArea.depth;
			int wd = w*d;
			
			if (this.voxelWalkableHeight >= 0xFFFF) {
				Debug.LogWarning ("Too high walkable height to guarantee correctness. Increase voxel height or lower walkable height.");
			}
			
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
			LinkedVoxelSpan[] spans = voxelArea.linkedSpans;
#endif
			
			//Parallel.For (0, voxelArea.depth, delegate (int pz) {
			for (int z=0, pz = 0;z < wd;z += w, pz++) {
				
				for (int x=0;x < w;x++) {
					
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
					
					int spanIndex = x+z;
					if (spans[spanIndex].bottom == VoxelArea.InvalidSpanValue) {
						voxelArea.compactCells[x+z] = new CompactVoxelCell (0,0);
						continue;
					}
					
					uint index = idx;
					uint count = 0;
					
					//Vector3 p = new Vector3(x,0,pz)*cellSize+voxelOffset;
					
					while (spanIndex != -1) {
						
						if (spans[spanIndex].area != UnwalkableArea) {
							int bottom = (int)spans[spanIndex].top;
							int next = spans[spanIndex].next;
							int top = next != -1 ? (int)spans[next].bottom : VoxelArea.MaxHeightInt;
							
							voxelArea.compactSpans[idx] = new CompactVoxelSpan ((ushort)(bottom > 0xFFFF ? 0xFFFF : bottom) , (uint)(top-bottom > 0xFFFF ? 0xFFFF : top-bottom));
							voxelArea.areaTypes[idx] = spans[spanIndex].area;
							idx++;
							count++;
						}
						spanIndex = spans[spanIndex].next;
					}
					
					voxelArea.compactCells[x+z] = new CompactVoxelCell (index, count);
#else
					VoxelSpan s = voxelArea.cells[x+z].firstSpan;
					
					if (s == null) {
						voxelArea.compactCells[x+z] = new CompactVoxelCell (0,0);
						continue;
					}
					
					uint index = idx;
					uint count = 0;
					
					//Vector3 p = new Vector3(x,0,pz)*cellSize+voxelOffset;
					
					while (s != null) {
						
						if (s.area != UnwalkableArea) {
							int bottom = (int)s.top;
							int top = s.next != null ? (int)s.next.bottom : VoxelArea.MaxHeightInt;
							
							voxelArea.compactSpans[idx] = new CompactVoxelSpan ((ushort)Mathfx.Clamp (bottom, 0, 0xffff) , (uint)Mathfx.Clamp (top-bottom, 0, 0xffff));
							voxelArea.areaTypes[idx] = s.area;
							idx++;
							count++;
						}
						s = s.next;
					}
					
					voxelArea.compactCells[x+z] = new CompactVoxelCell (index, count);
#endif
				}
			}
			
			AstarProfiler.EndProfile ("Build Compact Voxel Field");
		}
		
		public void BuildVoxelConnections () {
			AstarProfiler.StartProfile ("Build Voxel Connections");
			
			int wd = voxelArea.width*voxelArea.depth;
			
			CompactVoxelSpan[] spans = voxelArea.compactSpans;
			CompactVoxelCell[] cells = voxelArea.compactCells;
			
			//Build voxel connections
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
			
			
			//System.Threading.ManualResetEvent[] handles = new System.Threading.ManualResetEvent[voxelArea.depth];
			
			//This will run the loop in multiple threads (speedup by ? 40%)
			//Parallel.For (0, voxelArea.depth, delegate (int pz) {
			//System.Threading.WaitCallback del = delegate (System.Object _pz) {
				//int pz = (int)_pz;
				//int z = pz*voxelArea.width;
				
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = cells[x+z];
					
					//Vector3 p = new Vector3(x,0,pz)*cellSize+voxelOffset;
					
					for (int i=(int)c.index, ni = (int)(c.index+c.count);i<ni;i++) {
						
						CompactVoxelSpan s = spans[i];
						
						spans[i].con = 0xFFFFFFFF;
						
						//p.y = ((float)s.y)*cellHeight+voxelOffset.y;
						
						for (int d=0;d<4;d++) {
							//s.SetConnection (d,NotConnected);
							//voxelArea.compactSpans[i].SetConnection (d,NotConnected);
							
							int nx = x+voxelArea.DirectionX[d];
							int nz = z+voxelArea.DirectionZ[d];
							
							if (nx < 0 || nz < 0 || nz >= wd || nx >= voxelArea.width) {
								continue;
							}
							
							CompactVoxelCell nc = cells[nx+nz];
							
							for (int k=(int)nc.index, nk = (int)(nc.index+nc.count); k<nk; k++) {
								
								CompactVoxelSpan ns = spans[k];
								
								int bottom = System.Math.Max (s.y,ns.y);
								
								int top = AstarMath.Min ((int)s.y+(int)s.h,(int)ns.y+(int)ns.h);
								
								if ((top-bottom) >= voxelWalkableHeight && System.Math.Abs ((int)ns.y - (int)s.y) <= voxelWalkableClimb) {
									uint connIdx = (uint)k - nc.index;
									
									if (connIdx > MaxLayers) {
										Debug.LogError ("Too many layers");
										continue;
									}
									
									spans[i].SetConnection (d,connIdx);
									break;
								}
								
							}
						}
					}
				}
				
				//handles[pz].Set ();
			//};
			//});
			}
			
			/*for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				handles[pz] = new System.Threading.ManualResetEvent(false);
				System.Threading.ThreadPool.QueueUserWorkItem (del, pz);
			}
			
			System.Threading.WaitHandle.WaitAll (handles);*/
			
			AstarProfiler.EndProfile ("Build Voxel Connections");
		}
		
		public void DrawLine (int a, int b, int[] indices, int[] verts, Color col) {
			int p1 = (indices[a] & 0x0fffffff) * 4;
			int p2 = (indices[b] & 0x0fffffff) * 4;
			
			Debug.DrawLine (ConvertPosCorrZ (verts[p1+0],verts[p1+1],verts[p1+2]),ConvertPosCorrZ (verts[p2+0],verts[p2+1],verts[p2+2]),col);
		}
		
		public Vector3 ConvertPos (int x, int y, int z) {
			Vector3 p = Vector3.Scale (
				new Vector3 (
					x+0.5F,
					y,
					(z/(float)voxelArea.width)+0.5F
				)
				,cellScale)
				+voxelOffset;
			return p;
		}
		
		public Vector3 ConvertPosCorrZ (int x, int y, int z) {
			Vector3 p = Vector3.Scale (
				new Vector3 (
					x,
					y,
					z
				)
				,cellScale)
				+voxelOffset;
			return p;
		}
		
		public Vector3 ConvertPosWithoutOffset (int x, int y, int z) {
			Vector3 p = Vector3.Scale (
				new Vector3 (
					x,
					y,
					(z/(float)voxelArea.width)
				)
				,cellScale)
				+voxelOffset;
			return p;
		}
		
			/*
		if (rcGetCon(s, dirp) != RC_NOT_CONNECTED)
		{
			const int ax = x + rcGetDirOffsetX(dirp);
			const int ay = y + rcGetDirOffsetY(dirp);
			const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
			const rcCompactSpan& as = chf.spans[ai];
			ch = rcMax(ch, (int)as.y);
			regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);
			if (rcGetCon(as, dir) != RC_NOT_CONNECTED)
			{
				const int ax2 = ax + rcGetDirOffsetX(dir);
				const int ay2 = ay + rcGetDirOffsetY(dir);
				const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dir);
				const rcCompactSpan& as2 = chf.spans[ai2];
				ch = rcMax(ch, (int)as2.y);
				regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
			}
		}
	
		// Check if the vertex is special edge vertex, these vertices will be removed later.
		for (int j = 0; j < 4; ++j)
		{
			const int a = j;
			const int b = (j+1) & 0x3;
			const int c = (j+2) & 0x3;
			const int d = (j+3) & 0x3;
			
			// The vertex is a border vertex there are two same exterior cells in a row,
			// followed by two interior cells and none of the regions are out of bounds.
			const bool twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
			const bool twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
			const bool intsSameArea = (regs[c]>>16) == (regs[d]>>16);
			const bool noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;
			if (twoSameExts && twoInts && intsSameArea && noZeros)
			{
				isBorderVertex = true;
				break;
			}
		}
		
		return ch;*/
		
			
		public Vector3 ConvertPosition (int x,int z, int i) {
			CompactVoxelSpan s = voxelArea.compactSpans[i];
			return new Vector3 (x*cellSize,s.y*cellHeight,(z/(float)voxelArea.width)*cellSize)+voxelOffset;
		}

		
		public void ErodeWalkableArea (int radius) {
			
			AstarProfiler.StartProfile ("Erode Walkable Area");
			
			ushort[] src = voxelArea.tmpUShortArr;
			if (src == null || src.Length < voxelArea.compactSpanCount) {
				src = voxelArea.tmpUShortArr = new ushort[voxelArea.compactSpanCount];
			}
			
			Pathfinding.Util.Memory.MemSet<ushort> (src, 0xffff, sizeof(ushort));
			//for (int i=0;i<src.Length;i++) {
			//	src[i] = 0xffff;
			//}
			
			CalculateDistanceField (src);
			
			for (int i=0;i<src.Length;i++) {
				//Note multiplied with 2 because the distance field increments distance by 2 for each voxel (and 3 for diagonal)
				if (src[i] < radius*2) {
					voxelArea.areaTypes[i] = UnwalkableArea;
				}
			}
			
			AstarProfiler.EndProfile ("Erode Walkable Area");
		}
		
		public void BuildDistanceField () {
			AstarProfiler.StartProfile ("Build Distance Field");
			
			ushort[] src = voxelArea.tmpUShortArr;
			if (src == null || src.Length < voxelArea.compactSpanCount) {
				src = voxelArea.tmpUShortArr = new ushort[voxelArea.compactSpanCount];
			}
			
			Pathfinding.Util.Memory.MemSet<ushort> (src, 0xffff, sizeof(ushort));
			//for (int i=0;i<src.Length;i++) {
			//	src[i] = 0xffff;
			//}
			
			voxelArea.maxDistance = CalculateDistanceField (src);
			
			ushort[] dst = voxelArea.dist;
			if (dst == null || dst.Length < voxelArea.compactSpanCount) {
				dst = new ushort[voxelArea.compactSpanCount];
			}
			
			dst = BoxBlur (src,dst);
			
			voxelArea.dist = dst;
			
			
			AstarProfiler.EndProfile ("Build Distance Field");
		}
			
		/** \todo Complete the ErodeVoxels function translation */
		[System.Obsolete ("This function is not complete and should not be used")]
		public void ErodeVoxels (int radius) {
			
			if (radius > 255) {
				Debug.LogError ("Max Erode Radius is 255");
				radius = 255;
			}
			
			int wd = voxelArea.width*voxelArea.depth;
			
			int[] dist = new int[voxelArea.compactSpanCount];
			
			for (int i=0;i<dist.Length;i++) {
				dist[i] = 0xFF;
			}
			
			for (int z=0;z < wd;z += voxelArea.width) {
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					for (int i= (int)c.index, ni = (int)(c.index+c.count); i < ni; i++) {
						
						if (voxelArea.areaTypes[i] != UnwalkableArea) {
							
							CompactVoxelSpan s = voxelArea.compactSpans[i];
							int nc = 0;
							for (int dir=0;dir<4;dir++) {
								if (s.GetConnection (dir) != NotConnected)
									nc++;
							}
							//At least one missing neighbour
							if (nc != 4) {
								dist[i] = 0;
							}
						}
					}
				}
			}
			
			//int nd = 0;
			
			//Pass 1
			
			/*for (int z=0;z < wd;z += voxelArea.width) {
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					for (int i= (int)c.index, ci = (int)(c.index+c.count); i < ci; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						if (s.GetConnection (0) != NotConnected) {
							// (-1,0)
							int nx = x+voxelArea.DirectionX[0];
							int nz = z+voxelArea.DirectionZ[0];
							
							int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (0));
							CompactVoxelSpan ns = voxelArea.compactSpans[ni];
							
							if (dist[ni]+2 < dist[i]) {
								dist[i] = (ushort)(dist[ni]+2);
							}
							
							if (ns.GetConnection (3) != NotConnected) {
								// (-1,0) + (0,-1) = (-1,-1)
								int nnx = nx+voxelArea.DirectionX[3];
								int nnz = nz+voxelArea.DirectionZ[3];
								
								int nni = (int)(voxelArea.compactCells[nnx+nnz].index+ns.GetConnection (3));
								
								if (src[nni]+3 < src[i]) {
									src[i] = (ushort)(src[nni]+3);
								}
							}
						}
						
						if (s.GetConnection (3) != NotConnected) {
							// (0,-1)
							int nx = x+voxelArea.DirectionX[3];
							int nz = z+voxelArea.DirectionZ[3];
								
							int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (3));
							
							if (src[ni]+2 < src[i]) {
								src[i] = (ushort)(src[ni]+2);
							}
							
							CompactVoxelSpan ns = voxelArea.compactSpans[ni];
							
							if (ns.GetConnection (2) != NotConnected) {
								
								// (0,-1) + (1,0) = (1,-1)
								int nnx = nx+voxelArea.DirectionX[2];
								int nnz = nz+voxelArea.DirectionZ[2];
								
								int nni = (int)(voxelArea.compactCells[nnx+nnz].index+ns.GetConnection (2));
								
								if (src[nni]+3 < src[i]) {
									src[i] = (ushort)(src[nni]+3);
								}
							}
						}
					}
				}
			}*/
		}
		
		public void FilterLowHeightSpans (uint voxelWalkableHeight, float cs, float ch, Vector3 min) {
			int wd = voxelArea.width*voxelArea.depth;
			
			//Filter all ledges
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
			LinkedVoxelSpan[] spans = voxelArea.linkedSpans;
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					for (int s = z+x; s != -1 && spans[s].bottom != VoxelArea.InvalidSpanValue; s = spans[s].next) {
						
						uint bottom = spans[s].top;
						uint top = spans[s].next != -1 ? spans[spans[s].next].bottom : VoxelArea.MaxHeight;
						
						if (top - bottom < voxelWalkableHeight) {
							spans[s].area = UnwalkableArea;
						}
					}
				}
			}
#else
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					for (VoxelSpan s = voxelArea.cells[z+x].firstSpan; s != null; s = s.next) {
						
						uint bottom = s.top;
						uint top = s.next != null ? s.next.bottom : VoxelArea.MaxHeight;
						
						if (top - bottom < voxelWalkableHeight) {
							s.area = UnwalkableArea;
						}
					}
				}
			}
#endif
			
		}
		
		//Code almost completely ripped from Recast
		public void FilterLedges (uint voxelWalkableHeight, int voxelWalkableClimb, float cs, float ch, Vector3 min) {
			
			int wd = voxelArea.width*voxelArea.depth;
			
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
			LinkedVoxelSpan[] spans = voxelArea.linkedSpans;
#endif
			
			int[] DirectionX = voxelArea.DirectionX;
			int[] DirectionZ = voxelArea.DirectionZ;
			
			int width = voxelArea.width;
			//int depth = voxelArea.depth;
			
			//Filter all ledges
			for (int z=0, pz = 0;z < wd;z += width, pz++) {
				for (int x=0;x < width;x++) {
					
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
					if (spans[x+z].bottom == VoxelArea.InvalidSpanValue) continue;
					
					for (int s = x+z; s != -1; s = spans[s].next) {
						
						//Skip non-walkable spans
						if (spans[s].area == UnwalkableArea) {
							continue;
						}
						
						int bottom = (int)spans[s].top;
						int top = spans[s].next != -1 ? (int)spans[spans[s].next].bottom : VoxelArea.MaxHeightInt;
						
						int minHeight = VoxelArea.MaxHeightInt;
						
						int aMinHeight = (int)spans[s].top;
						int aMaxHeight = aMinHeight;
						
						for (int d = 0; d < 4; d++) {
							int nx = x+DirectionX[d];
							int nz = z+DirectionZ[d];
							
							//Skip out-of-bounds points
							if (nx < 0 || nz < 0 || nz >= wd || nx >= width) {
								spans[s].area = UnwalkableArea;
								break;
							}
							
							int nsx = nx+nz;
							
							int nbottom = -voxelWalkableClimb;
							
							int ntop = spans[nsx].bottom != VoxelArea.InvalidSpanValue ? (int)spans[nsx].bottom : VoxelArea.MaxHeightInt;
							
							if (System.Math.Min (top,ntop) - System.Math.Max (bottom,nbottom) > voxelWalkableHeight) {
								minHeight = System.Math.Min (minHeight, nbottom - bottom);
							}
							
							//Loop through spans
							if (spans[nsx].bottom != VoxelArea.InvalidSpanValue) {
								for (int ns = nsx; ns != -1; ns = spans[ns].next) {
									nbottom = (int)spans[ns].top;
									ntop = spans[ns].next != -1 ? (int)spans[spans[ns].next].bottom : VoxelArea.MaxHeightInt;
									
									if (System.Math.Min (top, ntop) - System.Math.Max (bottom, nbottom) > voxelWalkableHeight) {
										minHeight = AstarMath.Min (minHeight, nbottom - bottom);
										
										if (System.Math.Abs (nbottom - bottom) <= voxelWalkableClimb) {
											if (nbottom < aMinHeight) { aMinHeight = nbottom; }
											if (nbottom > aMaxHeight) { aMaxHeight = nbottom; }
										}
									}
								}
							}
						}
						
						if (minHeight < -voxelWalkableClimb || (aMaxHeight - aMinHeight) > voxelWalkableClimb) {
							spans[s].area = UnwalkableArea;
						}
					}
#else
					for (VoxelSpan s = voxelArea.cells[z+x].firstSpan; s != null; s = s.next) {
						
						//Skip non-walkable spans
						if (s.area == UnwalkableArea) {
							continue;
						}
						
						int bottom = (int)s.top;
						int top = s.next != null ? (int)s.next.bottom : VoxelArea.MaxHeightInt;
						
						int minHeight = VoxelArea.MaxHeightInt;
						
						int aMinHeight = (int)s.top;
						int aMaxHeight = (int)s.top;
						
						for (int d = 0; d < 4; d++) {
							
							int nx = x+voxelArea.DirectionX[d];
							int nz = z+voxelArea.DirectionZ[d];
							
							//Skip out-of-bounds points
							if (nx < 0 || nz < 0 || nz >= wd || nx >= voxelArea.width) {
								s.area = UnwalkableArea;
								break;
							}
							
							VoxelSpan nsx = voxelArea.cells[nx+nz].firstSpan;
							
							int nbottom = -voxelWalkableClimb;
							
							int ntop = nsx != null ? (int)nsx.bottom : VoxelArea.MaxHeightInt;
							
							if (Mathfx.Min (top,ntop) - Mathfx.Max (bottom,nbottom) > voxelWalkableHeight) {
								minHeight = Mathfx.Min (minHeight, nbottom - bottom);
							}
							
							//Loop through spans
							for (VoxelSpan ns = nsx; ns != null; ns = ns.next) {
								nbottom = (int)ns.top;
								ntop = ns.next != null ? (int)ns.next.bottom : VoxelArea.MaxHeightInt;
								
								if (Mathfx.Min (top, ntop) - Mathfx.Max (bottom, nbottom) > voxelWalkableHeight) {
									minHeight = Mathfx.Min (minHeight, nbottom - bottom);
									
									if (Mathfx.Abs (nbottom - bottom) <= voxelWalkableClimb) {
										if (nbottom < aMinHeight) { aMinHeight = nbottom; }
										if (nbottom > aMaxHeight) { aMaxHeight = nbottom; }
									}
								}
							}
						}
						
						if (minHeight < -voxelWalkableClimb || (aMaxHeight - aMinHeight) > voxelWalkableClimb) {
							s.area = UnwalkableArea;
						}
					}
#endif
					
				}
			}
		}
	}
}