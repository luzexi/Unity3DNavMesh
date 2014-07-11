//#define ASTARDEBUG
//#define ASTAR_DEBUGREPLAY
//#define ASTAR_RECAST_BFS //Enables an experimental optimization to recast. Faster scanning and updating but can sometimes fail to generate any navmesh at all (this is relatively rare though).
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.Voxels;

namespace Pathfinding.Voxels {
	public partial class Voxelize {
		
		public ushort[] ExpandRegions (int maxIterations, uint level, ushort[] srcReg, ushort[] srcDist, ushort[] dstReg, ushort[] dstDist, List<int> stack) {
			
			AstarProfiler.StartProfile ("---Expand 1");
			int w = voxelArea.width;
			int d = voxelArea.depth;
			
			int wd = w*d;

#if ASTAR_RECAST_BFS && FALSE
			List<int> st1 = new List<int>();
			List<int> st2 = new List<int>();
			
			for (int z=0, pz = 0;z < wd;z += w, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = voxelArea.compactCells[z+x];
					
					for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; i++) {
						
						if (voxelArea.dist[i] >= level && srcReg[i] == 0 && voxelArea.areaTypes[i] != UnwalkableArea) {
							
							st2.Add (x);
							st2.Add (z);
							st2.Add (i);
							//Debug.DrawRay (ConvertPosition(x,z,i),Vector3.up*0.5F,Color.cyan);
						}
					}
				}
			}
			throw new System.NotImplementedException();
			return null;
			
#else
			
			// Find cells revealed by the raised level.
			stack.Clear ();
			
			for (int z=0, pz = 0;z < wd;z += w, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = voxelArea.compactCells[z+x];
					
					for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; i++) {
						
						if (voxelArea.dist[i] >= level && srcReg[i] == 0 && voxelArea.areaTypes[i] != UnwalkableArea) {
							stack.Add (x);
							stack.Add (z);
							stack.Add (i);
							//Debug.DrawRay (ConvertPosition(x,z,i),Vector3.up*0.5F,Color.cyan);
						}
					}
				}
			}
			
			AstarProfiler.EndProfile ("---Expand 1");
			AstarProfiler.StartProfile ("---Expand 2");
			
			int iter = 0;
			
			int stCount = stack.Count;
			
			if (stCount > 0) while (true) {
				
				int failed = 0;
				
				AstarProfiler.StartProfile ("---- Copy");	
				
				System.Buffer.BlockCopy (srcReg, 0, dstReg, 0, srcReg.Length*sizeof(ushort));
				System.Buffer.BlockCopy (srcDist, 0, dstDist, 0, dstDist.Length*sizeof(ushort));
				
				/*//NOTE: half of execution time is spent here, optimize!!
				for (int i=0;i<srcReg.Length;i++) {
					dstReg[i] = srcReg[i];
				}
				
				for (int i=0;i<srcDist.Length;i++) {
					dstDist[i] = srcDist[i];
				}*/
				
				AstarProfiler.EndProfile ("---- Copy");
				
				for (int j=0;j<stCount;j += 3) {
					
					if (j >= stCount) break;
					
					int x = stack[j];
					int z = stack[j+1];
					int i = stack[j+2];
					
					if (i < 0) {
						//Debug.DrawRay (ConvertPosition(x,z,i),Vector3.up*2,Color.blue);
						failed++;
						continue;
					}
					
					ushort r = srcReg[i];
					ushort d2 = 0xffff;
					
					CompactVoxelSpan s = voxelArea.compactSpans[i];
					int area = voxelArea.areaTypes[i];
					
					for (int dir = 0; dir < 4; dir++) {
						
						if (s.GetConnection (dir) == NotConnected) { continue; }
						
						int nx = x + voxelArea.DirectionX[dir];
						int nz = z + voxelArea.DirectionZ[dir];
						
						int ni = (int)voxelArea.compactCells[nx+nz].index + s.GetConnection (dir);
						
						if (area != voxelArea.areaTypes[ni]) { continue; }
						
						if (srcReg[ni] > 0 && (srcReg[ni] & BorderReg) == 0) {
							if ((int)srcDist[ni]+2 < (int)d2)
							{
								r = srcReg[ni];
								d2 = (ushort)(srcDist[ni]+2);
							}
						}
					}
					
					if (r != 0) {
						
						stack[j+2] = -1; // mark as used
						dstReg[i] = r;
						dstDist[i] = d2;
						
					} else {
						failed++;
						//Debug.DrawRay (ConvertPosition(x,z,i),Vector3.up*2,Color.red);
					}
					
				}
				
				// Swap source and dest.
				ushort[] tmp = srcReg;
				srcReg = dstReg;
				dstReg = tmp;
				
				tmp = srcDist;
				srcDist = dstDist;
				dstDist = tmp;
				
				if (failed*3 >= stCount) {
					//Debug.Log("Failed count broke "+failed);
					break;
				}
				
				if (level > 0) {
					iter++;
	
					if (iter >= maxIterations) {
						//Debug.Log("Iterations broke");
						break;
					}
				}
			}
			
			AstarProfiler.EndProfile ("---Expand 2");
			
			return srcReg;
#endif			
		}
		
		public bool FloodRegion (int x, int z, int i, uint level, ushort r, ushort[] srcReg, ushort[] srcDist, List<int> stack) {
		
			int area = voxelArea.areaTypes[i];
		
			// Flood fill mark region.
			stack.Clear ();
			
			stack.Add (x);
			stack.Add (z);
			stack.Add (i);
			
			srcReg[i] = r;
			srcDist[i] = 0;
			
			int lev = (int)(level >= 2 ? level-2 : 0);
			
			int count = 0;
			
			
			
			while (stack.Count > 0) {
				
				//Similar to the Pop operation of an array, but Pop is not implemented in List<>
				int ci = stack[stack.Count-1]; stack.RemoveAt(stack.Count-1);
				int cz = stack[stack.Count-1]; stack.RemoveAt(stack.Count-1);
				int cx = stack[stack.Count-1]; stack.RemoveAt(stack.Count-1);
				
				
				CompactVoxelSpan cs = voxelArea.compactSpans[ci];
				
				//Debug.DrawRay (ConvertPosition(cx,cz,ci),Vector3.up, Color.cyan);
				
				// Check if any of the neighbours already have a valid region set.
				ushort ar = 0;
				
				for (int dir = 0; dir < 4; dir++)
				{
					// 8 connected
					if (cs.GetConnection (dir) != NotConnected)
					{
						int ax = cx + voxelArea.DirectionX[dir];
						int az = cz + voxelArea.DirectionZ[dir];
						
						int ai = (int)voxelArea.compactCells[ax+az].index + cs.GetConnection (dir);
						
						if (voxelArea.areaTypes[ai] != area)
							continue;
							
						ushort nr = srcReg[ai];
						
						if ((nr & BorderReg) == BorderReg) // Do not take borders into account.
							continue;
							
						if (nr != 0 && nr != r)
							ar = nr;
						
						CompactVoxelSpan aspan = voxelArea.compactSpans[ai];
						
						int dir2 = (dir+1) & 0x3;
						if (aspan.GetConnection (dir2) != NotConnected)
						{
							int ax2 = ax + voxelArea.DirectionX[dir2];
							int az2 = az + voxelArea.DirectionZ[dir2];
							
							int ai2 = (int)voxelArea.compactCells[ax2+az2].index + aspan.GetConnection(dir2);
							
							if (voxelArea.areaTypes[ai2] != area)
								continue;
								
							nr = srcReg[ai2];
							if (nr != 0 && nr != r) {
								ar = nr;
							}
						}
					}
				}
				
				if (ar != 0)
				{
					srcReg[ci] = 0;
					continue;
				}
				count++;
				
				// Expand neighbours.
				for (int dir = 0; dir < 4; ++dir)
				{
					if (cs.GetConnection (dir) != NotConnected)
					{
						int ax = cx + voxelArea.DirectionX[dir];
						int az = cz + voxelArea.DirectionZ[dir];
						int ai = (int)voxelArea.compactCells[ax+az].index + cs.GetConnection (dir);
						
						if (voxelArea.areaTypes[ai] != area)
							continue;
							
						if (voxelArea.dist[ai] >= lev && srcReg[ai] == 0)
						{
							srcReg[ai] = r;
							srcDist[ai] = 0;
							
							stack.Add(ax);
							stack.Add(az);
							stack.Add(ai);
						}
					}
				}
			}
			
			
			return count > 0;
		}
		
							
		public void MarkRectWithRegion (int minx, int maxx, int minz, int maxz, ushort region, ushort[] srcReg) {
			int md = maxz * voxelArea.width;
				
			for (int z= minz*voxelArea.width;z < md;z += voxelArea.width) {
				for (int x=minx;x < maxx;x++) {
					CompactVoxelCell c = voxelArea.compactCells[z+x];
					
					for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; i++) {
						
						//voxelArea.compactSpans[i].area = area;
						
						if (voxelArea.areaTypes[i] != UnwalkableArea) {
							srcReg[i] = region;
						}
					}
				}
			}
		}
		
		public ushort CalculateDistanceField (ushort[] src) {
			
			int wd = voxelArea.width*voxelArea.depth;
			
			//Mark boundary cells
			for (int z=0;z < wd;z += voxelArea.width) {
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					for (int i= (int)c.index, ci = (int)(c.index+c.count); i < ci; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						//int area = voxelArea.areaTypes[i];
						
						int nc = 0;
						for (int d = 0; d < 4; d++) {
							if (s.GetConnection (d) != NotConnected) {
								
								/*int nx = x+voxelArea.DirectionX[d];
								int nz = z+voxelArea.DirectionZ[d];
								
								int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (d));*/
								
								//This function (CalculateDistanceField) is used for both ErodeWalkableArea and by itself.
								//The C++ recast source uses different code for those two cases, but I have found it works with one function
								//the voxelArea.areaTypes[ni] will actually only be one of two cases when used from ErodeWalkableArea
								//so it will have the same effect as
								// if (area != UnwalkableArea) {
								//This line is the one where the differ most
								
								nc++;
#if FALSE
								if (area == voxelArea.areaTypes[ni]) {
									nc++;
								} else {
									//No way we can reach 4
									break;
								}
#endif
							} else {
								break;
							}
						}
						
						if (nc != 4) {
							src[i] = 0;
						}
					}
				}
			}
			
			//Pass 1
			
			for (int z=0;z < wd;z += voxelArea.width) {
				for (int x=0;x < voxelArea.width;x++) {
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					for (int i= (int)c.index, ci = (int)(c.index+c.count); i < ci; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						if (s.GetConnection (0) != NotConnected) {
							// (-1,0)
							int nx = x+voxelArea.DirectionX[0];
							int nz = z+voxelArea.DirectionZ[0];
								
							int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (0));
							
							if (src[ni]+2 < src[i]) {
								src[i] = (ushort)(src[ni]+2);
							}
							
							CompactVoxelSpan ns = voxelArea.compactSpans[ni];
							
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
			}
			
			//Pass 2
			
			for (int z= wd-voxelArea.width;z >= 0;z -= voxelArea.width) {
				for (int x= voxelArea.width-1;x >= 0;x--) {
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					for (int i= (int)c.index, ci = (int)(c.index+c.count); i < ci; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						if (s.GetConnection (2) != NotConnected) {
							// (-1,0)
							int nx = x+voxelArea.DirectionX[2];
							int nz = z+voxelArea.DirectionZ[2];
								
							int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (2));
							
							if (src[ni]+2 < src[i]) {
								src[i] = (ushort)(src[ni]+2);
							}
							
							CompactVoxelSpan ns = voxelArea.compactSpans[ni];
							
							if (ns.GetConnection (1) != NotConnected) {
								// (-1,0) + (0,-1) = (-1,-1)
								int nnx = nx+voxelArea.DirectionX[1];
								int nnz = nz+voxelArea.DirectionZ[1];
								
								int nni = (int)(voxelArea.compactCells[nnx+nnz].index+ns.GetConnection (1));
								
								if (src[nni]+3 < src[i]) {
									src[i] = (ushort)(src[nni]+3);
								}
							}
						}
						
						if (s.GetConnection (1) != NotConnected) {
							// (0,-1)
							int nx = x+voxelArea.DirectionX[1];
							int nz = z+voxelArea.DirectionZ[1];
								
							int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (1));
							
							if (src[ni]+2 < src[i]) {
								src[i] = (ushort)(src[ni]+2);
							}
							
							CompactVoxelSpan ns = voxelArea.compactSpans[ni];
							
							if (ns.GetConnection (0) != NotConnected) {
								
								// (0,-1) + (1,0) = (1,-1)
								int nnx = nx+voxelArea.DirectionX[0];
								int nnz = nz+voxelArea.DirectionZ[0];
								
								int nni = (int)(voxelArea.compactCells[nnx+nnz].index+ns.GetConnection (0));
								
								if (src[nni]+3 < src[i]) {
									src[i] = (ushort)(src[nni]+3);
								}
							}
						}
					}
				}
			}
			
			ushort maxDist = 0;
			
			for (int i=0;i<voxelArea.compactSpanCount;i++) {
				maxDist = System.Math.Max (src[i],maxDist);
			}
			
			
			return maxDist;
		}
		
		public ushort[] BoxBlur (ushort[] src, ushort[] dst) {
			
			ushort thr = 20;
				
			int wd = voxelArea.width*voxelArea.depth;
			
			for (int z= wd-voxelArea.width;z >= 0;z -= voxelArea.width) {
				for (int x= voxelArea.width-1;x >= 0;x--) {
					
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					
					for (int i= (int)c.index, ci = (int)(c.index+c.count); i < ci; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						ushort cd = src[i];
						
						if (cd < thr) {
							dst[i] = cd;
							continue;
						}
						
						int total = (int)cd;
						
						for (int d=0; d < 4; d++) {
							
							if (s.GetConnection (d) != NotConnected) {
								int nx = x+voxelArea.DirectionX[d];
								int nz = z+voxelArea.DirectionZ[d];
								
								int ni = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (d));
								
								total += (int)src[ni];
								
								CompactVoxelSpan ns = voxelArea.compactSpans[ni];
								
								int d2 = (d+1) & 0x3;
								
								if (ns.GetConnection (d2) != NotConnected) {
									int nnx = nx+voxelArea.DirectionX[d2];
									int nnz = nz+voxelArea.DirectionZ[d2];
								
									int nni = (int)(voxelArea.compactCells[nnx+nnz].index+ns.GetConnection (d2));
									total += (int)src[nni];
								} else {
									total += cd;
								}
							} else {
								total += cd*2;
							}
						}
						dst[i] = (ushort)((total+5)/9F);
					}
				}
			}
			return dst;
		}
		
		void FloodOnes (List<Int3> st1, ushort[] regs, uint level, ushort reg) {
			
			for (int j=0;j<st1.Count;j++) {
				int x = st1[j].x;
				int i = st1[j].y;
				int z = st1[j].z;
				regs[i] = reg;
				
				CompactVoxelSpan s = voxelArea.compactSpans[i];
				int area = voxelArea.areaTypes[i];
				
				for (int dir = 0; dir < 4; dir++) {
					
					if (s.GetConnection (dir) == NotConnected) { continue; }
					
					int nx = x + voxelArea.DirectionX[dir];
					int nz = z + voxelArea.DirectionZ[dir];
					
					int ni = (int)voxelArea.compactCells[nx+nz].index + s.GetConnection (dir);
					
					if (area != voxelArea.areaTypes[ni]) { continue; }
					
					
					if (regs[ni] == 1) {
						//if ((int)srcDist[ni]+2 < (int)d2)
						//{
						//visited[i] = true;
						regs[ni] = reg;
						st1.Add (new Int3(nx,ni,nz));
							//d2 = (ushort)(srcDist[ni]+2);
						//}
					}
				}
			}
		}
		
		public void BuildRegions () {
			
			AstarProfiler.StartProfile ("Build Regions");
			
			int w = voxelArea.width;
			int d = voxelArea.depth;
			
			int wd = w*d;
			
			int expandIterations = 8;
			
			int spanCount = voxelArea.compactSpanCount;
			
			
			//new List<int>(1024);
			//List<int> visited = new List<int>(1024);
			
			

#if ASTAR_RECAST_BFS
			ushort[] srcReg = voxelArea.tmpUShortArr;
			if (srcReg.Length < spanCount) {
				srcReg = voxelArea.tmpUShortArr = new ushort[spanCount];
			}
			Pathfinding.Util.Memory.MemSet<ushort> (srcReg,0,sizeof(ushort));
#else
			List<int> stack = Pathfinding.Util.ListPool<int>.Claim(1024);
			ushort[] srcReg = new ushort[spanCount];
			ushort[] srcDist = new ushort[spanCount];
			ushort[] dstReg = new ushort[spanCount];
			ushort[] dstDist = new ushort[spanCount];
			
#endif
			ushort regionId = 2;
			MarkRectWithRegion (0, borderSize, 0, d, 	(ushort)(regionId | BorderReg), srcReg);	regionId++;
			MarkRectWithRegion (w-borderSize, w, 0, d, 	(ushort)(regionId | BorderReg), srcReg);	regionId++;
			MarkRectWithRegion (0, w, 0, borderSize, 	(ushort)(regionId | BorderReg), srcReg);	regionId++;
			MarkRectWithRegion (0, w, d-borderSize, d, 	(ushort)(regionId | BorderReg), srcReg);	regionId++;
			
			
			
			
			
#if ASTAR_RECAST_BFS
			
			uint level = 0;
			
			List<Int3> basins = Pathfinding.Util.ListPool<Int3>.Claim (100);//new List<Int3>();
			
			// Find "basins"
			DebugReplay.BeginGroup ("Basins");
			for (int z=0, pz = 0;z < wd;z += w, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
				
					CompactVoxelCell c = voxelArea.compactCells[z+x];
					
					for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; i++) {
						
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						bool anyBelow = false;
						
						if (voxelArea.areaTypes[i] == UnwalkableArea || srcReg[i] != 0) continue;
						
						for (int dir=0; dir < 4; dir++) {
							
							
							if (s.GetConnection (dir) != NotConnected) {
								int nx = x+voxelArea.DirectionX[dir];
								int nz = z+voxelArea.DirectionZ[dir];
								
								int ni2 = (int)(voxelArea.compactCells[nx+nz].index+s.GetConnection (dir));
								
								if (voxelArea.dist[i] < voxelArea.dist[ni2]) {
									anyBelow = true;
									break;
								}
								
								
								//CompactVoxelSpan ns = voxelArea.compactSpans[ni];
							}
						}
						if (!anyBelow) {
							//Debug.DrawRay (this.ConvertPosition(x,z,i),Vector3.down,Color.red);
							DebugReplay.DrawCube (this.ConvertPosition(x,z,i),cellScale,Color.red);
							basins.Add (new Int3(x,i,z));
							//System.Console.WriteLine ("Basin at " + voxelArea.dist[i]);
							level = System.Math.Max (level, voxelArea.dist[i]);
						}
					}
				}
			}
			
			//Start at maximum possible distance. & ~1 is rounding down to an even value
			level = (uint)((level+1) & ~1);
			
			DebugReplay.EndGroup();
			DebugReplay.BeginGroup ("BFS");
			
			List<Int3> st1 = Pathfinding.Util.ListPool<Int3>.Claim(300);
			List<Int3> st2 = Pathfinding.Util.ListPool<Int3>.Claim(300);
			//bool visited = new bool[voxelArea.compactSpanCount];
			
			for (;; level-= 2) {
				DebugReplay.BeginGroup ("BFS " + level);
				//System.Console.WriteLine ("Starting level " + level + " with st1.Count = " + st1.Count);
				
				int ocount = st1.Count;
				int expandCount = 0;
				
				if (ocount == 0) {
					//int c = 0;
					for (int q=0;q<basins.Count;q++) {
						if (voxelArea.dist[basins[q].y] >= level) {
							DebugReplay.DrawCube (this.ConvertPosition(basins[q].x,basins[q].z,basins[q].y)+Vector3.up,cellScale,new Color(0,1,0,0.5f));
						}
						
						if (srcReg[basins[q].y] == 0 && voxelArea.dist[basins[q].y] >= level) {
							
							srcReg[basins[q].y] = 1;
							st1.Add (basins[q]);
							//c++;
							//visited[basins[i].y] = true;
						}
					}
				}
				
				for (int j=0;j<st1.Count;j++) {
					
					int x = st1[j].x;
					int i = st1[j].y;
					int z = st1[j].z;
					
					ushort r = srcReg[i];
					
					CompactVoxelSpan s = voxelArea.compactSpans[i];
					int area = voxelArea.areaTypes[i];
					
					DebugReplay.DrawCube (this.ConvertPosition(x,z,i),cellScale,Mathfx.IntToColor(srcReg[i],0.7f));
					
					bool anyAbove = false;
					for (int dir = 0; dir < 4; dir++) {
						
						if (s.GetConnection (dir) == NotConnected) { continue; }
						
						int nx = x + voxelArea.DirectionX[dir];
						int nz = z + voxelArea.DirectionZ[dir];
						
						int ni = (int)voxelArea.compactCells[nx+nz].index + s.GetConnection (dir);
						
						if (area != voxelArea.areaTypes[ni]) { continue; }
						
						if (voxelArea.dist[ni] < level) {
							anyAbove = true;
							continue;
						}
						
						
						if (srcReg[ni] == 0) {
							
							bool same = false;
							for (int v=(int)voxelArea.compactCells[nx+nz].index, vt=(int)voxelArea.compactCells[nx+nz].index+(int)voxelArea.compactCells[nx+nz].count;v < vt; v++) {
								
								if (srcReg[v] == srcReg[i]) {
									same = true;
									break;
								}
							}
							
							if (!same) {
								//if ((int)srcDist[ni]+2 < (int)d2)
								//{
								//visited[i] = true;
								srcReg[ni] = r;
								//Debug.DrawRay (ConvertPosition(x,z,i),Vector3.up,Mathfx.IntToColor((int)level,0.6f));
								
								//if (dstReg[ni] == 0)
								st1.Add (new Int3(nx,ni,nz));
									//d2 = (ushort)(srcDist[ni]+2);
								//}
							}
						}
					}
					
					//Still on the edge
					if (anyAbove) {
						st2.Add (st1[j]);
					}
					
					if (j == ocount-1) {
						expandCount++;
						ocount = st1.Count;
						
						if (expandCount == 8 || j == st1.Count-1) {
						
						//int c = 0;
							for (int q=0;q<basins.Count;q++) {
								if (voxelArea.dist[basins[q].y] >= level) {
									DebugReplay.DrawCube (this.ConvertPosition(basins[q].x,basins[q].z,basins[q].y)+Vector3.up,cellScale,new Color(0,1,0,0.5f));
								}
								
								if (srcReg[basins[q].y] == 0 && voxelArea.dist[basins[q].y] >= level) {
									
									srcReg[basins[q].y] = 1;
									st1.Add (basins[q]);
									//c++;
									//visited[basins[i].y] = true;
								}
							}
						}
					}
				}
				
				
				
				List<Int3> tmpList = st1;
				st1 = st2;
				st2 = tmpList;
				st2.Clear();
				
				//System.Console.WriteLine ("Flooding basins");
				
				for (int i=0;i<basins.Count;i++) {
					if (srcReg[basins[i].y] == 1) {
						st2.Add (basins[i]);
						FloodOnes (st2, srcReg, level, regionId); regionId++;
						st2.Clear();
					}
				}
				
				
				//System.Console.WriteLine ("Added " + c + " basins");
				DebugReplay.EndGroup();
				if (level == 0) break;
			}
			
			DebugReplay.EndGroup();
			
			Pathfinding.Util.ListPool<Int3>.Release (st1);
			Pathfinding.Util.ListPool<Int3>.Release (st2);
			Pathfinding.Util.ListPool<Int3>.Release (basins);
			// Filter out small regions.
			voxelArea.maxRegions = regionId;
			
			FilterSmallRegions (srcReg, minRegionSize, voxelArea.maxRegions);
			
			
			// Write the result out.
			for (int i = 0; i < voxelArea.compactSpanCount; i++) {
				voxelArea.compactSpans[i].reg = srcReg[i];
			}
			
			
#else       /// ====== Use original recast code ====== //
			
			
			//Start at maximum possible distance. & ~1 is rounding down to an even value
			uint level = (uint)((voxelArea.maxDistance+1) & ~1);
			
			int count = 0;
			
			
			
			while (level > 0) {
				level = level >= 2 ? level-2 : 0;
				
				AstarProfiler.StartProfile ("--Expand Regions");
				if (ExpandRegions (expandIterations, level, srcReg, srcDist, dstReg, dstDist, stack) != srcReg) {
					ushort[] tmp = srcReg;
					srcReg = dstReg;
					dstReg = tmp;
					
					tmp = srcDist;
					srcDist = dstDist;
					dstDist = tmp;
				}
				
				AstarProfiler.EndProfile ("--Expand Regions");
				
				AstarProfiler.StartProfile ("--Mark Regions");
				
				
				// Mark new regions with IDs.
				// Find "basins"
				for (int z=0, pz = 0;z < wd;z += w, pz++) {
					for (int x=0;x < voxelArea.width;x++) {
					
						CompactVoxelCell c = voxelArea.compactCells[z+x];
						
						for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; i++) {
							
							if (voxelArea.dist[i] < level || srcReg[i] != 0 || voxelArea.areaTypes[i] == UnwalkableArea)
								continue;
								
							if (FloodRegion(x, z, i, level, regionId, srcReg, srcDist, stack))
								regionId++;
						}
					}
				}
				
				
				AstarProfiler.EndProfile ("--Mark Regions");
				
				count++;
				
				//if (count == 10) {
				//	return;
				//}
			}
			
			if (ExpandRegions (expandIterations*8, 0, srcReg, srcDist, dstReg, dstDist, stack) != srcReg) {
				ushort[] tmp = srcReg;
				srcReg = dstReg;
				dstReg = tmp;
				
				tmp = srcDist;
				srcDist = dstDist;
				dstDist = tmp;
			}
			
			// Filter out small regions.
			voxelArea.maxRegions = regionId;
			
			FilterSmallRegions (srcReg, minRegionSize, voxelArea.maxRegions);
			                    
			// Write the result out.
			for (int i = 0; i < voxelArea.compactSpanCount; i++) {
				voxelArea.compactSpans[i].reg = srcReg[i];
			}
			
			Pathfinding.Util.ListPool<int>.Release (stack);
			
			
			/*
			int sCount = voxelArea.GetSpanCount ();
			Vector3[] debugPointsTop = new Vector3[sCount];
			Vector3[] debugPointsBottom = new Vector3[sCount];
			Color[] debugColors = new Color[sCount];
			
			int debugPointsCount = 0;
			//int wd = voxelArea.width*voxelArea.depth;
			
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					
					Vector3 p = new Vector3(x,0,pz)*cellSize+forcedBounds.min;
					
					//CompactVoxelCell c = voxelArea.compactCells[x+z];
					CompactVoxelCell c = voxelArea.compactCells[x+z];
					//if (c.count == 0) {
					//	Debug.DrawRay (p,Vector3.up,Color.red);
					//}
					
					//for (int i=(int)c.index, ni = (int)(c.index+c.count);i<ni;i++) 
					
					for (int i = (int)c.index; i < c.index+c.count; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						//CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						p.y = ((float)(s.y+0.1F))*cellHeight+forcedBounds.min.y;
						
						debugPointsTop[debugPointsCount] = p;
						
						p.y = ((float)s.y)*cellHeight+forcedBounds.min.y;
						debugPointsBottom[debugPointsCount] = p;
						
						debugColors[debugPointsCount] = Pathfinding.Mathfx.IntToColor(s.reg,0.7f);//s.reg == 1 ? Color.green : (s.reg == 2 ? Color.yellow : Color.red);
						debugPointsCount++;
						
						//Debug.DrawRay (p,Vector3.up*0.5F,Color.green);
					}
				}
			}
			
			DebugUtility.DrawCubes (debugPointsTop,debugPointsBottom,debugColors, cellSize);*/
#endif
			AstarProfiler.EndProfile ("Build Regions");
		}
		
		static int union_find_find (int[] arr, int x) {
			if (arr[x] < 0) return x;
			return arr[x] = union_find_find(arr, arr[x]);
		}
		
		static void union_find_union (int[] arr, int a, int b) {
			a = union_find_find(arr, a);
			b = union_find_find(arr, b);
			if (a == b) return;
			if (arr[a] > arr[b]) {
				int tmp = a;
				a = b;
				b = tmp;
			}
			arr[a] += arr[b];
			arr[b] = a;
		}
		
		/** Filters out or merges small regions.
		  */
		public void FilterSmallRegions (ushort[] reg, int minRegionSize, int maxRegions) {
			
			/*int maxID = 0;
			for (int i=0;i<reg.Length;i++) {
				
				maxID = Mathf.Max (maxID,(int)reg[i]);
			}*/
			
			RelevantGraphSurface c = RelevantGraphSurface.Root;
			bool anySurfaces = c != null && (relevantGraphSurfaceMode != RecastGraph.RelevantGraphSurfaceMode.DoNotRequire);
			
			// Nothing to do here
			if (!anySurfaces && minRegionSize <= 0) {
				return;
			}
			
			int[] counter = new int[maxRegions];
			
			ushort[] bits = voxelArea.tmpUShortArr;
			if (bits == null || bits.Length < maxRegions) {
				bits = voxelArea.tmpUShortArr = new ushort[maxRegions];
			}
			
			Util.Memory.MemSet (counter, -1, sizeof(int));
			Util.Memory.MemSet (bits, (ushort)0, maxRegions, sizeof(ushort));
			
			
			int nReg = counter.Length;			
			
			int wd = voxelArea.width*voxelArea.depth;
			
			const int RelevantSurfaceSet = 1 << 1;
			const int BorderBit = 1 << 0;
			
			// Mark RelevantGraphSurfaces
			
			// If they can also be adjacent to tile borders, this will also include the BorderBit
			int RelevantSurfaceCheck = RelevantSurfaceSet | ((relevantGraphSurfaceMode == RecastGraph.RelevantGraphSurfaceMode.OnlyForCompletelyInsideTile) ? BorderBit : 0x0);
			
			if (anySurfaces) {
				while (c != null) {
					int x, z;
					this.VectorToIndex (c.Position, out x, out z);
					
					// Out of bounds
					if (x < 0 || z < 0 || x >= voxelArea.width || z >= voxelArea.depth) {
						c = c.Next;
						continue;
					}
					
					int y = (int)((c.Position.y - voxelOffset.y)/cellHeight);
					int rad = (int)(c.maxRange / cellHeight);
					
					CompactVoxelCell cell = voxelArea.compactCells[x+z*voxelArea.width];
					for (int i = (int)cell.index; i < cell.index+cell.count; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						if (System.Math.Abs (s.y - y) <= rad && reg[i] != 0) {
							bits[union_find_find (counter, (int)reg[i] & ~BorderReg)] |= RelevantSurfaceSet;
						}
					}
					
					c = c.Next;
				}
			}
			
			for (int z=0, pz = 0;z < wd;z += voxelArea.width, pz++) {
				for (int x=0;x < voxelArea.width;x++) {
					CompactVoxelCell cell = voxelArea.compactCells[x+z];
					
					for (int i = (int)cell.index; i < cell.index+cell.count; i++) {
						CompactVoxelSpan s = voxelArea.compactSpans[i];
						
						int r = (int)reg[i];
						
						if ((r & ~BorderReg) == 0) continue;
						
						if (r >= nReg) { //Probably border
							bits[union_find_find (counter, r & ~BorderReg)] |= BorderBit;
							continue;
						}
						
						int k = union_find_find(counter,r);
						// Count this span
						counter[k]--;
						
						for (int dir = 0; dir < 4; dir++) {
						
							if (s.GetConnection (dir) == NotConnected) { continue; }
							
							int nx = x + voxelArea.DirectionX[dir];
							int nz = z + voxelArea.DirectionZ[dir];
							
							int ni = (int)voxelArea.compactCells[nx+nz].index + s.GetConnection (dir);
							
							int r2 = (int)reg[ni];
							
							if (r != r2 && (r2 & ~BorderReg) != 0) {
								if ((r2 & BorderReg) != 0) {
									bits[k] |= BorderBit;
								} else {
									union_find_union (counter, k, r2);
								}
								//counter[r] = minRegionSize;
							}
						}
						//counter[r]++;
					}
				}
			}
			
			// Propagate bits
			for (int i=0;i<counter.Length;i++) bits[union_find_find(counter,i)] |= bits[i];
			
			for (int i=0;i<counter.Length;i++) {
				int ctr = union_find_find(counter,i);
				
				// Adjacent to border
				if ((bits[ctr] & BorderBit) != 0) counter[ctr] = -minRegionSize-2;
				
				// Not in any relevant surface
				// or it is adjacent to a border (see RelevantSurfaceCheck)
				if (anySurfaces && (bits[ctr] & RelevantSurfaceCheck) == 0) counter[ctr] = -1;
				
			}
			
			for (int i=0;i<voxelArea.compactSpanCount;i++) {
				int r = (int)reg[i];
				if (r >= nReg) {
					continue;
				}
				
				if (counter[union_find_find(counter,r)] >= -minRegionSize-1) {
					reg[i] = 0;
				}
			}
		}
	}
}

