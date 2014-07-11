
//#define ASTARDEBUG
#define ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.Voxels;

namespace Pathfinding.Voxels {
#if ASTAR_RECAST_VOXEL_DEBUG
	public static class VoxelSerializeUtility {

		public static byte[] SerializeVoxelAreaCompactData (VoxelArea v) {
			System.IO.MemoryStream stream = new System.IO.MemoryStream();
			System.IO.BinaryWriter writer = new System.IO.BinaryWriter(stream);
			
			writer.Write (v.width);
			writer.Write (v.depth);
			writer.Write (v.compactCells.Length);
			writer.Write(v.compactSpans.Length);
			writer.Write(v.areaTypes.Length);
			
			for (int i=0;i<v.compactCells.Length;i++) {
				writer.Write(v.compactCells[i].index);
				writer.Write(v.compactCells[i].count);
			}
			
			for (int i=0;i<v.compactSpans.Length;i++) {
				writer.Write(v.compactSpans[i].con);
				writer.Write(v.compactSpans[i].h);
				writer.Write(v.compactSpans[i].reg);
				writer.Write(v.compactSpans[i].y);
			}
			for (int i=0;i<v.areaTypes.Length;i++) {
				//TODO: RLE encoding
				writer.Write(v.areaTypes[i]);
			}
			writer.Close();
			return stream.ToArray();
		}
		
		public static byte[] SerializeVoxelAreaData (VoxelArea v) {
			System.IO.MemoryStream stream = new System.IO.MemoryStream();
			System.IO.BinaryWriter writer = new System.IO.BinaryWriter(stream);
			
			writer.Write (v.width);
			writer.Write (v.depth);
			writer.Write (v.linkedSpans.Length);
			
			for (int i=0;i<v.linkedSpans.Length;i++) {
				writer.Write(v.linkedSpans[i].area);
				writer.Write(v.linkedSpans[i].bottom);
				writer.Write(v.linkedSpans[i].next);
				writer.Write(v.linkedSpans[i].top);
			}
			
			//writer.Close();
			writer.Flush();
			Ionic.Zip.ZipFile zip = new Ionic.Zip.ZipFile();
			stream.Position = 0;
			zip.AddEntry ("data",stream);
			System.IO.MemoryStream stream2 = new System.IO.MemoryStream();
			zip.Save(stream2);
			byte[] bytes = stream2.ToArray();
			stream.Close();
			stream2.Close();
			return bytes;
		}
		
		public static void DeserializeVoxelAreaData (byte[] bytes, VoxelArea target) {
			
			Ionic.Zip.ZipFile zip = new Ionic.Zip.ZipFile();
			System.IO.MemoryStream stream = new System.IO.MemoryStream();
			stream.Write(bytes,0,bytes.Length);
			stream.Position = 0;
			zip = Ionic.Zip.ZipFile.Read(stream);
			System.IO.MemoryStream stream2 = new System.IO.MemoryStream();
			
			zip["data"].Extract (stream2);
			stream2.Position = 0;
			System.IO.BinaryReader reader = new System.IO.BinaryReader(stream2);
			
			int width = reader.ReadInt32();
			int depth = reader.ReadInt32();
			if (target.width != width) throw new System.ArgumentException ("target VoxelArea has a different width than the data ("+target.width + " != " + width + ")");
			if (target.depth != depth) throw new System.ArgumentException ("target VoxelArea has a different depth than the data ("+target.depth + " != " + depth + ")");
			LinkedVoxelSpan[] spans = new LinkedVoxelSpan[reader.ReadInt32()];
			
			for (int i=0;i<spans.Length;i++) {
				spans[i].area = reader.ReadInt32();
				spans[i].bottom = reader.ReadUInt32();
				spans[i].next = reader.ReadInt32();
				spans[i].top = reader.ReadUInt32();
			}
			target.linkedSpans = spans;
		}
		
		public static void DeserializeVoxelAreaCompactData (byte[] bytes, VoxelArea target) {
			
			System.IO.MemoryStream stream = new System.IO.MemoryStream(bytes);
			System.IO.BinaryReader reader = new System.IO.BinaryReader(stream);
			int width = reader.ReadInt32();
			int depth = reader.ReadInt32();
			if (target.width != width) throw new System.ArgumentException ("target VoxelArea has a different width than the data ("+target.width + " != " + width + ")");
			if (target.depth != depth) throw new System.ArgumentException ("target VoxelArea has a different depth than the data ("+target.depth + " != " + depth + ")");
			CompactVoxelCell[] cells = new CompactVoxelCell[reader.ReadInt32()];
			CompactVoxelSpan[] spans = new CompactVoxelSpan[reader.ReadInt32()];
			int[] areas = new int[reader.ReadInt32()];
			
			for (int i=0;i<cells.Length;i++) {
				cells[i].index = reader.ReadUInt32();
				cells[i].count = reader.ReadUInt32();
			}
			for (int i=0;i<spans.Length;i++) {
				spans[i].con = reader.ReadUInt32();
				spans[i].h = reader.ReadUInt32();
				spans[i].reg = reader.ReadInt32();
				spans[i].y = reader.ReadUInt16();
			}
			for (int i=0;i<areas.Length;i++) {
				areas[i] = reader.ReadInt32();
			}
			
			target.compactCells = cells;
			target.compactSpans = spans;
			target.areaTypes = areas;
		}
		
		public static void MergeVoxelAreaData (VoxelArea source, VoxelArea merge, int voxelWalkableClimb) {
			
			LinkedVoxelSpan[] spans1 = source.linkedSpans;
			
			int wd = source.width*source.depth;
			
			for (int x=0;x<wd;x++) {
				
				int i = x;
				if (spans1[i].bottom == VoxelArea.InvalidSpanValue) continue;
				
				while (i != -1) {
					merge.AddLinkedSpan(x,spans1[i].bottom,spans1[i].top,spans1[i].area,voxelWalkableClimb);
					i = spans1[i].next;
				}
			}
			
			/*for (int c=0;c<cells1.Length;c++) {
				
				int i1 = (int)cells1[c].index;
				int i2 = (int)cells2[c].index;
				int c1 = (int)i1 + (int)cells1[c].count;
				int c2 = (int)i2 + (int)cells2[c].count;
				
				CompactVoxelSpan last;
				int lastIndex;
				int lastAreaType;
				if (i1 < c1 && (i2 >= c2 || spans1[i1].y < spans2[i2].y)) {
					last = spans1[i1];
					lastAreaType = source.areaTypes[i1];
					lastIndex = i1;
					i1++;
					spanCount++;
				} else if (i2 < c2) {
					last = spans2[i2];
					lastAreaType = merge.areaTypes[i2];
					lastIndex = i2;
					i2++;
					spanCount++;
				} else {
					continue;
				}
				
				while (i1 < c1 || i2 < c2) {
					
					CompactVoxelSpan span;
					int areaType;
					int spanIndex;
					
					Debug.Log (i1 + " " + i2 + " " + c1 + " " + c2);
					if (i1 < c1 && (i2 >= c2 || spans1[i1].y < spans2[i2].y)) {
						span = spans1[i1];
						spanIndex = i1;
						areaType = source.areaTypes[i1];
						i1++;
					} else if (i2 < c2) {
						span = spans2[i2];
						areaType = merge.areaTypes[i2];
						spanIndex = i2;
						i2++;
					} else {
						throw new System.Exception ("This should not happen");
					}
					
					Debug.Log (span.y + " " + (last.y+last.h));
					if (span.y > last.y+last.h) {
						last = span;
						spanCount++;
						continue;
					} else {
						last.h = System.Math.Max(last.h,span.y+span.h - last.y);
					}
				}
			}
			
			CompactVoxelSpan[] spansResult = new CompactVoxelSpan[spanCount];
			
			Debug.Log (spans1.Length + " : " + spans2.Length + " -> " + spanCount);
			
			
					int area;
					//1 is flagMergeDistance, when a walkable flag is favored before an unwalkable one
					if (Mathfx.Abs ((int)(span.y+span.h) - (int)(last.y+span.h)) <= voxelWalkableClimb) {
						area = Mathfx.Max (lastAreaType,areaType);
					}
				}
							
					
			}*/
		}
		
	}
#endif

	/** \astarpro */
	public class VoxelArea {
		
		public const uint MaxHeight = 65536;
		public const int MaxHeightInt = 65536;
		
		/** Constant for default LinkedVoxelSpan top and bottom values.
		 * It is important with the U since ~0 != ~0U
		 * This can be used to check if a LinkedVoxelSpan is valid and not just the default span
		 */
		public const uint InvalidSpanValue = ~0U;
		
		/** Initial estimate on the average number of spans (layers) in the voxel representation. Should be greater or equal to 1 */
		public const float AvgSpanLayerCountEstimate = 8;
		
		/** The width of the field along the x-axis. [Limit: >= 0] [Units: vx] */
		public readonly int width = 0;
		
		/** The depth of the field along the z-axis. [Limit: >= 0] [Units: vx] */
		public readonly int depth = 0;
		
#if !ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
		public VoxelCell[] cells;
#endif
		
		public CompactVoxelSpan[] compactSpans;
		public CompactVoxelCell[] compactCells;
		public int compactSpanCount;
		
		public ushort[] tmpUShortArr;
		
		public int[] areaTypes;
		
		public ushort[] dist;
		public ushort maxDistance;
		
		public int maxRegions = 0;
		
		public int[] DirectionX;
		public int[] DirectionZ;
		
		public Vector3[] VectorDirection;
		
		public void Reset () {
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
			ResetLinkedVoxelSpans();
#else
			for (int i=0;i<cells.Length;i++) cells[i].firstSpan = null;
#endif
			
			for (int i=0;i<compactCells.Length;i++) {
				compactCells[i].count = 0;
				compactCells[i].index = 0;
			}
		}
		
		private void ResetLinkedVoxelSpans () {
			int len = linkedSpans.Length;
			linkedSpanCount = width*depth;
			LinkedVoxelSpan df = new LinkedVoxelSpan(InvalidSpanValue,InvalidSpanValue,-1,-1);
			for (int i=0;i<len;) {
				// 16x unrolling, actually improves performance
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
				linkedSpans[i] = df;i++;
			}
			removedStackCount = 0;
		}
		
		public VoxelArea (int width, int depth) {
			this.width = width;
			this.depth = depth;
			
			int wd = width*depth;
			compactCells = new CompactVoxelCell[wd];
			
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
			// & ~0xF ensures it is a multiple of 16. Required for unrolling
			linkedSpans = new LinkedVoxelSpan[((int)(wd*AvgSpanLayerCountEstimate) + 15)& ~0xF];
			ResetLinkedVoxelSpans();
#else
			cells = new VoxelCell[wd];
#endif
			
			DirectionX = new int[4] {-1,0,1,0};
			DirectionZ = new int[4] {0,width,0,-width};
			
			VectorDirection = new Vector3[4] {Vector3.left, Vector3.forward,Vector3.right, Vector3.back};
		}
		
		public int GetSpanCountAll () {
			int count = 0;
			
			int wd = width*depth;
			
			for (int x=0;x<wd;x++) {
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
				for (int s = x; s != -1 && linkedSpans[s].bottom != InvalidSpanValue; s = linkedSpans[s].next) {
					count++;
				}
#else
				for (VoxelSpan s = cells[x].firstSpan; s != null; s = s.next) {
					count++;
				}
#endif
			}
			
			return count;
		}
		
		public int GetSpanCount () {
			int count = 0;
			
			int wd = width*depth;
			
			for (int x=0;x<wd;x++) {
#if ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
				for (int s = x; s != -1 && linkedSpans[s].bottom != InvalidSpanValue; s = linkedSpans[s].next) {
					if (linkedSpans[s].area != 0) {
						count++;
					}
				}
#else
				for (VoxelSpan s = cells[x].firstSpan; s != null; s = s.next) {
					if (s.area != 0) {
						count++;
					}
				}
#endif
			}
			return count;
		}
		
		
		private int linkedSpanCount;
		public LinkedVoxelSpan[] linkedSpans;
		
		private int[] removedStack = new int[128];
		private int removedStackCount = 0;
		
		public void AddLinkedSpan (int index, uint bottom, uint top, int area, int voxelWalkableClimb) {
#if !ASTAR_RECAST_ARRAY_BASED_LINKED_LIST
			cells[index].AddSpan(bottom,top,area,voxelWalkableClimb);
#else
			
			/* Check if the span is valid, otherwise we can replace it with a new (valid) span */
			if (linkedSpans[index].bottom == InvalidSpanValue) {
				linkedSpans[index] = new LinkedVoxelSpan(bottom,top,area);
				return;
			}
			
			
			int prev = -1;
			int oindex = index;
			
			while (index != -1) {
				if (linkedSpans[index].bottom > top) {
					break;
					
				} else if (linkedSpans[index].top < bottom) {
					prev = index;
					index = linkedSpans[index].next;
				} else {
					if (linkedSpans[index].bottom < bottom) {
						bottom = linkedSpans[index].bottom;
					}
					if (linkedSpans[index].top > top) {
						top = linkedSpans[index].top;
					}
					
					//1 is flagMergeDistance, when a walkable flag is favored before an unwalkable one
					if (AstarMath.Abs ((int)top - (int)linkedSpans[index].top) <= voxelWalkableClimb) {
						area = AstarMath.Max (area,linkedSpans[index].area);
					}
					
					int next = linkedSpans[index].next;
					if (prev != -1) {
						linkedSpans[prev].next = next;
						
						if (removedStackCount == removedStack.Length) {
							int[] st2 = new int[removedStackCount*4];
							System.Buffer.BlockCopy(removedStack,0,st2,0,removedStackCount*sizeof(int));
							removedStack = st2;
						}
						removedStack[removedStackCount] = index;
						removedStackCount++;
						
						index = next;
					} else if (next != -1) {
						linkedSpans[oindex] = linkedSpans[next];
						
						if (removedStackCount == removedStack.Length) {
							int[] st2 = new int[removedStackCount*4];
							System.Buffer.BlockCopy(removedStack,0,st2,0,removedStackCount*sizeof(int));
							removedStack = st2;
						}
						removedStack[removedStackCount] = next;
						removedStackCount++;
						
						index = linkedSpans[oindex].next;
					} else {
						linkedSpans[oindex] = new LinkedVoxelSpan(bottom,top,area);
						return;
					}
				}
			}
			
			if (linkedSpanCount >= linkedSpans.Length) {
				LinkedVoxelSpan[] tmp = linkedSpans;
				int count = linkedSpanCount;
				int popped = removedStackCount;
				linkedSpans = new LinkedVoxelSpan[linkedSpans.Length*2];
				ResetLinkedVoxelSpans();
				linkedSpanCount = count;
				removedStackCount = popped;
				for (int i=0;i<linkedSpanCount;i++) {
					linkedSpans[i] = tmp[i];
				}
				Debug.Log ("Layer estimate too low, doubling size of buffer.\nThis message is harmless.");
			}
			
			int nextIndex;
			if (removedStackCount > 0) {
				removedStackCount--;
				nextIndex = removedStack[removedStackCount];
			} else {
				nextIndex = linkedSpanCount;
				linkedSpanCount++;
			}
			
			if (prev != -1) {
				
				//span.next = prev.next;
				//prev.next = span;
				
				linkedSpans[nextIndex] = new LinkedVoxelSpan(bottom,top,area,linkedSpans[prev].next);
				linkedSpans[prev].next = nextIndex;
			} else {
				//span.next = firstSpan;
				//firstSpan = span;
				
				linkedSpans[nextIndex] = linkedSpans[oindex];
				linkedSpans[oindex] = new LinkedVoxelSpan(bottom,top,area,nextIndex);
			}
#endif	
		}
	}
	
	public struct LinkedVoxelSpan {
		public uint bottom;
		public uint top;
		
		public int next;
		
		/*Area
		0 is an unwalkable span (triangle face down)
		1 is a walkable span (triangle face up)
		*/
		public int area;
		
		public LinkedVoxelSpan (uint bottom, uint top, int area) {
			this.bottom = bottom; this.top = top; this.area = area; this.next = -1;
		}
		
		public LinkedVoxelSpan (uint bottom, uint top, int area, int next) {
			this.bottom = bottom; this.top = top; this.area = area; this.next = next;
		}
	}
	
	/** Represents a custom mesh.
	 * The vertices will be multiplied with the matrix when rasterizing it to voxels.
	 * The vertices and triangles array may be used in multiple instances, it is not changed when voxelizing.
	 * 
	 * \see SceneMesh
	 * 
	 * \astarpro
	 */
	public struct ExtraMesh {
		/** Source of the mesh.
		 * May be null if the source was not a mesh filter
		 */
		public MeshFilter original;
		
		public int area;
		public Vector3[] vertices;
		public int[] triangles;
		
		/** World bounds of the mesh. Assumed to already be multiplied with the matrix */
		public Bounds bounds;
		
		public Matrix4x4 matrix;
		
		public ExtraMesh (Vector3[] v, int[] t, Bounds b) {
			matrix = Matrix4x4.identity;
			vertices = v;
			triangles = t;
			bounds = b;
			original = null;
			area = 0;
		}
		
		public ExtraMesh (Vector3[] v, int[] t, Bounds b, Matrix4x4 matrix) {
			this.matrix = matrix;
			vertices = v;
			triangles = t;
			bounds = b;
			original = null;
			area = 0;
		}
		
		/** Recalculate the bounds based on vertices and matrix */
		public void RecalculateBounds () {
			Bounds b = new Bounds(matrix.MultiplyPoint3x4(vertices[0]),Vector3.zero);
			for (int i=1;i<vertices.Length;i++) {
				b.Encapsulate (matrix.MultiplyPoint3x4(vertices[i]));
			}
			//Assigned here to avoid changing bounds if vertices would happen to be null
			bounds = b;
		}
	}
	
	/** VoxelContourSet used for recast graphs.
	 * \astarpro
	 */
	public class VoxelContourSet {
		public List<VoxelContour> conts;		// Pointer to all contours.
		//public int nconts;				// Number of contours.
		public Bounds bounds;	// Bounding box of the heightfield.
		//public float cellSize, cellHeight;			// Cell size and height.
	}
	
	/** VoxelContour used for recast graphs.
	 * \astarpro
	 */
	public struct VoxelContour {
		public int nverts;
		public int[] verts;		// Vertex coordinates, each vertex contains 4 components.
		public int[] rverts;		// Raw vertex coordinates, each vertex contains 4 components.
		
		public int reg;			// Region ID of the contour.
		public int area;			// Area ID of the contour.
	}
	
	/** VoxelMesh used for recast graphs.
	 * \astarpro
	 */
	public struct VoxelMesh {
		public Int3[] verts;
		public int[] tris;
	}
	
	/** VoxelCell used for recast graphs.
	 * \astarpro
	 */
	public struct VoxelCell {
		
		public VoxelSpan firstSpan;
		
		//public System.Object lockObject;
		
		public void AddSpan (uint bottom, uint top, int area, int voxelWalkableClimb) {
			VoxelSpan span = new VoxelSpan (bottom,top,area);
				
			if (firstSpan == null) {
				firstSpan = span;
				return;
			}
			
			VoxelSpan prev = null;
			
			VoxelSpan cSpan = firstSpan;
			
			//for (VoxelSpan cSpan = firstSpan; cSpan != null; cSpan = cSpan.next) {
			while (cSpan != null) {
				if (cSpan.bottom > span.top) {
					break;
					
				} else if (cSpan.top < span.bottom) {
					prev = cSpan;
					cSpan = cSpan.next;
				} else {
					if (cSpan.bottom < bottom) {
						span.bottom = cSpan.bottom;
					}
					if (cSpan.top > top) {
						span.top = cSpan.top;
					}
					
					//1 is flagMergeDistance, when a walkable flag is favored before an unwalkable one
					if (AstarMath.Abs ((int)span.top - (int)cSpan.top) <= voxelWalkableClimb) {
						span.area = AstarMath.Max (span.area,cSpan.area);
					}
					
					VoxelSpan next = cSpan.next;
					if (prev != null) {
						prev.next = next;
					} else {
						firstSpan = next;
					}
					cSpan = next;
					
					/*cSpan.bottom = cSpan.bottom < bottom ? cSpan.bottom : bottom;
					cSpan.top = cSpan.top > top ? cSpan.top : top;
					
					if (cSpan.bottom < span.bottom) {
						span.bottom = cSpan.bottom;
					}
					if (cSpan.top > span.top) {
						span.top = cSpan.top;
					}
					
					span.area = Mathfx.Min (span.area,cSpan.area);
					VoxelSpan next = cSpan.next;
					
					if (prev != null) {
						prev.next = next;
					} else {
						firstSpan = next;
					}
					cSpan = next;*/
				}
			}
			
			if (prev != null) {
				span.next = prev.next;
				prev.next = span;
			} else {
				span.next = firstSpan;
				firstSpan = span;
			}
		}
		
		/*public void FilterWalkable (uint walkableHeight) {
			VoxelSpan prev = null;
			
			for (VoxelSpan cSpan = firstSpan; cSpan != null; cSpan = cSpan.next) {
				if (cSpan.area == 1) {
					if (prev != null) {
						prev.next = cSpan.next;
					} else {
						firstSpan = cSpan.next;
					}
				} else {
					if (cSpan.next != null) {
						cSpan.top = cSpan.next.bottom;
					} else {
						cSpan.top = VoxelArea.MaxHeight;
					}
					
					uint val = cSpan.top-cSpan.bottom;
					if (cSpan.top < cSpan.bottom) {
						Debug.Log ((cSpan.top-cSpan.bottom));
					}
					
					if (val < walkableHeight) {
						if (prev != null) {
							prev.next = cSpan.next;
						} else {
							firstSpan = cSpan.next;
						}
					} else {
						prev = cSpan;
						continue;
					}
				}
				
			}
		}*/
	}
	
	/** CompactVoxelCell used for recast graphs.
	 * \astarpro
	 */
	public struct CompactVoxelCell {
		public uint index;
		public uint count;
		
		public CompactVoxelCell (uint i, uint c) {
			index = i;
			count = c;
		}
	}
	
	/** CompactVoxelSpan used for recast graphs.
	 * \astarpro
	 */
	public struct CompactVoxelSpan {
		public ushort y;
		public uint con;
		public uint h;
		public int reg;
		
		public CompactVoxelSpan (ushort bottom, uint height) {
			con = 24;
			y = bottom;
			h = height;
			reg = 0;
		}
		
		/*public CompactVoxelSpan (uint bottom, uint top) {
			con = 24;
			y = (ushort)bottom;
			h = top-bottom;
			area = 1;
		}*/
		
		public void SetConnection (int dir, uint value) {
			int shift = dir*6;
			con  = (uint) ( (con & ~(0x3f << shift)) | ((value & 0x3f) << shift) );
		}
	
		public int GetConnection (int dir) {
	        return ((int)con >> dir*6) & 0x3f;
		}
		
		//const unsigned int shift = (unsigned int)dir*6;
	      //  s.con = (con & ~(63 << shift)) | (((uint)value & 63) << shift);
	}
	
	/** VoxelSpan used for recast graphs.
	 * \astarpro
	 */
	public class VoxelSpan {
		public uint bottom;
		public uint top;
		
		public VoxelSpan next;
		
		/*Area
		0 is an unwalkable span (triangle face down)
		1 is a walkable span (triangle face up)
		*/
		public int area;
		//public VoxelSpan () {
		//}
		
		public VoxelSpan (uint b, uint t,int area) {
			bottom = b;
			top = t;
			this.area = area;
		}
	}
}