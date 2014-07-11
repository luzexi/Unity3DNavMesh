using System;
using System.Collections.Generic;
using UnityEngine;
using Pathfinding;
using Pathfinding.ClipperLib;
using Pathfinding.Poly2Tri;

namespace Pathfinding.Util
{
	public class TileHandler
	{
		RecastGraph _graph;
		List<TileType> tileTypes = new List<TileType>();
		
		Clipper clipper;
		int[] cached_int_array = new int[32];
		Dictionary<Int3, int> cached_Int3_int_dict = new Dictionary<Int3, int>();
		Dictionary<Int2, int> cached_Int2_int_dict = new Dictionary<Int2, int>();
		
		TileType[] activeTileTypes;
		int[] activeTileRotations;
		int[] activeTileOffsets;
		bool[] reloadedInBatch;
		
		bool isBatching = false;
		
		public RecastGraph graph {
			get {
				return _graph;
			}
		}
		
		public TileHandler (RecastGraph graph) {
			if (graph == null) throw new System.ArgumentNullException ("'graph' cannot be null");
			if (graph.GetTiles() == null) throw new System.ArgumentException ("graph has no tiles. Please scan the graph before creating a TileHandler");
			activeTileTypes = new TileType[graph.tileXCount*graph.tileZCount];
			activeTileRotations = new int[activeTileTypes.Length];
			activeTileOffsets = new int[activeTileTypes.Length];
			reloadedInBatch = new bool[activeTileTypes.Length];
			
			this._graph = graph;
		}
		
		public int GetActiveRotation ( Int2 p )
		{
			return activeTileRotations[p.x + p.y*_graph.tileXCount];
		}
		
		public TileType GetTileType (int index) {
			return tileTypes[index];
		}
		
		public int GetTileTypeCount () {
			return tileTypes.Count;
		}
		
		public class TileType {
			Int3[] verts;
			int[] tris;
			Int3 offset;
			int lastYOffset;
			int lastRotation;
			int width;
			int depth;
			
			public int Width {
				get {
					return width;
				}
			}
			
			public int Depth {
				get {
					return depth;
				}
			}
			
			/** Matrices for rotation.
			 * Each group of 4 elements is a 2x2 matrix.
			 * The XZ position is multiplied by this.
			 * So
			 * \code
			 * //A rotation by 90 degrees clockwise, second matrix in the array
			 * (5,2) * ((0, 1), (-1, 0)) = (2,-5)
			 * \endcode
			 */
			private static readonly int[] Rotations = {
				 1, 0, //Identity matrix
				 0, 1,
				
				 0, 1,
				-1, 0,
				
				-1, 0,
				 0,-1,
				
				 0,-1,
				 1, 0
			};
			
			public TileType (Int3[] sourceVerts, int[] sourceTris, Int3 tileSize, Int3 centerOffset, int width=1, int depth=1) {
				if (sourceVerts == null) throw new System.ArgumentNullException ("sourceVerts");
				if (sourceTris == null) throw new System.ArgumentNullException ("sourceTris");
				
				tris = new int[sourceTris.Length];
				for (int i=0;i<tris.Length;i++) tris[i] = sourceTris[i];
				
				verts = new Int3[sourceVerts.Length];
				
				for (int i=0;i<sourceVerts.Length;i++) {
					verts[i] = sourceVerts[i] + centerOffset;
				}
				
				offset = tileSize/2;
				offset.x *= width;
				offset.z *= depth;
				offset.y = 0;
				
				for (int i=0;i<sourceVerts.Length;i++) {
					verts[i] = verts[i] + offset;
				}
				
				lastRotation = 0;
				lastYOffset = 0;
				
				this.width = width;
				this.depth = depth;
			}
			
			/** Create a new TileType.
			 * First all vertices of the source mesh are offseted by the \a centerOffset.
			 * The source mesh is assumed to be centered (after offsetting). Corners of the tile should be at tileSize*0.5 along all axes. When width or depth is not 1,
			 * the tileSize param should not change, but corners of the tile are assumed to lie further out.
			 *
			 * \param width the number of base tiles this tile type occupies on the x-axis
			 * \param depth the number of base tiles this tile type occupies on the z-axis
			 * \param tileSize Size of a single tile, the y-coordinate will be ignored.
			 */
			public TileType (Mesh source, Int3 tileSize, Int3 centerOffset, int width=1, int depth=1) {
				if (source == null) throw new System.ArgumentNullException ("source");
				
				Vector3[] vectorVerts = source.vertices;
				tris = source.triangles;
				verts = new Int3[vectorVerts.Length];
				
				for (int i=0;i<vectorVerts.Length;i++) {
					verts[i] = (Int3)vectorVerts[i] + centerOffset;
				}
				
				offset = tileSize/2;
				offset.x *= width;
				offset.z *= depth;
				offset.y = 0;
				
				for (int i=0;i<vectorVerts.Length;i++) {
					verts[i] = verts[i] + offset;
				}
				
				lastRotation = 0;
				lastYOffset = 0;
				
				this.width = width;
				this.depth = depth;
			}
			
			/** Load a tile, result given by the vert and tris array.
			 * \warning For performance and memory reasons, the returned arrays are internal arrays, so they must not be modified in any way or
			 * subsequent calls to Load may give corrupt output. The contents of the verts array is only valid until the next call to Load since
			 * different rotations and y offsets can be applied.
			 * If you need persistent arrays, please copy the returned ones. 
			 */
			public void Load (out Int3[] verts, out int[] tris, int rotation, int yoffset) {
				
				/*System.IO.BinaryReader reader = new System.IO.BinaryReader (new System.IO.MemoryStream(source.bytes));
				verts = new Int3[reader.ReadInt32()];
				tris = new int[reader.ReadInt32()];
				for (int i=0;i<verts.Length;i++) verts[i] = new Int3(reader.ReadInt32(), reader.ReadInt32(), reader.ReadInt32());
				for (int i=0;i<tris.Length;i++) tris[i] = reader.ReadInt32();
				for (int i=0;i<tris.Length;i+=3) {
					if (!Polygon.IsClockwise(verts[tris[i]],verts[tris[i+1]],verts[tris[i+2]])) {
						Debug.LogWarning("Non clockwise triangle");
						int tmp = tris[i];
						tris[i] = tris[i+2];
						tris[i+2] = tmp;
					}
				}*/
				
				//Make sure it is a number 0 <= x < 4
				rotation = ((rotation % 4) + 4) % 4;
				
				//Figure out relative rotation (relative to previous rotation that is, since that is still applied to the verts array)
				int tmp = rotation;
				rotation = (rotation - (lastRotation % 4) + 4) % 4;
				lastRotation = tmp;
				
				verts = this.verts;
				
				int relYOffset = yoffset - lastYOffset;
				lastYOffset = yoffset;
				
				if (rotation != 0 || relYOffset != 0) {
					for (int i=0;i<verts.Length;i++) {
						Int3 op = verts[i] - offset;
						Int3 p = op;
						p.y += relYOffset;
						p.x = op.x * Rotations[rotation*4 + 0] + op.z * Rotations[rotation*4 + 1];
						p.z = op.x * Rotations[rotation*4 + 2] + op.z * Rotations[rotation*4 + 3];
						verts[i] = p + offset;
					}
				}
				
				tris = this.tris;
			}
		}
		
		/** Register that a tile can be loaded from \a source.
		 * 
		 * 
		 * 
		 * \param centerOffset Assumes that the mesh has its pivot point at the center of the tile.
		 * If it has not, you can supply a non-zero \a centerOffset to offset all vertices.
		 * 
		 * \param width width of the tile. In base tiles, not world units.
		 * \param depth depth of the tile. In base tiles, not world units.
		 * \param source Source mesh, must be readable.
		 * 
		 * \returns Identifier for loading that tile type
		 */
		public TileType RegisterTileType (Mesh source, Int3 centerOffset, int width = 1, int depth=1) {
			TileType tp = new TileType(source, new Int3(graph.tileSizeX,1,graph.tileSizeZ)*(Int3.Precision*graph.cellSize), centerOffset, width, depth);
			tileTypes.Add(tp);
			return tp;
		}
		
		public void CreateTileTypesFromGraph () {
			RecastGraph.NavmeshTile[] tiles = graph.GetTiles();
			if ( tiles == null || tiles.Length != graph.tileXCount*graph.tileZCount ) {
				throw new System.InvalidOperationException ("Graph tiles are invalid (either null or number of tiles is not equal to width*depth of the graph");
			}
			
			for (int z=0;z<graph.tileZCount;z++) {
				for (int x=0;x<graph.tileXCount;x++) {
					RecastGraph.NavmeshTile tile = tiles[x + z*graph.tileXCount];
					
					Bounds b = graph.GetTileBounds(x,z);
					Int3 min = (Int3)b.min;
					Int3 size = new Int3(graph.tileSizeX,1,graph.tileSizeZ)*(Int3.Precision*graph.cellSize);
					min += new Int3(size.x*tile.w/2,0,size.z*tile.d/2);
					min = -min;
					
					TileType tp = new TileType(tile.verts, tile.tris, size, min, tile.w, tile.d);
					tileTypes.Add(tp);
					
					int index = x + z*graph.tileXCount;
					activeTileTypes[index] = tp;
					activeTileRotations[index] = 0;
					activeTileOffsets[index] = 0;
				}
			}
		}
		
		/** Start batch loading.
		 * \returns True if batching wasn't started yet, and thus EndBatchLoad should be called,
		 * False if batching was already started by some other part of the code and you should not call EndBatchLoad
		 */
		public bool StartBatchLoad () {
			//if (isBatching) throw new System.Exception ("Starting batching when batching has already been started");
			if ( isBatching ) return false;
			
			isBatching = true;
			
			AstarPath.active.AddWorkItem(new AstarPath.AstarWorkItem (delegate (bool force) {
				graph.StartBatchTileUpdate ();
				return true;
			}));
			
			return true;
		}
		
		public void EndBatchLoad () {
			if (!isBatching) throw new System.Exception ("Ending batching when batching has not been started");
			
			for (int i=0;i<reloadedInBatch.Length;i++) reloadedInBatch[i] = false;
			isBatching = false;
			
			AstarPath.active.AddWorkItem(new AstarPath.AstarWorkItem (delegate (bool force) {
				
				graph.EndBatchTileUpdate ();
			
				//Flood fill everything to make sure graph areas are still valid
				//This tends to take more than 50% of the calculation time
				//AstarPath.active.FloodFill();
				
				return true;
			}));
		}
		
		const int CUT_ALL = 0;
		const int CUT_DUAL = CUT_ALL+1;
		const int CUT_BREAK = CUT_DUAL+1;
		
		public enum CutMode {
			CutAll = 1,
			CutDual = 2,
			CutExtra = 4
		}
		
		void CutPoly (Int3[] verts, int[] tris, ref Int3[] outVertsArr, ref int[] outTrisArr, out int outVCount, out int outTCount, Int3[] extraShape, Int3 cuttingOffset, Bounds realBounds, CutMode mode = CutMode.CutAll | CutMode.CutDual, int perturbate = 0) {
			
			// Nothing to do here
			
			if (verts.Length == 0 || tris.Length == 0) {
				
				outVCount = 0;
				outTCount = 0;
				outTrisArr = new int[0];
				outVertsArr = new Int3[0];
				
				return;
			}
			
			/*Profile watch = new Profile("Total");
			Profile watch2 = new Profile("Clipping");
			Profile watch3 = new Profile("Triangulation");
			Profile watch4 = new Profile("Overhead");
			Profile watch5 = new Profile("Coordinate Compression");
			Profile watch6 = new Profile("Init");
			Profile watch8 = new Profile("Navmesh Cut Components");
			
			watch.Start();
			watch6.Start();*/
			
			List<IntPoint> extraClipShape = null;
			
			// Do not cut with extra shape if there is no extra shape
			if (extraShape == null && (mode & CutMode.CutExtra) != 0) {
				throw new System.Exception ("extraShape is null and the CutMode specifies that it should be used. Cannot use null shape.");
			}
			
			if ((mode & CutMode.CutExtra) != 0) {
				extraClipShape = new List<IntPoint>(extraShape.Length);
				for (int i = 0; i < extraShape.Length; i++ ) {
					extraClipShape.Add (new IntPoint (extraShape[i].x + cuttingOffset.x, extraShape[i].z + cuttingOffset.z));
				}
			}
			
			List<IntPoint> poly = new List<IntPoint> (5);
			
			Dictionary<TriangulationPoint, int> point2Index = new Dictionary<TriangulationPoint, int>();
			
			List<Poly2Tri.PolygonPoint> polypoints = new List<Poly2Tri.PolygonPoint>();
			
			IntRect bounds = new IntRect(verts[0].x,verts[0].z,verts[0].x,verts[0].z);
			
			for (int i=0;i<verts.Length;i++) {
				//clip.Add (new IntPoint(verts[i].x,verts[i].z));
				bounds = bounds.ExpandToContain(verts[i].x,verts[i].z);
			}
			
#if ASTARDEBUG
			for (int i=0;i<cutPoly.Length;i++) {
				Debug.DrawLine ((Vector3)IntPoint2Int3(clip[i]),(Vector3)IntPoint2Int3(clip[(i+1) % cutPoly.Length]), Color.blue);
			}
#endif
			
			List<Int3> outverts = Pathfinding.Util.ListPool<Int3>.Claim(verts.Length*2);
			List<int> outtris = Pathfinding.Util.ListPool<int>.Claim(tris.Length);
			//List<Pathfinding.ClipperLib.ExPolygon> sol = new List<Pathfinding.ClipperLib.Pa>();
			Pathfinding.ClipperLib.PolyTree sol = new Pathfinding.ClipperLib.PolyTree();
			
			List<List<IntPoint>> sol2 = new List<List<IntPoint>>();
			
			Stack<Poly2Tri.Polygon> polyCache = new Stack<Poly2Tri.Polygon>();
			
			//Lazy initialization
			if (clipper == null) {
				clipper = new Clipper();
			}
			
			clipper.ReverseSolution = true;
			
			//watch8.Start();
			
			// Cutting variables
			List<NavmeshCut> navmeshCuts;
			if ( mode == CutMode.CutExtra ) {
				// Not needed when only cutting extra
				navmeshCuts = Pathfinding.Util.ListPool<NavmeshCut>.Claim ();
			} else {
				navmeshCuts = NavmeshCut.GetAllInRange(realBounds);//GameObject.FindObjectsOfType(typeof(NavmeshCut)) as NavmeshCut[];
			}
			
			List<int> tmpIntersectingCuts = Pathfinding.Util.ListPool<int>.Claim();
			//Don't want to write a completely new IntBounds class, so an IntRect and a range for Y axis will have to suffice.
			List<IntRect> cutBounds = Pathfinding.Util.ListPool<IntRect>.Claim();
			List<Int2> cutBoundsY = Pathfinding.Util.ListPool<Int2>.Claim();
			List<List<IntPoint>> cutVertices = new List<List<IntPoint>>();
			List<bool> cutIsDual = Pathfinding.Util.ListPool<bool>.Claim();
			List<bool> cutsAddedGeom = Pathfinding.Util.ListPool<bool>.Claim();

			if (perturbate > 10) {
				Debug.LogError ("Too many perturbations aborting : " + mode);
				Debug.Break();
				outVCount = verts.Length;
				outTCount = tris.Length;
				outTrisArr = tris;
				outVertsArr = verts;
				return;
			}
			
			System.Random rnd = null;
			if (perturbate > 0) {
				rnd = new System.Random();
			}
			
			for (int i=0;i<navmeshCuts.Count;i++) {
				Bounds b = navmeshCuts[i].GetBounds();
				Int3 mn = (Int3)b.min + cuttingOffset;
				Int3 mx = (Int3)b.max + cuttingOffset;
				IntRect b3 = new IntRect(mn.x,mn.z,mx.x,mx.z);
				
				if (IntRect.Intersects(b3,bounds)) {
					
					// Generate random perturbation for this obstacle if required
					Int2 perturbation = new Int2(0,0);
					if (perturbate > 0) {
						// Create a perturbation vector, choose a point with coordinates in the set [-5,5] \ 0
						// makes sure the coordinates are not zero
						
						perturbation.x = (rnd.Next() % 6*perturbate) - 3*perturbate;
						if (perturbation.x >= 0) perturbation.x++;
						
						perturbation.y = (rnd.Next() % 6*perturbate) - 3*perturbate;
						if (perturbation.y >= 0) perturbation.y++;
					}
					
					int origCutCount = cutVertices.Count;
					navmeshCuts[i].GetContour(cutVertices);
					
					for (int j=origCutCount;j<cutVertices.Count;j++) {
						
						List<IntPoint> cut = cutVertices[j];
						if (cut.Count == 0) {
							Debug.LogError ("Zero Length Contour");
							cutBounds.Add (new IntRect());
							cutBoundsY.Add (new Int2(0,0));
							continue;
						}
						
						IntRect b2 = new IntRect((int)cut[0].X+cuttingOffset.x,(int)cut[0].Y+cuttingOffset.y,(int)cut[0].X+cuttingOffset.x,(int)cut[0].Y+cuttingOffset.y);
						
						for (int q=0;q<cut.Count;q++) {
							IntPoint p = cut[q];
							p.X += cuttingOffset.x;
							p.Y += cuttingOffset.z;
							if (perturbate > 0) {
								p.X += perturbation.x;
								p.Y += perturbation.y;
							}
							
							cut[q] = p;
							b2 = b2.ExpandToContain((int)p.X,(int)p.Y);
						}
						
						cutBoundsY.Add (new Int2(mn.y,mx.y));
						cutBounds.Add (b2);
						cutIsDual.Add (navmeshCuts[i].isDual);
						cutsAddedGeom.Add (navmeshCuts[i].cutsAddedGeom);
					}
				}
			}

			List<NavmeshAdd> navmeshAdds = NavmeshAdd.GetAllInRange (realBounds);

			//watch8.Stop();
			
			//watch6.Stop();

			Int3[] cverts = verts;
			int[] ctris = tris;
			int addIndex = -1;
			int tri = -3;

			//for (int tri=0;tri<tris.Length;tri+=3) {

			Int3[] clipIn = null;
			Int3[] clipOut = null;
			Int3 clipExtents = Int3.zero;

			if ( navmeshAdds.Count > 0 ) {
				clipIn = new Int3[7];
				clipOut = new Int3[7];
				clipExtents = (Int3)realBounds.extents;
			}


			while ( true ) {

				tri += 3;
				while ( tri >= ctris.Length ) {
					addIndex++;
					tri = 0;
					
					if ( addIndex >= navmeshAdds.Count ) {
						cverts = null;
						break;
					}
					
					// This array must not be modified
					if ( cverts == verts ) cverts = null;
					
					navmeshAdds[addIndex].GetMesh ( cuttingOffset, ref cverts, out ctris );
				}
				
				// Inner loop above decided that we should break the while(true) loop
				if ( cverts == null ) break;

				Int3 tp1 = cverts[ctris[tri+0]];
				Int3 tp2 = cverts[ctris[tri+1]];
				Int3 tp3 = cverts[ctris[tri+2]];

				//Debug.DrawLine ((Vector3)(tp1 - cuttingOffset), (Vector3)(tp2 - cuttingOffset), Color.red);
				//Debug.DrawLine ((Vector3)(tp2 - cuttingOffset), (Vector3)(tp3 - cuttingOffset), Color.red);
				//Debug.DrawLine ((Vector3)(tp3 - cuttingOffset), (Vector3)(tp1 - cuttingOffset), Color.red);

				//Debug.DrawLine ((Vector3)(tp1), (Vector3)(tp2), Color.red);
				//Debug.DrawLine ((Vector3)(tp2), (Vector3)(tp3), Color.red);
				//Debug.DrawLine ((Vector3)(tp3), (Vector3)(tp1), Color.red);
				//Debug.Break ();

				IntRect triBounds = new IntRect(tp1.x,tp1.z,tp1.x,tp1.z);
				triBounds = triBounds.ExpandToContain(tp2.x,tp2.z);
				triBounds = triBounds.ExpandToContain(tp3.x,tp3.z);
				
				//Upper and lower bound on the Y-axis, the above bounds does not have Y axis information
				int tpYMin = System.Math.Min(tp1.y, System.Math.Min(tp2.y, tp3.y));
				int tpYMax = System.Math.Max(tp1.y, System.Math.Max(tp2.y, tp3.y));
				
				tmpIntersectingCuts.Clear();
				bool hasDual = false;
				
				for (int i=0;i<cutVertices.Count;i++) {
					int ymin = cutBoundsY[i].x;
					int ymax = cutBoundsY[i].y;
					
					if (IntRect.Intersects(triBounds, cutBounds[i]) && !(ymax < tpYMin || ymin > tpYMax) && (cutsAddedGeom[i] || addIndex == -1) ) {
						Int3 p1 = tp1;
						p1.y = ymin;
						Int3 p2 = tp1;
						p2.y = ymax;
						
#if ASTARDEBUG
						Debug.DrawLine ((Vector3)p1, (Vector3)p2, Color.magenta);
						Debug.DrawRay ((Vector3)tp1, Vector3.right*0.2f, Color.green);
						Debug.DrawRay ((Vector3)tp1, Vector3.left*0.2f, Color.green);
						Debug.DrawRay ((Vector3)tp1, Vector3.forward*0.2f, Color.green);
						Debug.DrawRay ((Vector3)tp1, Vector3.back*0.2f, Color.green);
						Debug.DrawLine ((Vector3)tp1, (Vector3)tp2, Color.green);
						Debug.DrawLine ((Vector3)tp2, (Vector3)tp3, Color.green);
						Debug.DrawLine ((Vector3)tp3, (Vector3)tp1, Color.green);
#endif
						
						tmpIntersectingCuts.Add (i);
						hasDual |= cutIsDual[i];
					}
				}
				
				//if (!IntRect.Intersects (triBounds, bounds)) {
				if (tmpIntersectingCuts.Count == 0 && (mode & CutMode.CutExtra) == 0 && (mode & CutMode.CutAll) != 0 && addIndex == -1) {
					//Just add the node and be done with it
					
					//Refer to vertices to be added a few lines below
					//watch4.Start();
					outtris.Add (outverts.Count+0);
					outtris.Add (outverts.Count+1);
					outtris.Add (outverts.Count+2);
					
					outverts.Add (tp1);
					outverts.Add (tp2);
					outverts.Add (tp3);
					//watch4.Stop();
					
					continue;
				}
				
				//watch2.Start();
				
				//watch2.Stop();
				
				//watch3.Start();
				
				
				//Add current triangle as subject polygon for cutting
				poly.Clear();
				if ( addIndex == -1 ) {
					// geometry from a tile mesh is assumed to be completely inside the tile
					poly.Add (new IntPoint (tp1.x,tp1.z));
					poly.Add (new IntPoint (tp2.x,tp2.z));
					poly.Add (new IntPoint (tp3.x,tp3.z));

				} else {
					// Added geometry must be clipped against the tile bounds
					clipIn[0] = tp1;
					clipIn[1] = tp2;
					clipIn[2] = tp3;

					int ct;
					ct = Pathfinding.Voxels.Utility.ClipPolygon (clipIn , 3 , clipOut ,  1 , 0, 0);
					if ( ct == 0 ) continue;

					ct = Pathfinding.Voxels.Utility.ClipPolygon (clipOut, ct , clipIn , -1 , 2*clipExtents.x, 0);
					if ( ct == 0 ) continue;

					ct = Pathfinding.Voxels.Utility.ClipPolygon (clipIn, ct , clipOut ,  1 , 0, 2);
					if ( ct == 0 ) continue;

					ct = Pathfinding.Voxels.Utility.ClipPolygon (clipOut, ct , clipIn , -1 , 2*clipExtents.z, 2);
					if ( ct == 0 ) continue;

					for ( int q=0;q<ct; q++ ) poly.Add ( new IntPoint ( clipIn[q].x, clipIn[q].z ) );
				}

				
				point2Index.Clear();
				
				//Set up some values to help in sampling Y coordinates from the original triangle
				Int3 d1 = tp2-tp1;
				Int3 d2 = tp3-tp1;
				
				Int3 d10 = d1;
				Int3 d20 = d2;
				d10.y = 0;
				d20.y = 0;
				
				//sol.Clear();
				//clipper.Execute(ClipType.ctDifference, sol, PolyFillType.pftEvenOdd, PolyFillType.pftNonZero);
				
				// Loop through all possible modes (just 4 at the moment, so < 4 could be used actually)
				for (int cmode=0; cmode < 16; cmode++) {
					
					// Ignore modes which are not active
					if ((((int)mode >> cmode) & 0x1) == 0) continue;
					
					if (1 << cmode == (int)CutMode.CutAll) {
						clipper.Clear();
						clipper.AddPolygon(poly, PolyType.ptSubject);
						
						//Add all holes (cuts) as clip polygons
						for (int i=0;i<tmpIntersectingCuts.Count;i++) {
							clipper.AddPolygon (cutVertices[tmpIntersectingCuts[i]], PolyType.ptClip);
						}
						sol.Clear();
						clipper.Execute(ClipType.ctDifference, sol, PolyFillType.pftEvenOdd, PolyFillType.pftNonZero);
						
					} else if (1 << cmode == (int)CutMode.CutDual) {
						// No duals, don't bother processing this
						if (!hasDual) continue;
						
						// First calculate
						// a = original intersection dualCuts
						// then
						// b = a difference normalCuts
						// then process b as normal
						clipper.Clear();
						clipper.AddPolygon(poly, PolyType.ptSubject);
						
						//Add all holes (cuts) as clip polygons
						for (int i=0;i<tmpIntersectingCuts.Count;i++) {
							if (cutIsDual[tmpIntersectingCuts[i]]) {
								clipper.AddPolygon (cutVertices[tmpIntersectingCuts[i]], PolyType.ptClip);
							}
						}
						
						sol2.Clear();
						clipper.Execute(ClipType.ctIntersection, sol2, PolyFillType.pftEvenOdd, PolyFillType.pftNonZero);
						
						clipper.Clear();
						
						for (int i=0;i<sol2.Count;i++) {
							clipper.AddPolygon (sol2[i], Pathfinding.ClipperLib.Clipper.Orientation (sol2[i]) ? PolyType.ptClip : PolyType.ptSubject);
							
							//for (int j=0;j<sol[i].holes.Count;j++) {
							//	clipper.AddPolygon (sol[i].holes[j], PolyType.ptClip);
							//}
						}
						
						for (int i=0;i<tmpIntersectingCuts.Count;i++) {
							if (!cutIsDual[tmpIntersectingCuts[i]]) {
								clipper.AddPolygon (cutVertices[tmpIntersectingCuts[i]], PolyType.ptClip);
							}
						}
						
						sol.Clear();
						clipper.Execute(ClipType.ctDifference, sol, PolyFillType.pftEvenOdd, PolyFillType.pftNonZero);
						
					} else if (1 << cmode == (int)CutMode.CutExtra) {
						clipper.Clear();
						clipper.AddPolygon(poly, PolyType.ptSubject);
						
						//Add all holes (cuts) as clip polygons
						//for (int i=0;i<tmpIntersectingCuts.Count;i++) {
						//	clipper.AddPolygon (cutVertices[tmpIntersectingCuts[i]], PolyType.ptClip);
						//}
						
						clipper.AddPolygon (extraClipShape, PolyType.ptClip);
						
						sol.Clear();
						
						clipper.Execute(ClipType.ctIntersection, sol, PolyFillType.pftEvenOdd, PolyFillType.pftNonZero);
					}
					
					
					for (int exp=0;exp<sol.ChildCount;exp++) {
						PolyNode node = sol.Childs[exp];
						List<IntPoint> outer = node.Contour;
						List<PolyNode> holes = node.Childs;
						
						if (holes.Count == 0 && outer.Count == 3 && addIndex == -1) {
							for (int i=0;i<outer.Count;i++) {
								
								Int3 p = new Int3((int)outer[i].X,0,(int)outer[i].Y);
								//Sample Y coordinate from original triangle
								//This will find the Y coordinate in the triangle at that XZ point
								
								double det = ((double)(tp2.z - tp3.z))*(tp1.x - tp3.x) + ((double)(tp3.x - tp2.x))*(tp1.z - tp3.z);
									
								if (det == 0) {
									Debug.LogWarning ("Degenerate triangle");
									continue;
								}
								
								double lambda1 = ((((double)(tp2.z-tp3.z))*(p.x-tp3.x)+((double)(tp3.x-tp2.x))*(p.z-tp3.z))/det);
								double lambda2 = ((((double)(tp3.z-tp1.z))*(p.x-tp3.x)+((double)(tp1.x-tp3.x))*(p.z-tp3.z))/det);
								
								p.y = (int)System.Math.Round(lambda1*tp1.y + lambda2*tp2.y + (1-lambda1-lambda2)*tp3.y);
								
								outtris.Add (outverts.Count);
								outverts.Add (p);
							}
						} else {
						
		#if ASTARDEBUG
							float offset = UnityEngine.Random.value*Int3.FloatPrecision;
		#endif
							
							Poly2Tri.Polygon pl = null;
							//Loop over outer and all holes
							int hole = -1;
							List<IntPoint> contour = outer;
							while (contour != null) {
								polypoints.Clear();
								for (int i=0;i<contour.Count;i++) {
		#if ASTARDEBUG
									Debug.DrawLine(new Vector3(contour[i].X,offset,contour[i].Y)*Int3.PrecisionFactor, new Vector3(contour[(i+1) % contour.Count].X,offset,contour[(i+1) % contour.Count].Y)*Int3.PrecisionFactor, Color.cyan);
									Debug.DrawRay(new Vector3(contour[i].X,offset,contour[i].Y)*Int3.PrecisionFactor, Vector3.up, Color.cyan);
		#endif
									
									/*
									 * if (rnd == null) rnd = new System.Random ();
								Vector3 rof = new Vector3 (0,(float)rnd.NextDouble()*2,0);
								*/
									//Create a new point
									PolygonPoint pp = new PolygonPoint(contour[i].X, contour[i].Y);
									
									//Add the point to the polygon
									polypoints.Add(pp);
									
									Int3 p = new Int3((int)contour[i].X,0,(int)contour[i].Y);
									
									double det = ((double)(tp2.z - tp3.z))*(tp1.x - tp3.x) + ((double)(tp3.x - tp2.x))*(tp1.z - tp3.z);
									
									if (det == 0) {
										Debug.LogWarning ("Degenerate triangle");
										continue;
									}
									
									double lambda1 = ((((double)(tp2.z-tp3.z))*(p.x-tp3.x)+((double)(tp3.x-tp2.x))*(p.z-tp3.z))/det);
									double lambda2 = ((((double)(tp3.z-tp1.z))*(p.x-tp3.x)+((double)(tp1.x-tp3.x))*(p.z-tp3.z))/det);
									
									p.y = (int)System.Math.Round(lambda1*tp1.y + lambda2*tp2.y + (1-lambda1-lambda2)*tp3.y);
									
									//Prepare a lookup table for pp -> vertex index
									point2Index[pp] = outverts.Count;
									
									//Add to resulting vertex list
									outverts.Add (p);
								}
								
								Poly2Tri.Polygon tmpPoly = null;
								if (polyCache.Count > 0) {
									tmpPoly = polyCache.Pop ();
									tmpPoly.AddPoints (polypoints);
								} else {
									tmpPoly = new Poly2Tri.Polygon(polypoints);
								}
								
								//Since the outer contour is the first to be processed, pl will be null
								//Holes are processed later, when pl is not null
								if (pl == null) {
									pl = tmpPoly;
								} else {
									pl.AddHole (tmpPoly);
								}
								
								hole++;
								contour = hole < holes.Count ? holes[hole].Contour : null;
							}
							
							//Triangulate the polygon with holes
							try {
								P2T.Triangulate(pl);
							} catch (Poly2Tri.PointOnEdgeException) {
								Debug.LogWarning ("PointOnEdgeException, perturbating vertices slightly ( at "+cmode+" in " + mode +")");
								
	#if ASTARDEBUG
								if ( rnd == null ) rnd = new System.Random();
								Color col = new Color((float)rnd.NextDouble(), (float)rnd.NextDouble(), (float)rnd.NextDouble());
								for (int i=0;i<pl.Points.Count;i++) {
									Debug.DrawLine (Point2D2V3(pl.Points[i]) + realBounds.min,Point2D2V3(pl.Points[(i+1) % pl.Points.Count]) + realBounds.min, Color.red);
								}
								if (pl.Holes != null) {
									foreach (Poly2Tri.Polygon hl in pl.Holes) {
										for (int i=0;i<hl.Points.Count;i++) {
											Debug.DrawLine (Point2D2V3(hl.Points[i]) + realBounds.min,Point2D2V3(hl.Points[(i+1) % hl.Points.Count]) + realBounds.min, col);
										}
									}
								}
	#endif
								
								CutPoly (verts, tris, ref outVertsArr, ref outTrisArr, out outVCount, out outTCount, extraShape, cuttingOffset, realBounds, mode, perturbate+1);
								return;
							}
							
							for (int i=0;i<pl.Triangles.Count;i++) {
								Poly2Tri.DelaunayTriangle t = pl.Triangles[i];
		#if ASTARDEBUG
								Debug.DrawLine (Point2D2V3(t.Points._0),Point2D2V3(t.Points._1),Color.red);
								Debug.DrawLine (Point2D2V3(t.Points._1),Point2D2V3(t.Points._2),Color.red);
								Debug.DrawLine (Point2D2V3(t.Points._2),Point2D2V3(t.Points._0),Color.red);
								
								Debug.DrawLine ((Vector3)outverts[point2Index[t.Points._0]]+Vector3.up*0.1f,(Vector3)outverts[point2Index[t.Points._1]]+Vector3.up*0.1f,Color.blue);
								Debug.DrawLine ((Vector3)outverts[point2Index[t.Points._1]]+Vector3.up*0.1f,(Vector3)outverts[point2Index[t.Points._2]]+Vector3.up*0.1f,Color.blue);
								Debug.DrawLine ((Vector3)outverts[point2Index[t.Points._2]]+Vector3.up*0.1f,(Vector3)outverts[point2Index[t.Points._0]]+Vector3.up*0.1f,Color.blue);
		#endif
								
								//Add the triangle with the correct indices (using the previously built lookup table)
								outtris.Add (point2Index[t.Points._0]);
								outtris.Add (point2Index[t.Points._1]);
								outtris.Add (point2Index[t.Points._2]);
							}
							
							if (pl.Holes != null) for (int i=0;i<pl.Holes.Count;i++) {
								pl.Holes[i].Points.Clear();
								pl.Holes[i].ClearTriangles();
								
								if (pl.Holes[i].Holes != null) pl.Holes[i].Holes.Clear();
								
								polyCache.Push(pl.Holes[i]);
							}
							pl.ClearTriangles();
							if (pl.Holes != null) pl.Holes.Clear();
							pl.Points.Clear();
							polyCache.Push(pl);
						}
					}
				}
				//watch3.Stop();
			}
			
			
			/*
			 * This next step will remove all duplicate vertices in the data (of which there are quite a few) 
			 * and output the final vertex and triangle arrays to the vert and tris variables
			 */
			//watch5.Start();
			
			Dictionary<Int3, int> firstVerts = cached_Int3_int_dict; //new Dictionary<Int3, int>();
			firstVerts.Clear();
			
			// Use cached array to reduce memory allocations
			if (cached_int_array.Length < outverts.Count) cached_int_array = new int[System.Math.Max(cached_int_array.Length*2, outverts.Count)];
			int[] compressedPointers = cached_int_array;
			
			int count = 0;
			for (int i=0;i<outverts.Count;i++) {
				int ind;
				if (!firstVerts.TryGetValue(outverts[i], out ind)) {
					
					firstVerts.Add (outverts[i], count);
					compressedPointers[i] = count;
					outverts[count] = outverts[i];
					count++;
					
				} else {
					compressedPointers[i] = ind;
				}
			}
			
			//Finalized triangle array
			outTCount = outtris.Count;
			if (outTrisArr == null || outTrisArr.Length < outTCount) outTrisArr = new int[outTCount];
			
			for (int i=0;i<outTCount;i++) {
				outTrisArr[i] = compressedPointers[outtris[i]];
			}
			
			//Out-vertices, compressed
			outVCount = count;
			if (outVertsArr == null || outVertsArr.Length < outVCount) outVertsArr = new Int3[outVCount];
			for (int i=0;i<outVCount;i++) outVertsArr[i] = outverts[i];
			
			// Notify that they were used
			for (int i=0; i<navmeshCuts.Count;i++) {
				navmeshCuts[i].UsedForCut();
			}
			
			//Release back to object pool
			Pathfinding.Util.ListPool<Int3>.Release(outverts);
			Pathfinding.Util.ListPool<int>.Release(outtris);
			Pathfinding.Util.ListPool<int>.Release(tmpIntersectingCuts);
			Pathfinding.Util.ListPool<Int2>.Release(cutBoundsY);
			Pathfinding.Util.ListPool<bool>.Release(cutIsDual);
			Pathfinding.Util.ListPool<bool>.Release(cutsAddedGeom);
			Pathfinding.Util.ListPool<IntRect>.Release(cutBounds);
			Pathfinding.Util.ListPool<NavmeshCut>.Release(navmeshCuts);

			//watch5.Stop();
			//watch.Stop();
			
			/*System.Console.WriteLine("");
			System.Console.WriteLine (watch2.ToString());
			System.Console.WriteLine (watch3.ToString());
			System.Console.WriteLine (watch4.ToString());
			System.Console.WriteLine (watch5.ToString());
			System.Console.WriteLine (watch6.ToString());
			System.Console.WriteLine (watch8.ToString());
			System.Console.WriteLine (watch.ToString());*/
		}
		
		/** Refine a mesh using delaunay refinement.
		 * Loops through all pairs of neighbouring triangles and check if it would be better to flip the diagonal joining them
		 * using the delaunay criteria.
		 * 
		 * Does not require triangles to be clockwise, triangles will be checked for if they are clockwise and made clockwise if not.
		 * The resulting mesh will have all triangles clockwise.
		 */
		void DelaunayRefinement (Int3[] verts, int[] tris, ref int vCount, ref int tCount, bool delaunay, bool colinear, Int3 worldOffset) {
			
			if (tCount % 3 != 0) throw new System.Exception ("Triangle array length must be a multiple of 3");
			
			Dictionary<Int2, int> lookup = this.cached_Int2_int_dict;
			lookup.Clear();
			
			for (int i=0;i<tCount;i+=3) {
				if (!Polygon.IsClockwise(verts[tris[i]],verts[tris[i+1]],verts[tris[i+2]])) {
					int tmp = tris[i];
					tris[i] = tris[i+2];
					tris[i+2] = tmp;
				}
				
				lookup[new Int2(tris[i+0],tris[i+1])] = i+2;
				lookup[new Int2(tris[i+1],tris[i+2])] = i+0;
				lookup[new Int2(tris[i+2],tris[i+0])] = i+1;
			}
			
			int maxError = 3 *3;//(int)((graph.contourMaxError*Int3.Precision)*(graph.contourMaxError*Int3.Precision));
			
			for (int i=0;i<tCount;i+=3) {
				for (int j=0;j<3;j++) {
					int opp;
					//Debug.DrawLine ((Vector3)verts[tris[i+((j+1)%3)]], (Vector3)verts[tris[i+((j+0)%3)]], Color.yellow);
					
					if (lookup.TryGetValue(new Int2(tris[i+((j+1)%3)], tris[i+((j+0)%3)]), out opp)) {
						
						Int3 po = verts[tris[i+((j+2)%3)]];
						Int3 pr = verts[tris[i+((j+1)%3)]];
						Int3 pl = verts[tris[i+((j+3)%3)]];
						
						//Debug.DrawLine (pr, pl, Color.red);
						//Debug.DrawLine (po, (pl+pr)/2, Color.blue);
						
						
						Int3 popp = verts[tris[opp]];
						
						//continue;
						
						po.y = 0;
						pr.y = 0;
						pl.y = 0;
						popp.y = 0;
						
						bool noDelaunay = false;
						
						if (!Polygon.Left (po,pl,popp) || Polygon.LeftNotColinear(po,pr,popp)) {
							//Debug.DrawLine (po, popp, Color.red);
							if (colinear) {
								noDelaunay = true;
							} else {
								continue;
							}
						}
						
						if (colinear) {
							
							/*if (AstarMath.DistancePointSegment (po, popp, pl) < maxError) {
								Debug.DrawLine ((Vector3)po, (Vector3)popp, Color.blue);
								Debug.Break();
							}
							
							if (Polygon.IsColinear (po, pr, popp)) {
								Debug.DrawLine ((Vector3)po, (Vector3)popp, Color.blue);
								Debug.Break();
							}*/
							
							// Check if op - right shared - opposite in other - is colinear
							// and if the edge right-op is not shared and if the edge opposite in other - right shared is not shared
							if (AstarMath.DistancePointSegment (po, popp, pr) < maxError &&
								!lookup.ContainsKey(new Int2(tris[i+((j+2)%3)], tris[i+((j+1)%3)])) &&
								!lookup.ContainsKey(new Int2(tris[i+((j+1)%3)], tris[opp]))) {
								
								//Debug.DrawLine ((Vector3)(po+worldOffset), (Vector3)(pr+worldOffset), Color.red);
								//Debug.DrawLine ((Vector3)(pr+worldOffset), (Vector3)(popp+worldOffset), Color.blue);
								//Debug.Break();
								
								tCount -= 3;
								
								int root = (opp/3)*3;
								
								// Move right vertex to the other triangle's opposite
								tris[i+((j+1)%3)] = tris[opp];
								
								// Move last triangle to delete
								if (root != tCount) {
									tris[root+0] = tris[tCount+0];
									tris[root+1] = tris[tCount+1];
									tris[root+2] = tris[tCount+2];
									lookup[new Int2(tris[root+0],tris[root+1])] = root+2;
									lookup[new Int2(tris[root+1],tris[root+2])] = root+0;
									lookup[new Int2(tris[root+2],tris[root+0])] = root+1;
									
									tris[tCount+0] = 0;
									tris[tCount+1] = 0;
									tris[tCount+2] = 0;
								} else {
									tCount += 3;
								}
								
								// Since the above mentioned edges are not shared, we don't need to bother updating them
								
								// However some need to be updated
								// left - new right (previously opp) should have opposite vertex po
								//lookup[new Int2(tris[i+((j+3)%3)],tris[i+((j+1)%3)])] = i+((j+2)%3);
								
								lookup[new Int2(tris[i+0],tris[i+1])] = i+2;
								lookup[new Int2(tris[i+1],tris[i+2])] = i+0;
								lookup[new Int2(tris[i+2],tris[i+0])] = i+1;
								continue;
							}
							
							// Check if op - left shared - opposite in other - is colinear
							// and if the edge left-op is not shared and if the edge opposite in other - left shared is not shared
							/*if (AstarMath.DistancePointSegment (po, popp, pl) < maxError &&
								!lookup.ContainsKey(new Int2(tris[i+((j+3)%3)], tris[i+((j+2)%3)])) &&
								!lookup.ContainsKey(new Int2(tris[opp], tris[i+((j+3)%3)]))) {
								
								Debug.DrawLine ((Vector3)po, (Vector3)popp, Color.red);
								Debug.Break();
								
								triCount -= 3;
								
								int root = (opp/3)*3;
								
								// Move left vertex to the other triangle's opposite
								tris[i+((j+3)%3)] = tris[opp];
								
								// Move last triangle to delete
								if (root != triCount) {
									tris[root+0] = tris[triCount+0];
									tris[root+1] = tris[triCount+1];
									tris[root+2] = tris[triCount+2];
									lookup[new Int2(tris[root+0],tris[root+1])] = root+2;
									lookup[new Int2(tris[root+1],tris[root+2])] = root+0;
									lookup[new Int2(tris[root+2],tris[root+0])] = root+1;
									
									tris[triCount+0] = 0;
									tris[triCount+1] = 0;
									tris[triCount+2] = 0;
								} else {
									triCount += 3;
								}
								// Since the above mentioned edges are not shared, we don't need to bother updating them
								
								// However some need to be updated
								// right - new left (previously opp) should have opposite vertex po
								//lookup[new Int2(tris[i+((j+1)%3)],tris[i+((j+3)%3)])] = i+((j+2)%3);
								
								lookup[new Int2(tris[i+0],tris[i+1])] = i+2;
								lookup[new Int2(tris[i+1],tris[i+2])] = i+0;
								lookup[new Int2(tris[i+2],tris[i+0])] = i+1;
								continue;
							}*/
							
						}
						if (delaunay && !noDelaunay) {
							float beta = Int3.Angle(pr-po,pl-po);
							float alpha = Int3.Angle(pr-popp,pl-popp);
							//Debug.Log (beta + " " + alpha + " " +  (2*Mathf.PI - 2*beta));
							if (alpha > (2*Mathf.PI - 2*beta)) {
								//Debug.DrawLine (po, popp, Color.green);
								//Denaunay condition not holding, refine please
								tris[i+((j+1)%3)] = tris[opp];
								
								int root = (opp/3)*3;
								int off = opp-root;
								tris[root+((off-1+3) % 3)] = tris[i+((j+2)%3)];
								
								
								/*if (!Polygon.IsClockwise(verts[tris[i]],verts[tris[i+1]],verts[tris[i+2]])) {
									Debug.LogError("Non clockwise triangle");
									int tmp2 = tris[i];
									tris[i] = tris[i+2];
									tris[i+2] = tmp2;
								}*/
								
								lookup[new Int2(tris[i+0],tris[i+1])] = i+2;
								lookup[new Int2(tris[i+1],tris[i+2])] = i+0;
								lookup[new Int2(tris[i+2],tris[i+0])] = i+1;
								
								lookup[new Int2(tris[root+0],tris[root+1])] = root+2;
								lookup[new Int2(tris[root+1],tris[root+2])] = root+0;
								lookup[new Int2(tris[root+2],tris[root+0])] = root+1;
								
								//i = tris.Length; j = 3;
								//continue;
							} else {
								//Debug.DrawLine (po, popp, Color.cyan);
								
								
							}
						}
					}
				}
			}
		}
		
		Vector3 Point2D2V3 (Poly2Tri.TriangulationPoint p) {
			return new Vector3((float)p.X,0,(float)p.Y)*Int3.PrecisionFactor;
		}
		
		Int3 IntPoint2Int3 (IntPoint p) {
			return new Int3((int)p.X,0,(int)p.Y);
		}
		
		public void ClearTile (int x, int z) {
			
			if (AstarPath.active == null) return;
			
			if ( x < 0 || z < 0 || x >= graph.tileXCount || z >= graph.tileZCount ) return;
			
			AstarPath.active.AddWorkItem (new AstarPath.AstarWorkItem (delegate (bool force) {
				
				//Replace the tile using the final vertices and triangles
				graph.ReplaceTile(x,z,new Int3[0], new int[0], false);
				
				activeTileTypes[x + z*graph.tileXCount] = null;
				//Trigger post update event
				//This can trigger for example recalculation of navmesh links
				GraphModifier.TriggerEvent (GraphModifier.EventType.PostUpdate);
				
				//Flood fill everything to make sure graph areas are still valid
				//This tends to take more than 50% of the calculation time
				AstarPath.active.QueueWorkItemFloodFill();
				
				/*if (!AstarPath.active.isScanning) {
					
					AstarPath.active.FloodFill();
				}*/
				
				return true;
				
			}));
		}
		
		/** Reloads all tiles intersecting with the specified bounds */
		public void ReloadInBounds (Bounds b) {
			
			
			Int2 min = graph.GetTileCoordinates (b.min);
			Int2 max = graph.GetTileCoordinates (b.max);
			IntRect r = new IntRect(min.x,min.y,max.x,max.y);
			
			// Make sure the rect is inside graph bounds
			r = IntRect.Intersection (r, new IntRect(0,0,graph.tileXCount-1,graph.tileZCount-1));
			
			if (!r.IsValid()) return;
			
			for (int z=r.ymin;z<=r.ymax;z++) {
				for (int x=r.xmin;x<=r.xmax;x++) {
					ReloadTile (x,z);
				}
			}
		}
		
		/** Reload tile at tile coordinate.
		  * The last tile loaded at that position will be reloaded (e.g to account for moved navmesh cut components)
		  */
		public void ReloadTile (int x, int z) {
			if ( x < 0 || z < 0 || x >= graph.tileXCount || z >= graph.tileZCount ) return;
			
			int index = x + z*graph.tileXCount;
			if ( activeTileTypes[index] != null ) LoadTile (activeTileTypes[index], x, z, activeTileRotations[index], activeTileOffsets[index]);
		}
		
		public void CutShapeWithTile (int x, int z, Int3[] shape, ref Int3[] verts, ref int[] tris, out int vCount, out int tCount) {
			if (isBatching) {
				throw new System.Exception ("Cannot cut with shape when batching. Please stop batching first.");
			}
			
			int index = x + z*graph.tileXCount;
			
			if (x < 0 || z < 0 || x >= graph.tileXCount || z >= graph.tileZCount || activeTileTypes[index] == null) {
				verts = new Int3[0];
				tris = new int[0];
				vCount = 0;
				tCount = 0;
				return;
			}
			
			Int3[] tverts;
			int[] ttris;
				
			activeTileTypes[index].Load (out tverts, out ttris, activeTileRotations[index], activeTileOffsets[index]);
			
			//Calculate tile bounds so that the correct cutting offset can be used
			//The tile will be cut in local space (i.e it is at the world origin) so cuts need to be translated
			//to that point from their world space coordinates
			Bounds r = graph.GetTileBounds(x,z);
			Int3 cutOffset = (Int3)r.min;
			cutOffset = -cutOffset;
			
			this.CutPoly (tverts, ttris, ref verts, ref tris, out vCount, out tCount, shape, cutOffset, r, CutMode.CutExtra);
			
			for (int i=0; i < verts.Length; i++ ) verts[i] -= cutOffset;
		}
		
		/** Returns a new array with at most length \a newLength.
		 * The array will a copy of all elements of \a arr.
		 */
		protected static T[] ShrinkArray<T> (T[] arr, int newLength) {
			newLength = System.Math.Min(newLength, arr.Length);
			T[] arr2 = new T[newLength];
			
			// Unrolling
			if (newLength % 4 == 0) {
				for (int i=0;i<newLength;i+=4) {
					arr2[i+0] = arr[i+0];
					arr2[i+1] = arr[i+1];
					arr2[i+2] = arr[i+2];
					arr2[i+3] = arr[i+3];
				}
			} else if (newLength % 3 == 0) {
				for (int i=0;i<newLength;i+=3) {
					arr2[i+0] = arr[i+0];
					arr2[i+1] = arr[i+1];
					arr2[i+2] = arr[i+2];
				}
			} else if (newLength % 2 == 0) {
				for (int i=0;i<newLength;i+=2) {
					arr2[i+0] = arr[i+0];
					arr2[i+1] = arr[i+1];
				}
			} else {
				for (int i=0;i<newLength;i++) {
					arr2[i+0] = arr[i+0];
				}
			}
			return arr2;
		}
		
		/** Load a tile \a id at tile coordinate \a x, \a z.
		 * 
		 * \param id Tile id as got from RegisterTile
		 * \param x Tile x coordinate (first tile is at (0,0), second at (1,0) etc.. ).
		 * \param z Tile z coordinate.
		 * \param rotation Rotate tile by 90 degrees * value.
		 * \param yoffset Offset Y coordinates by this amount. In Int3 space, so if you have a world space
		 * offset, multiply by Int3.Precision and round to the nearest integer before calling this function.
		 */
		public void LoadTile (TileType tile, int x, int z, int rotation, int yoffset) {
			if (tile == null) throw new System.ArgumentNullException("tile");
			
			if (AstarPath.active == null) return;
			
			int index = x + z*graph.tileXCount;
			rotation = rotation % 4;
			
			// If loaded during this batch with the same settings, skip it
			if (isBatching && reloadedInBatch[index] && activeTileOffsets[index] == yoffset && activeTileRotations[index] == rotation && activeTileTypes[index] == tile) {
				return;
			}
			
			if (isBatching) {
				reloadedInBatch[index] = true;
			}
			
			activeTileOffsets[index] = yoffset;
			activeTileRotations[index] = rotation;
			activeTileTypes[index] = tile;
			
			//Add a work item
			//This will pause pathfinding as soon as possible
			//and call the delegate when it is safe to update graphs
			AstarPath.active.AddWorkItem (new AstarPath.AstarWorkItem (delegate (bool force) {
				
				// If this was not the correct settings to load with, ignore
				if ( !(activeTileOffsets[index] == yoffset && activeTileRotations[index] == rotation && activeTileTypes[index] == tile )) return true;
				
				GraphModifier.TriggerEvent (GraphModifier.EventType.PreUpdate);
				
				//Profile timer = new Profile("Replace Tile");
				//timer.Start();
				
				Int3[] verts;
				int[] tris;
				
				tile.Load ( out verts, out tris, rotation, yoffset);
				
				//Calculate tile bounds so that the correct cutting offset can be used
				//The tile will be cut in local space (i.e it is at the world origin) so cuts need to be translated
				//to that point from their world space coordinates
				Bounds r = graph.GetTileBounds (x,z, tile.Width, tile.Depth);
				Int3 cutOffset = (Int3)r.min;
				cutOffset = -cutOffset;
				
				Int3[] outVerts = null;
				int[] outTris = null;
				int vCount, tCount;
				
				//Cut the polygon
				CutPoly (verts, tris, ref outVerts, ref outTris, out vCount, out tCount, null, cutOffset, r);
				
				//Refine to remove bad triangles
				DelaunayRefinement(outVerts, outTris, ref vCount, ref tCount, true, false, -cutOffset);
				
				if (tCount != outTris.Length) outTris = ShrinkArray (outTris, tCount);
				if (vCount != outVerts.Length) outVerts = ShrinkArray (outVerts, vCount);
				
				// Rotate the mask correctly
				// and update width and depth to match rotation
				// (width and depth will swap if rotated 90 or 270 degrees )
				//int resMask = 0;
				//int initMask = tile.mask;
				int newWidth = rotation % 2 == 0 ? tile.Width : tile.Depth;
				int newDepth = rotation % 2 == 0 ? tile.Depth : tile.Width;
				
				/*Int2 offset = new Int2(0,0);
				if ( rotation == 1 || rotation == 2) offset.y = newDepth;
				if ( rotation == 3 || rotation == 2 ) offset.x = newWidth;
				
				for ( int cx=0; cx<tile.Width; cx++ ) {
					for ( int cz=0; cz<tile.Depth; cz++ ) {
						if ( ((initMask >> (cz*tile.Width + cx)) & 0x1 ) != 0 ) {
							Int2 p = offset + Int2.Rotate ( new Int2(cx,cz), rotation );
							resMask |= 1 << ( p.z*newWidth + p.x );
						}
					}
				}*/
				
				//Replace the tile using the final vertices and triangles
				//The vertices are still in local space
				graph.ReplaceTile ( x, z, newWidth, newDepth, outVerts, outTris, false );
				
				//Trigger post update event
				//This can trigger for example recalculation of navmesh links
				GraphModifier.TriggerEvent (GraphModifier.EventType.PostUpdate);
				
				//Flood fill everything to make sure graph areas are still valid
				//This tends to take more than 50% of the calculation time
				AstarPath.active.QueueWorkItemFloodFill();
				
				/*if (!AstarPath.active.isScanning && !wasBatching) {
					//Flood fill everything to make sure graph areas are still valid
					//This tends to take more than 50% of the calculation time
					AstarPath.active.FloodFill();
				}*/
				
				//timer.Stop ();
				//System.Console.WriteLine ( timer.ToString());
				
				return true;
			}));
		}
		
	}
}

