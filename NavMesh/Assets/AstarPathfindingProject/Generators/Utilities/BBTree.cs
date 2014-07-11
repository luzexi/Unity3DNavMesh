//#define ASTARDEBUG   //"BBTree Debug" If enables, some queries to the tree will show debug lines. Turn off multithreading when using this since DrawLine calls cannot be called from a different thread

//#define ASTAR_OLD_BBTREE // Use class based BBTree implementation instead of struct based. Struct based is better for runtime performance and memory, but class based scans slightly faster.
using System;
using UnityEngine;
using Pathfinding;
using System.Collections.Generic;

namespace Pathfinding
{
	/** Axis Aligned Bounding Box Tree.
	 * Holds a bounding box tree of triangles.\n
	 * \b Performance: Insertion - Practically O(1) - About 0.003 ms
	 * \astarpro
	 */
	public class BBTree
	{

#if !ASTAR_OLD_BBTREE

		/** Holds an Axis Aligned Bounding Box Tree used for faster node lookups.
		 * \astarpro */
		BBTreeBox[] arr = new BBTreeBox[6];
		int count = 0;
		public INavmeshHolder graph;

		public BBTree (INavmeshHolder graph) {
			this.graph = graph;
		}
		
		public Rect Size {
			get {
				return count != 0 ? arr[0].rect : new Rect (0,0,0,0);
			}
		}

		/** Clear the tree.
		 * Note that references to old nodes will still be intact so the GC cannot immediately collect them.
		 */
		public void Clear () {
			count = 0;
		}

		void EnsureCapacity ( int c ) {
			if ( arr.Length < c ) {
				var narr = new BBTreeBox[System.Math.Max ( c , (int)(arr.Length*1.5f))];
				for ( int i = 0; i < count; i++ ) {
					narr[i] = arr[i];
				}
				arr = narr;
			}
		}

		int GetBox ( MeshNode node ) {
			if ( count >= arr.Length ) EnsureCapacity ( count+1 );

			arr[count] = new BBTreeBox ( this, node );
			count++;
			return count-1;
		}


		/** Inserts a mesh node in the tree */
		public void Insert (MeshNode node) {
			int boxi = GetBox (node);

			// Was set to root
			if (boxi == 0) {
				return;
			}

			BBTreeBox box = arr[boxi];

			//int depth = 0;

			int c = 0;
			while (true) {

				BBTreeBox cb = arr[c];

				cb.rect = ExpandToContain (cb.rect,box.rect);
				if (cb.node != null) {
					//Is Leaf
					cb.left = boxi;

					int box2 = GetBox (cb.node);
					//BBTreeBox box2 = new BBTreeBox (this,c.node);
					
					//Console.WriteLine ("Inserted "+box.node+", rect "+box.rect.ToString ());
					cb.right = box2;
					
					
					cb.node = null;
					//cb.depth++;

					//c.rect = c.rect.
					arr[c] = cb;
					//Debug.Log (depth);
					return;
				} else {
					//depth++;
					//cb.depth++;
					arr[c] = cb;

					float e1 = ExpansionRequired (arr[cb.left].rect,box.rect);// * arr[cb.left].depth;
					float e2 =  ExpansionRequired (arr[cb.right].rect,box.rect);// * arr[cb.left].depth;
					
					//Choose the rect requiring the least expansion to contain box.rect
					if (e1 < e2) {
						c = cb.left;
					} else if (e2 < e1) {
						c = cb.right;
					} else {
						//Equal, Choose the one with the smallest area
						c = RectArea (arr[cb.left].rect) < RectArea (arr[cb.right].rect) ? cb.left : cb.right;
					}
				}
			}
		}

		public NNInfo Query (Vector3 p, NNConstraint constraint) {
			
			if ( count == 0 ) return new NNInfo(null);
			
			NNInfo nnInfo = new NNInfo ();
			
			SearchBox (0,p, constraint, ref nnInfo);
			
			nnInfo.UpdateInfo ();
			
			return nnInfo;
		}
		
		/** Queries the tree for the best node, searching within a circle around \a p with the specified radius.
		  * Will fill in both the constrained node and the not constrained node in the NNInfo.
		  * 
		  * \see QueryClosest
		  */
		public NNInfo QueryCircle (Vector3 p, float radius, NNConstraint constraint) {

			if ( count == 0 ) return new NNInfo(null);
			
			NNInfo nnInfo = new NNInfo (null);
			
			SearchBoxCircle (0,p, radius, constraint, ref nnInfo);
			
			nnInfo.UpdateInfo ();
			
			return nnInfo;
		}

		/** Queries the tree for the closest node to \a p constrained by the NNConstraint.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * \see QueryCircle
		  */
		public NNInfo QueryClosest (Vector3 p, NNConstraint constraint, out float distance) {
			distance = float.PositiveInfinity;
			return QueryClosest (p, constraint, ref distance, new NNInfo (null));
		}

		/** Queries the tree for the closest node to \a p constrained by the NNConstraint trying to improve an existing solution.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * This search will start from the \a previous NNInfo and improve it if possible.
		  * Even if the search fails on this call, the solution will never be worse than \a previous.
		  * 
		  * This method will completely ignore any Y-axis differences in positions.
		  * 
		  * \param distance The best distance for the \a previous solution. Will be updated with the best distance
		  * after this search. Will be positive infinity if no node could be found.
		  * Set to positive infinity if there was no previous solution.
		  * 
		  * 
		  * \see QueryCircle
		  */
		public NNInfo QueryClosestXZ (Vector3 p, NNConstraint constraint, ref float distance, NNInfo previous) {

			if (count == 0) {
				return previous;
			}
			
			SearchBoxClosestXZ (0,p, ref distance, constraint, ref previous);
			
			return previous;
		}

		void SearchBoxClosestXZ (int boxi, Vector3 p, ref float closestDist, NNConstraint constraint, ref NNInfo nnInfo) {

			BBTreeBox box = arr[boxi];

			if (box.node != null) {
				//Leaf node
				//if (NodeIntersectsCircle (box.node,p,closestDist)) {
				//Update the NNInfo
				#if ASTARDEBUG
				Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
				Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
				Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
				#endif
				
				Vector3 closest = box.node.ClosestPointOnNodeXZ (p);//NavMeshGraph.ClosestPointOnNode (box.node,graph.vertices,p);
				
				// XZ distance
				float dist = (closest.x-p.x)*(closest.x-p.x)+(closest.z-p.z)*(closest.z-p.z);
				
				if (constraint == null || constraint.Suitable (box.node)) {
					if (nnInfo.constrainedNode == null) {
						nnInfo.constrainedNode = box.node;
						nnInfo.constClampedPosition = closest;
						closestDist = (float)System.Math.Sqrt (dist);
					} else if (dist < closestDist*closestDist) {
						nnInfo.constrainedNode = box.node;
						nnInfo.constClampedPosition = closest;
						closestDist = (float)System.Math.Sqrt (dist);
					}
				}
				//} else {
				#if ASTARDEBUG
				Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
				Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
				Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
				#endif
				//}
			} else {
				
				#if ASTARDEBUG
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				#endif
				
				//Search children
				if (RectIntersectsCircle (arr[box.left].rect,p,closestDist)) {
					SearchBoxClosestXZ (box.left,p, ref closestDist, constraint, ref nnInfo);
				}
				
				if (RectIntersectsCircle (arr[box.right].rect,p,closestDist)) {
					SearchBoxClosestXZ (box.right,p, ref closestDist, constraint, ref nnInfo);
				}
			}
		}

		/** Queries the tree for the closest node to \a p constrained by the NNConstraint trying to improve an existing solution.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * This search will start from the \a previous NNInfo and improve it if possible.
		  * Even if the search fails on this call, the solution will never be worse than \a previous.
		  * 
		  * 
		  * \param distance The best distance for the \a previous solution. Will be updated with the best distance
		  * after this search. Will be positive infinity if no node could be found.
		  * Set to positive infinity if there was no previous solution.
		  * 
		  * 
		  * \see QueryCircle
		  */
		public NNInfo QueryClosest (Vector3 p, NNConstraint constraint, ref float distance, NNInfo previous) {
		
			if ( count == 0 ) return previous;
			
			SearchBoxClosest (0,p, ref distance, constraint, ref previous);
			
			return previous;
		}
		
		void SearchBoxClosest (int boxi, Vector3 p, ref float closestDist, NNConstraint constraint, ref NNInfo nnInfo) {

			BBTreeBox box = arr[boxi];

			if (box.node != null) {
				//Leaf node
				if (NodeIntersectsCircle (box.node,p,closestDist)) {
					//Update the NNInfo
					#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
					#endif
					
					Vector3 closest = box.node.ClosestPointOnNode (p);//NavMeshGraph.ClosestPointOnNode (box.node,graph.vertices,p);
					
					float dist = (closest-p).sqrMagnitude;
					
					if (constraint == null || constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
							closestDist = (float)System.Math.Sqrt (dist);
						} else if (dist < closestDist*closestDist) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
							closestDist = (float)System.Math.Sqrt (dist);
						}
					}
				} else {
					#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
					#endif
				}
			} else {
				
				#if ASTARDEBUG
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				#endif
				
				//Search children
				if (RectIntersectsCircle (arr[box.left].rect,p,closestDist)) {
					SearchBoxClosest (box.left,p, ref closestDist, constraint, ref nnInfo);
				}
				
				if (RectIntersectsCircle (arr[box.right].rect,p,closestDist)) {
					SearchBoxClosest (box.right,p, ref closestDist, constraint, ref nnInfo);
				}
			}
		}
		
		public MeshNode QueryInside (Vector3 p, NNConstraint constraint) {

			if ( count == 0 ) return null;
			
			return SearchBoxInside (0,p, constraint);
		}
		
		MeshNode SearchBoxInside (int boxi, Vector3 p, NNConstraint constraint) {

			BBTreeBox box = arr[boxi];

			if (box.node != null) {
				if (box.node.ContainsPoint ((Int3)p)) {
					//Update the NNInfo
					
					#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
					#endif
					
					
					if (constraint == null || constraint.Suitable (box.node)) {
						return box.node;
					}
				} else {
					#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
					#endif
				}
			} else {
				
				#if ASTARDEBUG
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				#endif
				
				//Search children
				MeshNode g;
				if (arr[box.left].rect.Contains (new Vector2(p.x,p.z))) {
					g = SearchBoxInside (box.left,p, constraint);
					if (g != null) return g;
				}
				
				if (arr[box.right].rect.Contains (new Vector2(p.x,p.z))) {
					g = SearchBoxInside (box.right, p, constraint);
					if (g != null) return g;
				}
			}
			
			return null;
		}
		
		void SearchBoxCircle (int boxi, Vector3 p, float radius, NNConstraint constraint, ref NNInfo nnInfo) {//, int intendentLevel = 0) {

			BBTreeBox box = arr[boxi];

			if (box.node != null) {
				//Leaf node
				if (NodeIntersectsCircle (box.node,p,radius)) {
					//Update the NNInfo
					
					#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.red);
					#endif
					
					Vector3 closest = box.node.ClosestPointOnNode (p);//NavMeshGraph.ClosestPointOnNode (box.node,graph.vertices,p);
					float dist = (closest-p).sqrMagnitude;
					
					if (nnInfo.node == null) {
						nnInfo.node = box.node;
						nnInfo.clampedPosition = closest;
					} else if (dist < (nnInfo.clampedPosition - p).sqrMagnitude) {
						nnInfo.node = box.node;
						nnInfo.clampedPosition = closest;
					}
					if (constraint == null || constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
						} else if (dist < (nnInfo.constClampedPosition - p).sqrMagnitude) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
						}
					}
				} else {
					#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
					#endif
				}
				return;
			}
			
			#if ASTARDEBUG
			Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
			Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
			Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
			Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
			#endif
			
			//Search children
			if (RectIntersectsCircle (arr[box.left].rect,p,radius)) {
				SearchBoxCircle (box.left,p, radius, constraint, ref nnInfo);
			}
			
			if (RectIntersectsCircle (arr[box.right].rect,p,radius)) {
				SearchBoxCircle (box.right,p, radius, constraint, ref nnInfo);
			}
		}
		
		void SearchBox (int boxi, Vector3 p, NNConstraint constraint, ref NNInfo nnInfo) {//, int intendentLevel = 0) {

			BBTreeBox box = arr[boxi];

			if (box.node != null) {
				//Leaf node
				if (box.node.ContainsPoint ((Int3)p)) {
					//Update the NNInfo
					
					if (nnInfo.node == null) {
						nnInfo.node = box.node;
					} else if (Mathf.Abs(((Vector3)box.node.position).y - p.y) < Mathf.Abs (((Vector3)nnInfo.node.position).y - p.y)) {
						nnInfo.node = box.node;
					}
					if (constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
						} else if (Mathf.Abs(box.node.position.y - p.y) < Mathf.Abs (nnInfo.constrainedNode.position.y - p.y)) {
							nnInfo.constrainedNode = box.node;
						}
					}
				}
				return;
			}
			
			//Search children
			if (RectContains (arr[box.left].rect,p)) {
				SearchBox (box.left,p, constraint, ref nnInfo);
			}
			
			if (RectContains (arr[box.right].rect,p)) {
				SearchBox (box.right,p, constraint, ref nnInfo);
			}
		}

		//[System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
		struct BBTreeBox {
			public Rect rect;

			public MeshNode node;
			public int left, right;
			//public short depth;

			public bool IsLeaf {
				get {
					return node != null;
				}
			}

			public BBTreeBox (BBTree tree, MeshNode node) {
				this.node = node;
				//depth = 0;
				Vector3 first = (Vector3)node.GetVertex(0);
				Vector2 min = new Vector2(first.x,first.z);
				Vector2 max = min;
				
				for (int i=1;i<node.GetVertexCount();i++) {
					Vector3 p = (Vector3)node.GetVertex(i);
					min.x = System.Math.Min (min.x,p.x);
					min.y = System.Math.Min (min.y,p.z);
					
					max.x = System.Math.Max (max.x,p.x);
					max.y = System.Math.Max (max.y,p.z);
				}
				
				rect = Rect.MinMaxRect (min.x,min.y,max.x,max.y);
				left = right = -1;
			}
			
			public bool Contains (Vector3 p) {
				return rect.Contains (new Vector2(p.x,p.z));
			}
		}

		public void OnDrawGizmos () {
			Gizmos.color = new Color (1,1,1,0.5F);
			if ( count == 0 ) return;
			//OnDrawGizmos (0, 0);
			//Debug.Log ("Size " + arr.Length + " actual count " + count );
		}

		void OnDrawGizmos ( int boxi, int depth ) {
			BBTreeBox box = arr[boxi];

			Vector3 min = new Vector3 (box.rect.xMin,0,box.rect.yMin);
			Vector3 max = new Vector3 (box.rect.xMax,0,box.rect.yMax);
			
			Vector3 center = (min+max)*0.5F;
			Vector3 size = (max-center)*2;
			center.y += depth * 0.2f;

			//Gizmos.color = new Color (1,1,1,0.5F);
			//Gizmos.DrawWireCube (center,size);
			Gizmos.color = AstarMath.IntToColor (depth, 0.05f);//new Color (0,0,0,0.2F);
			Gizmos.DrawCube (center,size);

			if ( box.node != null ) {
			} else {
				OnDrawGizmos ( box.left, depth + 1 );
				OnDrawGizmos ( box.right, depth + 1 );
			}
		}
#else

		/** Holds an Axis Aligned Bounding Box Tree used for faster node lookups.
		 * \astarpro */
		BBTreeBox root;
		public INavmeshHolder graph;
		
		public BBTree (INavmeshHolder graph) {
			this.graph = graph;
		}

		public Rect Size {
			get {
				return root != null ? root.rect : new Rect (0,0,0,0);
			}
		}

		public void Clear () {
			root = null;
		}

		public NNInfo Query (Vector3 p, NNConstraint constraint) {
			
			BBTreeBox c = root;
			
			if (c == null) {
				return new NNInfo();
			}
			
			NNInfo nnInfo = new NNInfo ();
			
			SearchBox (c,p, constraint, ref nnInfo);
			
			nnInfo.UpdateInfo ();
			
			return nnInfo;
		}
		
		/** Queries the tree for the best node, searching within a circle around \a p with the specified radius.
		  * Will fill in both the constrained node and the not constrained node in the NNInfo.
		  * 
		  * \see QueryClosest
		  */
		public NNInfo QueryCircle (Vector3 p, float radius, NNConstraint constraint) {
			BBTreeBox c = root;
			
			if (c == null) {
				return new NNInfo();
			}
			
#if ASTARDEBUG
			Vector3 prev = new Vector3 (1,0,0)*radius+p;
			for (double i=0;i< Math.PI*2; i += Math.PI/50.0) {
				Vector3 cpos = new Vector3 ((float)Math.Cos (i),0,(float)Math.Sin (i))*radius+p;
				Debug.DrawLine (prev,cpos,Color.yellow);
				prev = cpos;
			}
#endif
			
			NNInfo nnInfo = new NNInfo (null);
			
			SearchBoxCircle (c,p, radius, constraint, ref nnInfo);
			
			nnInfo.UpdateInfo ();
			
			return nnInfo;
		}
		
		/** Queries the tree for the closest node to \a p constrained by the NNConstraint.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * \see QueryCircle
		  */
		public NNInfo QueryClosest (Vector3 p, NNConstraint constraint, out float distance) {
			distance = float.PositiveInfinity;
			return QueryClosest (p, constraint, ref distance, new NNInfo (null));
		}
		
		/** Queries the tree for the closest node to \a p constrained by the NNConstraint trying to improve an existing solution.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * This search will start from the \a previous NNInfo and improve it if possible.
		  * Even if the search fails on this call, the solution will never be worse than \a previous.
		  * 
		  * This method will completely ignore any Y-axis differences in positions.
		  * 
		  * \param distance The best distance for the \a previous solution. Will be updated with the best distance
		  * after this search. Will be positive infinity if no node could be found.
		  * Set to positive infinity if there was no previous solution.
		  * 
		  * 
		  * \see QueryCircle
		  */
		public NNInfo QueryClosestXZ (Vector3 p, NNConstraint constraint, ref float distance, NNInfo previous) {
			BBTreeBox c = root;
			
			if (c == null) {
				return previous;
			}
			
			SearchBoxClosestXZ (c,p, ref distance, constraint, ref previous);
			
			return previous;
		}
		
		/** Queries the tree for the closest node to \a p constrained by the NNConstraint trying to improve an existing solution.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * This search will start from the \a previous NNInfo and improve it if possible.
		  * Even if the search fails on this call, the solution will never be worse than \a previous.
		  * 
		  * 
		  * \param distance The best distance for the \a previous solution. Will be updated with the best distance
		  * after this search. Will be positive infinity if no node could be found.
		  * Set to positive infinity if there was no previous solution.
		  * 
		  * 
		  * \see QueryCircle
		  */
		public NNInfo QueryClosest (Vector3 p, NNConstraint constraint, ref float distance, NNInfo previous) {
			BBTreeBox c = root;
			
			if (c == null) {
				return previous;
			}
			
			SearchBoxClosest (c,p, ref distance, constraint, ref previous);
			
			return previous;
		}
		
		void SearchBoxClosest (BBTreeBox box, Vector3 p, ref float closestDist, NNConstraint constraint, ref NNInfo nnInfo) {
			
			if (box.node != null) {
				//Leaf node
				if (NodeIntersectsCircle (box.node,p,closestDist)) {
					//Update the NNInfo
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
#endif
					
					Vector3 closest = box.node.ClosestPointOnNode (p);//NavMeshGraph.ClosestPointOnNode (box.node,graph.vertices,p);
					
					float dist = (closest-p).sqrMagnitude;
					
					if (constraint == null || constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
							closestDist = (float)System.Math.Sqrt (dist);
						} else if (dist < closestDist*closestDist) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
							closestDist = (float)System.Math.Sqrt (dist);
						}
					}
				} else {
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
#endif
				}
			} else {
				
	#if ASTARDEBUG
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
	#endif
				
				//Search children
				if (RectIntersectsCircle (box.c1.rect,p,closestDist)) {
					SearchBoxClosest (box.c1,p, ref closestDist, constraint, ref nnInfo);
				}
				
				if (RectIntersectsCircle (box.c2.rect,p,closestDist)) {
					SearchBoxClosest (box.c2,p, ref closestDist, constraint, ref nnInfo);
				}
			}
		}
		
		public MeshNode QueryInside (Vector3 p, NNConstraint constraint) {
			BBTreeBox c = root;
			
			if (c == null) {
				return null;
			}
			
			return SearchBoxInside (c,p, constraint);
		}
		
		MeshNode SearchBoxInside (BBTreeBox box, Vector3 p, NNConstraint constraint) {
			
			if (box.node != null) {
				if (box.node.ContainsPoint ((Int3)p)) {
					//Update the NNInfo
					
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
#endif
					
					
					if (constraint == null || constraint.Suitable (box.node)) {
						return box.node;
					}
				} else {
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
#endif
				}
			} else {
				
	#if ASTARDEBUG
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
	#endif
				
				//Search children
				MeshNode g;
				if (box.c1.rect.Contains (new Vector2(p.x,p.z))) {
					g = SearchBoxInside (box.c1,p, constraint);
					if (g != null) return g;
				}
				
				if (box.c2.rect.Contains (new Vector2(p.x,p.z))) {
					g = SearchBoxInside (box.c2, p, constraint);
					if (g != null) return g;
				}
			}
			
			return null;
		}

		void SearchBoxClosestXZ (BBTreeBox box, Vector3 p, ref float closestDist, NNConstraint constraint, ref NNInfo nnInfo) {
			
			if (box.node != null) {
				//Leaf node
				//if (NodeIntersectsCircle (box.node,p,closestDist)) {
					//Update the NNInfo
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
#endif
					
					Vector3 closest = box.node.ClosestPointOnNodeXZ (p);//NavMeshGraph.ClosestPointOnNode (box.node,graph.vertices,p);
					
					// XZ distance
					float dist = (closest.x-p.x)*(closest.x-p.x)+(closest.z-p.z)*(closest.z-p.z);
					
					if (constraint == null || constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
							closestDist = (float)System.Math.Sqrt (dist);
						} else if (dist < closestDist*closestDist) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
							closestDist = (float)System.Math.Sqrt (dist);
						}
					}
				//} else {
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
#endif
				//}
			} else {
				
	#if ASTARDEBUG
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
				Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
	#endif
				
				//Search children
				if (RectIntersectsCircle (box.c1.rect,p,closestDist)) {
					SearchBoxClosestXZ (box.c1,p, ref closestDist, constraint, ref nnInfo);
				}
				
				if (RectIntersectsCircle (box.c2.rect,p,closestDist)) {
					SearchBoxClosestXZ (box.c2,p, ref closestDist, constraint, ref nnInfo);
				}
			}
		}
		
		void SearchBoxCircle (BBTreeBox box, Vector3 p, float radius, NNConstraint constraint, ref NNInfo nnInfo) {//, int intendentLevel = 0) {
			
			if (box.node != null) {
				//Leaf node
				if (NodeIntersectsCircle (box.node,p,radius)) {
					//Update the NNInfo
					
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.red);
#endif
					
					Vector3 closest = box.node.ClosestPointOnNode (p);//NavMeshGraph.ClosestPointOnNode (box.node,graph.vertices,p);
					float dist = (closest-p).sqrMagnitude;
					
					if (nnInfo.node == null) {
						nnInfo.node = box.node;
						nnInfo.clampedPosition = closest;
					} else if (dist < (nnInfo.clampedPosition - p).sqrMagnitude) {
						nnInfo.node = box.node;
						nnInfo.clampedPosition = closest;
					}
					if (constraint == null || constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
						} else if (dist < (nnInfo.constClampedPosition - p).sqrMagnitude) {
							nnInfo.constrainedNode = box.node;
							nnInfo.constClampedPosition = closest;
						}
					}
				} else {
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(0),(Vector3)box.node.GetVertex(1),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(1),(Vector3)box.node.GetVertex(2),Color.blue);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2),(Vector3)box.node.GetVertex(0),Color.blue);
#endif
				}
				return;
			}
			
#if ASTARDEBUG
			Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMin),Color.white);
			Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMax),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
			Debug.DrawLine (new Vector3 (box.rect.xMin,0,box.rect.yMin),new Vector3 (box.rect.xMin,0,box.rect.yMax),Color.white);
			Debug.DrawLine (new Vector3 (box.rect.xMax,0,box.rect.yMin),new Vector3 (box.rect.xMax,0,box.rect.yMax),Color.white);
#endif
			
			//Search children
			if (RectIntersectsCircle (box.c1.rect,p,radius)) {
				SearchBoxCircle (box.c1,p, radius, constraint, ref nnInfo);
			}
			
			if (RectIntersectsCircle (box.c2.rect,p,radius)) {
				SearchBoxCircle (box.c2,p, radius, constraint, ref nnInfo);
			}
		}
		
		void SearchBox (BBTreeBox box, Vector3 p, NNConstraint constraint, ref NNInfo nnInfo) {//, int intendentLevel = 0) {
			
			if (box.node != null) {
				//Leaf node
				if (box.node.ContainsPoint ((Int3)p)) {
					//Update the NNInfo
					
					if (nnInfo.node == null) {
						nnInfo.node = box.node;
					} else if (Mathf.Abs(((Vector3)box.node.position).y - p.y) < Mathf.Abs (((Vector3)nnInfo.node.position).y - p.y)) {
						nnInfo.node = box.node;
					}
					if (constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
						} else if (Mathf.Abs(box.node.position.y - p.y) < Mathf.Abs (nnInfo.constrainedNode.position.y - p.y)) {
							nnInfo.constrainedNode = box.node;
						}
					}
				}
				return;
			}
			
			//Search children
			if (RectContains (box.c1.rect,p)) {
				SearchBox (box.c1,p, constraint, ref nnInfo);
			}
			
			if (RectContains (box.c2.rect,p)) {
				SearchBox (box.c2,p, constraint, ref nnInfo);
			}
		}
		
		/** Inserts a mesh node in the tree */
		public void Insert (MeshNode node) {
			BBTreeBox box = new BBTreeBox (this,node);
			
			if (root == null) {
				root = box;
				return;
			}
			
			BBTreeBox c = root;
			while (true) {
				
				c.rect = ExpandToContain (c.rect,box.rect);
				if (c.node != null) {
					//Is Leaf
					c.c1 = box;
					BBTreeBox box2 = new BBTreeBox (this,c.node);
					//Console.WriteLine ("Inserted "+box.node+", rect "+box.rect.ToString ());
					c.c2 = box2;
					
					
					c.node = null;
					//c.rect = c.rect.
					return;
				} else {
					float e1 = ExpansionRequired (c.c1.rect,box.rect);
					float e2 = ExpansionRequired (c.c2.rect,box.rect);
					
					//Choose the rect requiring the least expansion to contain box.rect
					if (e1 < e2) {
						c = c.c1;
					} else if (e2 < e1) {
						c = c.c2;
					} else {
						//Equal, Choose the one with the smallest area
						c = RectArea (c.c1.rect) < RectArea (c.c2.rect) ? c.c1 : c.c2;
					}
				}
			}
		}
		

		
		void OnDrawGizmos (BBTreeBox box) {
			if (box == null) {
				return;
			}
			
			Vector3 min = new Vector3 (box.rect.xMin,0,box.rect.yMin);
			Vector3 max = new Vector3 (box.rect.xMax,0,box.rect.yMax);
			
			Vector3 center = (min+max)*0.5F;
			Vector3 size = (max-center)*2;
			
			Gizmos.DrawCube (center,size);
			
			OnDrawGizmos (box.c1);
			OnDrawGizmos (box.c2);
		}

		public void OnDrawGizmos () {
			//Gizmos.color = new Color (1,1,1,0.01F);
			//OnDrawGizmos (root);
		}

#endif


		/*static void TestIntersections (BBTreeBox box, Vector3 p, float radius) {
			
			if (box == null) {
				return;
			}
			
			RectIntersectsCircle (box.rect,p,radius);
			
			TestIntersections (box.c1,p,radius);
			TestIntersections (box.c2,p,radius);
		}*/

		static bool NodeIntersectsCircle (MeshNode node, Vector3 p, float radius) {
			
			if (float.IsPositiveInfinity(radius)) return true;

			/** \bug Is not correct on the Y axis */
			/*if (node.ContainsPoint ((Int3)p)) {
				return true;
			}*/
			return (p - node.ClosestPointOnNode (p)).sqrMagnitude < radius*radius;
			
			/*Int3[] vertices = graph.vertices;
			Vector3 p1 = (Vector3)vertices[node[0]], p2 = (Vector3)vertices[node[1]], p3 = (Vector3)vertices[node[2]];
			
			float r2 = radius*radius;
			p1.y = p.y;
			p2.y = p.y;
			p3.y = p.y;
			
			return 	Mathfx.DistancePointSegmentStrict (p1,p2,p) < r2 ||
					Mathfx.DistancePointSegmentStrict (p2,p3,p) < r2 ||
					Mathfx.DistancePointSegmentStrict (p3,p1,p) < r2;*/
			
		}

		/** Returns true if \a p is within \a radius from \a r.
		 * Correctly handles cases where \a radius is positive infinity.
		  */
		static bool RectIntersectsCircle (Rect r, Vector3 p, float radius) {
			
			if (float.IsPositiveInfinity(radius)) return true;
			
			Vector3 po = p;
			p.x = System.Math.Max (p.x, r.xMin);
			p.x = System.Math.Min (p.x, r.xMax);
			p.z = System.Math.Max (p.z, r.yMin);
			p.z = System.Math.Min (p.z, r.yMax);

			// XZ squared magnitude comparison
			return (p.x-po.x)*(p.x-po.x) + (p.z-po.z)*(p.z-po.z) < radius*radius;
		}
		
		/** Returns if a rect contains the 3D point in XZ space */
		static bool RectContains (Rect r, Vector3 p) {
			return p.x >= r.xMin && p.x <= r.xMax && p.z >= r.yMin && p.z <= r.yMax;
		}
		
		/** Returns the difference in area between \a r and \a r expanded to contain \a r2 */
		static float ExpansionRequired (Rect r, Rect r2) {
			float xMin = System.Math.Min (r.xMin,r2.xMin);
			float xMax = System.Math.Max (r.xMax,r2.xMax);
			float yMin = System.Math.Min (r.yMin,r2.yMin);
			float yMax = System.Math.Max (r.yMax,r2.yMax);
			
			return (xMax-xMin)*(yMax-yMin)-RectArea (r);
		}
		
		/** Returns a new rect which contains both \a r and \a r2 */
		static Rect ExpandToContain (Rect r, Rect r2) {
			float xMin = System.Math.Min (r.xMin,r2.xMin);
			float xMax = System.Math.Max (r.xMax,r2.xMax);
			float yMin = System.Math.Min (r.yMin,r2.yMin);
			float yMax = System.Math.Max (r.yMax,r2.yMax);
			
			return Rect.MinMaxRect (xMin,yMin,xMax,yMax);
		}
		
		/** Returns the area of a rect */
		static float RectArea (Rect r) {
			return r.width*r.height;
		}

#if ASTAR_OLD_BBTREE
		public new void ToString () {
			Console.WriteLine ("Root "+(root.node != null ? root.node.ToString () : ""));
			
			BBTreeBox c = root;
			
			Stack<BBTreeBox> stack = new Stack<BBTreeBox>();
			stack.Push (c);
			
			c.WriteChildren (0);
		}
#else
#endif
	}
	
#if ASTAR_OLD_BBTREE
	class BBTreeBox {
		public Rect rect;
		public MeshNode node;
		
		public BBTreeBox c1;
		public BBTreeBox c2;
		
		public BBTreeBox (BBTree tree, MeshNode node) {
			this.node = node;
			Vector3 first = (Vector3)node.GetVertex(0);
			Vector2 min = new Vector2(first.x,first.z);
			Vector2 max = min;
			
			for (int i=1;i<node.GetVertexCount();i++) {
				Vector3 p = (Vector3)node.GetVertex(i);
				min.x = Mathf.Min (min.x,p.x);
				min.y = Mathf.Min (min.y,p.z);
				
				max.x = Mathf.Max (max.x,p.x);
				max.y = Mathf.Max (max.y,p.z);
			}
			
			rect = Rect.MinMaxRect (min.x,min.y,max.x,max.y);
		}
		
		public bool Contains (Vector3 p) {
			return rect.Contains (p);
		}
		
		public void WriteChildren (int level) {
			for (int i=0;i<level;i++) {
				Console.Write ("  ");
			}
			if (node != null) {
				Console.WriteLine ("Leaf ");//+triangle.ToString ());
			} else {
				Console.WriteLine ("Box ");//+rect.ToString ());
				c1.WriteChildren (level+1);
				c2.WriteChildren (level+1);
			}
		}
	}
#endif
	
}