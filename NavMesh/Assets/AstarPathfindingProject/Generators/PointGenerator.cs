using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.Serialization.JsonFx;
using Pathfinding.Serialization;

namespace Pathfinding {
	
	/** Basic point graph.
	 * \ingroup graphs
	  * The List graph is the most basic graph structure, it consists of a number of interconnected points in space, waypoints or nodes.\n
	  * The list graph takes a Transform object as "root", this Transform will be searched for child objects, every child object will be treated as a node.
	  * It will then check if any connections between the nodes can be made, first it will check if the distance between the nodes isn't too large ( #maxDistance )
	  * and then it will check if the axis aligned distance isn't too high. The axis aligned distance, named #limits,
	  * is useful because usually an AI cannot climb very high, but linking nodes far away from each other,
	  * but on the same Y level should still be possible. #limits and #maxDistance won't affect anything if the values are 0 (zero) though. \n
	  * Lastly it will check if there are any obstructions between the nodes using
	  * <a href="http://unity3d.com/support/documentation/ScriptReference/Physics.Raycast.html">raycasting</a> which can optionally be thick.\n
	  * One thing to think about when using raycasting is to either place the nodes a small
	  * distance above the ground in your scene or to make sure that the ground is not in the raycast \a mask to avoid the raycast from hitting the ground.\n
	  * \note Does not support linecast because of obvious reasons.
	  * 
\shadowimage{pointgraph_graph.png}
\shadowimage{pointgraph_inspector.png}

	  */
	[JsonOptIn]
	public class PointGraph : NavGraph
	,IUpdatableGraph
	{
		
		[JsonMember]
		/** Childs of this transform are treated as nodes */
		public Transform root;
		
		[JsonMember]
		/** If no #root is set, all nodes with the tag is used as nodes */
		public string searchTag;
		
		[JsonMember]
		/** Max distance for a connection to be valid.
		 * The value 0 (zero) will be read as infinity and thus all nodes not restricted by
		 * other constraints will be added as connections.
		 * 
		 * A negative value will disable any neighbours to be added.
		 * It will completely stop the connection processing to be done, so it can save you processing
		 * power if you don't these connections.
		 */
		public float maxDistance = 0;
		
		[JsonMember]
		/** Max distance along the axis for a connection to be valid. 0 = infinity */
		public Vector3 limits;
		
		[JsonMember]
		/** Use raycasts to check connections */
		public bool raycast = true;
		
		[JsonMember]
		/** Use thick raycast */
		public bool thickRaycast = false;
		
		[JsonMember]
		/** Thick raycast radius */
		public float thickRaycastRadius = 1;
		
		[JsonMember]
		/** Recursively search for childnodes to the #root */
		public bool recursive = true;
		
		[JsonMember]
		public bool autoLinkNodes = true;
		
		[JsonMember]
		/** Layer mask to use for raycast */
		public LayerMask mask;
		
		[JsonMember]
		/** Optimizes the graph for sparse graphs.
		 * 
		 * This can reduce calculation times for both scanning and for normal path requests by huge amounts.
		 * 
		 * You should enable this when your #maxDistance and/or #limits variables are set relatively low compared to the world
		 * size. It reduces the number of node-node checks that need to be done during scan, and can also optimize getting the nearest node from the graph (such as when querying for a path).
		 * 
		 * Try enabling and disabling this option, check the scan times logged when you scan the graph to see if your graph is suited for this optimization
		 * or if it makes it slower.
		 * 
		 * The gain of using this optimization increases with larger graphs, the default scan algorithm is brute force an requires O(n^2) checks, this optimization
		 * along with a graph suited for it, requires only O(n) checks during scan.
		 * 
		 * \note
		 * When you have this enabled, you will not be able to move nodes around using scripting unless you recalculate the lookup structure at the same time.
		 * \see RebuildNodeLookup
		 * 
		 * \astarpro
		 */
		public bool optimizeForSparseGraph = false;

		[JsonMember]
		/** Optimizes for when the graph is mostly spread out in the XZ plane.
		 * Requires #optimizeForSparseGraph.
		 * 
		 * When your graph is mostly spread out in the XZ plane instead of equally in all directions,
		 * you can tick this toggle to speed up some calculations.
		 * If you do not have a graph which is spread out mostly on the XZ plane, enabling this option
		 * will most likely degrade the performance of the graph instead of improve it.
		 * 
		 * \astarpro
		 */
		public bool optimizeFor2D = false;
		
		private static readonly Int3[] ThreeDNeighbours = new Int3[] {
			new Int3 ( -1,  0, -1),
			new Int3 (  0,  0, -1),
			new Int3 (  1,  0, -1),
			
			new Int3 ( -1,  0,  0),
			new Int3 (  0,  0,  0),
			new Int3 (  1,  0,  0),
			
			new Int3 ( -1,  0,  1),
			new Int3 (  0,  0,  1),
			new Int3 (  1,  0,  1),




			new Int3 ( -1, -1, -1),
			new Int3 (  0, -1, -1),
			new Int3 (  1, -1, -1),

			new Int3 ( -1, -1,  0),
			new Int3 (  0, -1,  0),
			new Int3 (  1, -1,  0),

			new Int3 ( -1, -1,  1),
			new Int3 (  0, -1,  1),
			new Int3 (  1, -1,  1),



			new Int3 ( -1,  1, -1),
			new Int3 (  0,  1, -1),
			new Int3 (  1,  1, -1),
			
			new Int3 ( -1,  1,  0),
			new Int3 (  0,  1,  0),
			new Int3 (  1,  1,  0),
			
			new Int3 ( -1,  1,  1),
			new Int3 (  0,  1,  1),
			new Int3 (  1,  1,  1),
		};

		Dictionary<Int3, PointNode> nodeLookup;
		Int3 minLookup, maxLookup;
		Int3 lookupCellSize;
		
		/** GameObjects which defined the node in the #nodes array.
		 * Entries are permitted to be null in case no GameObject was used to define a node.
		 * 
		 * \note Set, but not used at the moment. Does not work after deserialization.
		 */
		GameObject[] nodeGameObjects;
		
		/** All nodes in this graph.
		 * Note that only the first #nodeCount will be non-null.
		 * 
		 * You can also use the GetNodes method to get all nodes.
		 */
		public PointNode[] nodes;
		
		/** Number of nodes in this graph.
		 * 
		 * \warning Do not edit directly
		 */
		public int nodeCount;
		
		Int3 WorldToLookupSpace ( Int3 p ) {
			Int3 lp = Int3.zero;

			lp.x = lookupCellSize.x != 0 ? p.x/lookupCellSize.x : 0;
			lp.y = lookupCellSize.y != 0 ? p.y/lookupCellSize.y : 0;
			lp.z = lookupCellSize.z != 0 ? p.z/lookupCellSize.z : 0;

			return lp;
		}
		
		public override void GetNodes (GraphNodeDelegateCancelable del) {
			if (nodes == null) return;
			for (int i=0;i<nodeCount && del (nodes[i]);i++) {}
		}
		
		public override NNInfo GetNearest (Vector3 position, NNConstraint constraint, GraphNode hint) {
			return GetNearestForce (position, constraint);
		}

		public override NNInfo GetNearestForce (Vector3 position, NNConstraint constraint)
		{
			//Debug.LogError ("This function (GetNearest) is not implemented in the navigation graph generator : Type "+this.GetType ().Name);
			
			if (nodes == null) return new NNInfo();
			
			float maxDistSqr = constraint.constrainDistance ? AstarPath.active.maxNearestNodeDistanceSqr : float.PositiveInfinity;
			
			float minDist = float.PositiveInfinity;
			GraphNode minNode = null;
			
			float minConstDist = float.PositiveInfinity;
			GraphNode minConstNode = null;
			
			if ( optimizeForSparseGraph ) {

				Int3 lookupStart = WorldToLookupSpace ( (Int3)position );

				Int3 size = lookupStart-minLookup;

				int mw = 0;
				mw = System.Math.Max (mw, System.Math.Abs (size.x) );
				mw = System.Math.Max (mw, System.Math.Abs (size.y) );
				mw = System.Math.Max (mw, System.Math.Abs (size.z) );

				size = lookupStart-maxLookup;
				mw = System.Math.Max (mw, System.Math.Abs (size.x) );
				mw = System.Math.Max (mw, System.Math.Abs (size.y) );
				mw = System.Math.Max (mw, System.Math.Abs (size.z) );

				PointNode node = null;

				if ( nodeLookup.TryGetValue (lookupStart, out node) ) {
					while ( node != null ) {
						float dist = (position-(Vector3)node.position).sqrMagnitude;
						if (dist < minDist) { minDist = dist; minNode = node; }
						if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
						
						node = node.next;
					}
				}

				for ( int w = 1; w <= mw; w++ ) {

					if ( w >= 20 ) {
						Debug.LogWarning ("Aborting GetNearest call at maximum distance because it has iterated too many times.\n" +
						                  "If you get this regularly, check your settings for PointGraph -> <b>Optimize For Sparse Graph</b> and " +
						                  "PointGraph -> <b>Optimize For 2D</b>.\nThis happens when the closest node was very far away (20*link distance between nodes). " +
						                  "When optimizing for sparse graphs, getting the nearest node from far away positions is <b>very slow</b>.\n");
						break;
					}

					if ( lookupCellSize.y == 0 ) {

						Int3 reference = lookupStart + new Int3(-w,0,-w);

						for ( int x=0; x <= 2*w; x++ ) {
							if ( nodeLookup.TryGetValue (reference + new Int3 ( x, 0, 0 ), out node) ) {
								while ( node != null ) {
									float dist = (position-(Vector3)node.position).sqrMagnitude;
									if (dist < minDist) { minDist = dist; minNode = node; }
									if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }

									node = node.next;
								}
							}
							if ( nodeLookup.TryGetValue (reference + new Int3 ( x, 0, 2*w ), out node) ) {
								while ( node != null ) {
									float dist = (position-(Vector3)node.position).sqrMagnitude;
									if (dist < minDist) { minDist = dist; minNode = node; }
									if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
									
									node = node.next;
								}
							}
						}

						for ( int x=1; x <  2*w; x++ ) {
							if ( nodeLookup.TryGetValue (reference + new Int3 ( 0, 0, x ), out node) ) {
								while ( node != null ) {
									float dist = (position-(Vector3)node.position).sqrMagnitude;
									if (dist < minDist) { minDist = dist; minNode = node; }
									if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
									
									node = node.next;
								}
							}
							if ( nodeLookup.TryGetValue (reference + new Int3 ( 2*w, 0, x ), out node) ) {
								while ( node != null ) {
									float dist = (position-(Vector3)node.position).sqrMagnitude;
									if (dist < minDist) { minDist = dist; minNode = node; }
									if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
									
									node = node.next;
								}
							}
						}
					} else {

						Int3 reference = lookupStart + new Int3(-w,-w,-w);

						for ( int x=0; x <= 2*w; x++ ) {
							for ( int y=0; y <= 2*w; y++ ) {
								if ( nodeLookup.TryGetValue (reference + new Int3 ( x, y, 0 ), out node) ) {
									while ( node != null ) {
										float dist = (position-(Vector3)node.position).sqrMagnitude;
										if (dist < minDist) { minDist = dist; minNode = node; }
										if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
										
										node = node.next;
									}
								}
								if ( nodeLookup.TryGetValue (reference + new Int3 ( x, y, 2*w ), out node) ) {
									while ( node != null ) {
										float dist = (position-(Vector3)node.position).sqrMagnitude;
										if (dist < minDist) { minDist = dist; minNode = node; }
										if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
										
										node = node.next;
									}
								}
							}
						}
						
						for ( int x=1; x <  2*w; x++ ) {
							for ( int y=0; y <= 2*w; y++ ) {
								if ( nodeLookup.TryGetValue (reference + new Int3 ( 0, y, x ), out node) ) {
									while ( node != null ) {
										float dist = (position-(Vector3)node.position).sqrMagnitude;
										if (dist < minDist) { minDist = dist; minNode = node; }
										if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
										
										node = node.next;
									}
								}
								if ( nodeLookup.TryGetValue (reference + new Int3 ( 2*w, y, x ), out node) ) {
									while ( node != null ) {
										float dist = (position-(Vector3)node.position).sqrMagnitude;
										if (dist < minDist) { minDist = dist; minNode = node; }
										if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
										
										node = node.next;
									}
								}
							}
						}

						for ( int x=1; x <  2*w; x++ ) {
							for ( int y=1; y <  2*w; y++ ) {
								if ( nodeLookup.TryGetValue (reference + new Int3 ( x, 0, y ), out node) ) {
									while ( node != null ) {
										float dist = (position-(Vector3)node.position).sqrMagnitude;
										if (dist < minDist) { minDist = dist; minNode = node; }
										if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
										
										node = node.next;
									}
								}
								if ( nodeLookup.TryGetValue (reference + new Int3 ( x, 2*w, y ), out node) ) {
									while ( node != null ) {
										float dist = (position-(Vector3)node.position).sqrMagnitude;
										if (dist < minDist) { minDist = dist; minNode = node; }
										if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) { minConstDist = dist; minConstNode = node; }
										
										node = node.next;
									}
								}
							}
						}
					}

					if ( minConstNode != null ) {
						// Only search one more layer
						mw = System.Math.Min (mw, w+1);
					}
				}

			} else {
				for (int i=0;i<nodeCount;i++) {
					PointNode node = nodes[i];
					float dist = (position-(Vector3)node.position).sqrMagnitude;
					
					if (dist < minDist) {
						minDist = dist;
						minNode = node;
					}
					
					if (constraint == null || (dist < minConstDist && dist < maxDistSqr && constraint.Suitable (node))) {
						minConstDist = dist;
						minConstNode = node;
					}
				}
			}
			
			NNInfo nnInfo = new NNInfo (minNode);
			
			nnInfo.constrainedNode = minConstNode;
			
			if (minConstNode != null) {
				nnInfo.constClampedPosition = (Vector3)minConstNode.position;
			} else if (minNode != null) {
				nnInfo.constrainedNode = minNode;
				nnInfo.constClampedPosition = (Vector3)minNode.position;
			}
			
			return nnInfo;
		}

		/** Add a node to the graph at the specified position.
		 * \note Vector3 can be casted to Int3 using (Int3)myVector.
		 */
		public PointNode AddNode (Int3 position) {
			return AddNode ( new PointNode (active), position );
		}
		
		/** Add a node with the specified type to the graph at the specified position.
		 * \note Vector3 can be casted to Int3 using (Int3)myVector.
		 * 
		 * \param nd This must be a node created using T(AstarPath.active) right before the call to this method.
		 * The node parameter is only there because there is no new(AstarPath) constraint on
		 * generic type parameters.
		 */
		public T AddNode<T> (T nd, Int3 position) where T : PointNode {
			
			if ( nodes == null || nodeCount == nodes.Length ) {
				PointNode[] nds = new PointNode[nodes != null ? System.Math.Max (nodes.Length+4, nodes.Length*2) : 4];
				for ( int i = 0; i < nodeCount; i++ ) nds[i] = nodes[i];
				nodes = nds;
			}
			//T nd = new T( active );//new PointNode ( active );
			nd.SetPosition (position);
			nd.GraphIndex = graphIndex;
			nd.Walkable = true;
			
			nodes[nodeCount] = nd;
			nodeCount++;

			AddToLookup ( nd );

			return nd;
		}
		
		/** Recursively counds children of a transform */
		public static int CountChildren (Transform tr) {
			int c = 0;
			foreach (Transform child in tr) {
				c++;
				c+= CountChildren (child);
			}
			return c;
		}
		
		/** Recursively adds childrens of a transform as nodes */
		public void AddChildren (ref int c, Transform tr) {
			foreach (Transform child in tr) {
				(nodes[c] as PointNode).SetPosition ((Int3)child.position);
				nodes[c].Walkable = true;
				
				nodeGameObjects[c] = child.gameObject;
				
#if !AstarRelease && FALSE
				NodeObject nobj = child.GetComponent<NodeObject>();
				if (nobj != null) nodes[c].Walkable = nobj.walkable;
#endif
				
				c++;
				AddChildren (ref c,child);
			}
		}

		/** Rebuilds the lookup structure for nodes.
		 * 
		 * This is used when #optimizeForSparseGraph is enabled.
		 * 
		 * You should call this method every time you move a node in the graph manually and
		 * you are using #optimizeForSparseGraph, otherwise pathfinding might not work correctly.
		 * 
		 * \astarpro
		 */
		public void RebuildNodeLookup () {
			if ( !optimizeForSparseGraph ) return;

			if ( maxDistance == 0 ) {
				lookupCellSize = (Int3)limits;
			} else {
				lookupCellSize.x = Mathf.CeilToInt(Int3.Precision*(limits.x != 0 ? Mathf.Min(maxDistance, limits.x) : maxDistance));
				lookupCellSize.y = Mathf.CeilToInt(Int3.Precision*(limits.y != 0 ? Mathf.Min(maxDistance, limits.y) : maxDistance));
				lookupCellSize.z = Mathf.CeilToInt(Int3.Precision*(limits.z != 0 ? Mathf.Min(maxDistance, limits.z) : maxDistance));
			}

			if ( optimizeFor2D ) {
				lookupCellSize.y = 0;
			}

			if ( nodeLookup == null ) nodeLookup = new Dictionary<Int3, PointNode>();

			nodeLookup.Clear ();
			
			for ( int i = 0; i < nodeCount; i++ ) {
				PointNode node = nodes[i];
				AddToLookup ( node );
			}
		}

		public void AddToLookup ( PointNode node ) {
			if ( nodeLookup == null ) return;

			Int3 p = WorldToLookupSpace ( node.position );
			
			if ( nodeLookup.Count == 0 ) {
				minLookup = p;
				maxLookup = p;
			} else {
				minLookup = new Int3(System.Math.Min (minLookup.x,p.x),System.Math.Min (minLookup.y,p.y),System.Math.Min (minLookup.z,p.z));
				maxLookup = new Int3(System.Math.Max (minLookup.x,p.x),System.Math.Max (minLookup.y,p.y),System.Math.Max (minLookup.z,p.z));
			}
			
			// Does not cover all cases, but at least some of them
			if ( node.next != null ) throw new System.Exception ( "This node has already been added to the lookup structure" );

			PointNode root;
			if (nodeLookup.TryGetValue ( p, out root )){
				// Insert in between
				node.next = root.next;
				root.next = node;
			} else {
				nodeLookup[p] = node;
			}
		}

		public override void ScanInternal (OnScanStatus statusCallback) {

			if (root == null) {
				//If there is no root object, try to find nodes with the specified tag instead
				GameObject[] gos = GameObject.FindGameObjectsWithTag (searchTag);
				nodeGameObjects = gos;
				
				if (gos == null) {
					nodes = new PointNode[0];
					nodeCount = 0;
					return;
				}
				
				//Create and set up the found nodes
				nodes = new PointNode[gos.Length];
				nodeCount = nodes.Length;

				for (int i=0;i<nodes.Length;i++) nodes[i] = new PointNode(active);
				
				//CreateNodes (gos.Length);
				for (int i=0;i<gos.Length;i++) {
					(nodes[i] as PointNode).SetPosition ((Int3)gos[i].transform.position);
					nodes[i].Walkable = true;
					
#if !AstarRelease && FALSE
					NodeObject nobj = gos[i].GetComponent<NodeObject>();
					if (nobj != null) nodes[i].Walkable = nobj.walkable;
#endif
					
				}
			} else {
				
				//Search the root for children and create nodes for them
				if (!recursive) {
					nodes = new PointNode[root.childCount];
					nodeCount = nodes.Length;

					for (int i=0;i<nodes.Length;i++) nodes[i] = new PointNode(active);
					
					nodeGameObjects = new GameObject[nodes.Length];
					
					int c = 0;
					foreach (Transform child in root) {
						(nodes[c] as PointNode).SetPosition ((Int3)child.position);
						nodes[c].Walkable = true;
						
						nodeGameObjects[c] = child.gameObject;
						
#if !AstarRelease && FALSE
						NodeObject nobj = child.GetComponent<NodeObject>();
						if (nobj != null) nodes[c].Walkable = nobj.walkable;
#endif
						
						c++;
					}
				} else {
					nodes = new PointNode[CountChildren(root)];
					nodeCount = nodes.Length;

					for (int i=0;i<nodes.Length;i++) nodes[i] = new PointNode(active);
						//CreateNodes (CountChildren (root));
					nodeGameObjects = new GameObject[nodes.Length];
					
					int startID = 0;
					AddChildren (ref startID,root);
				}
			}
			
			if ( optimizeForSparseGraph ) {
				RebuildNodeLookup ();
			}

			if (maxDistance >= 0) {
				//To avoid too many allocations, these lists are reused for each node
				List<PointNode> connections = new List<PointNode>(3);
				List<uint> costs = new List<uint>(3);

				//Loop through all nodes and add connections to other nodes
				for (int i=0;i<nodes.Length;i++) {
					
					connections.Clear ();
					costs.Clear ();
					
					PointNode node = nodes[i];
					

					if ( optimizeForSparseGraph ) {

						Int3 p = WorldToLookupSpace ( node.position );

						int l = lookupCellSize.y == 0 ? 9 : ThreeDNeighbours.Length;

						for ( int j = 0; j < l; j++ ) {
							Int3 np = p + ThreeDNeighbours[j];

							PointNode other;
							if ( nodeLookup.TryGetValue ( np, out other ) ) {

								while ( other != null ) {

									float dist = 0;
									if (IsValidConnection (node,other,out dist)) {
										connections.Add (other);
										/** \todo Is this equal to .costMagnitude */
										costs.Add ((uint)Mathf.RoundToInt (dist*Int3.FloatPrecision));
									}

									other = other.next;
								}
							}
						}

					} else {
						// Only brute force is available in the free version
						for (int j=0;j<nodes.Length;j++) {
							if (i == j) continue;
								
							PointNode other = nodes[j];
							
							float dist = 0;
							if (IsValidConnection (node,other,out dist)) {
								connections.Add (other);
								/** \todo Is this equal to .costMagnitude */
								costs.Add ((uint)Mathf.RoundToInt (dist*Int3.FloatPrecision));
							}
						}
					}
					node.connections = connections.ToArray();
					node.connectionCosts = costs.ToArray();
				}
			}

			//GC can clear this up now.
			nodeGameObjects = null;
		}
		
		/** Returns if the connection between \a a and \a b is valid.
		 * Checks for obstructions using raycasts (if enabled) and checks for height differences.\n
		 * As a bonus, it outputs the distance between the nodes too if the connection is valid */
		public virtual bool IsValidConnection (GraphNode a, GraphNode b, out float dist) {
			dist = 0;
			
			if (!a.Walkable || !b.Walkable) return false;
			
			Vector3 dir = (Vector3)(a.position-b.position);
			
			if (
				(!Mathf.Approximately (limits.x,0) && Mathf.Abs (dir.x) > limits.x) ||
				(!Mathf.Approximately (limits.y,0) && Mathf.Abs (dir.y) > limits.y) ||
				(!Mathf.Approximately (limits.z,0) && Mathf.Abs (dir.z) > limits.z))
			{
				return false;
			}
			
			dist = dir.magnitude;
			if (maxDistance == 0 || dist < maxDistance) {
				
				if (raycast) {
					
					Ray ray = new Ray ((Vector3)a.position,(Vector3)(b.position-a.position));
					Ray invertRay = new Ray ((Vector3)b.position,(Vector3)(a.position-b.position));
					
					if (thickRaycast) {
						if (!Physics.SphereCast (ray,thickRaycastRadius,dist,mask) && !Physics.SphereCast (invertRay,thickRaycastRadius,dist,mask)) {
							return true;
						}
					} else {
						if (!Physics.Raycast (ray,dist,mask) && !Physics.Raycast (invertRay,dist,mask)) {
							return true;
						}
					}
				} else {
					return true;
				}
			}
			return false;
		}
		
		
		public GraphUpdateThreading CanUpdateAsync (GraphUpdateObject o) {
			return GraphUpdateThreading.UnityThread;
		}
		
		public void UpdateAreaInit (GraphUpdateObject o) {}
		
		/** Updates an area in the list graph.
		 * Recalculates possibly affected connections, i.e all connectionlines passing trough the bounds of the \a guo will be recalculated
		 * \astarpro */
		public void UpdateArea (GraphUpdateObject guo) {
			
			if (nodes == null) {
				return;
			}
			
			for (int i=0;i<nodeCount;i++) {
				if (guo.bounds.Contains ((Vector3)nodes[i].position)) {
					guo.WillUpdateNode (nodes[i]);
					guo.Apply (nodes[i]);
				}
			}
			
			if (guo.updatePhysics) {
				
				//Use a copy of the bounding box, we should not change the GUO's bounding box since it might be used for other graph updates
				Bounds bounds = guo.bounds;
				
				if (thickRaycast) {
					//Expand the bounding box to account for the thick raycast
					bounds.Expand (thickRaycastRadius*2);
				}
				
				//Create two temporary arrays used for holding new connections and costs
				List<GraphNode> tmp_arr = Pathfinding.Util.ListPool<GraphNode>.Claim ();
				List<uint>  tmp_arr2 = Pathfinding.Util.ListPool<uint>.Claim ();

				for (int i=0;i<nodeCount;i++) {
					PointNode node = nodes[i] as PointNode;
					Vector3 a = (Vector3)node.position;
					
					List<GraphNode> conn = null;
					List<uint> costs = null;
					
					for (int j=0;j<nodeCount;j++) {
						if (j==i) continue;
						
						Vector3 b = (Vector3)nodes[j].position;
						if (Polygon.LineIntersectsBounds (bounds,a,b)) {
							
							float dist;
							PointNode other = nodes[j] as PointNode;
							bool contains = node.ContainsConnection (other);
							
							//Note, the IsValidConnection test will actually only be done once
							//no matter what,so there is no performance penalty there
							if (!contains && IsValidConnection (node,other, out dist)) {
								//Debug.DrawLine (a+Vector3.up*0.1F,b+Vector3.up*0.1F,Color.green);
								if (conn == null) {
									tmp_arr.Clear();
									tmp_arr2.Clear ();
									conn = tmp_arr;
									costs = tmp_arr2;
									conn.AddRange (node.connections);
									costs.AddRange (node.connectionCosts);
								}
								
								uint cost = (uint)Mathf.RoundToInt (dist*Int3.FloatPrecision);
								conn.Add (other);
								costs.Add (cost);
								
							} else if (contains && !IsValidConnection (node,other, out dist)) {
								//Debug.DrawLine (a+Vector3.up*0.5F*Random.value,b+Vector3.up*0.5F*Random.value,Color.red);
								if (conn == null) {
									tmp_arr.Clear();
									tmp_arr2.Clear ();
									conn = tmp_arr;
									costs = tmp_arr2;
									conn.AddRange (node.connections);
									costs.AddRange (node.connectionCosts);
								}
								
								int p = conn.IndexOf (other);
								
								//Shouldn't have to check for it, but who knows what might go wrong
								if (p != -1) {
									conn.RemoveAt (p);
									costs.RemoveAt (p);
								}
							}
						}
					}
					
					if (conn != null) {
						node.connections = conn.ToArray ();
						node.connectionCosts = costs.ToArray ();
					}
				}

				Pathfinding.Util.ListPool<GraphNode>.Release (tmp_arr);
				Pathfinding.Util.ListPool<uint>.Release (tmp_arr2);
			}
		}

		public override void PostDeserialization ()
		{
			RebuildNodeLookup ();
		}

		public override void RelocateNodes (Matrix4x4 oldMatrix, Matrix4x4 newMatrix)
		{
			base.RelocateNodes (oldMatrix, newMatrix);
			RebuildNodeLookup ();
		}

		public override void SerializeExtraInfo (GraphSerializationContext ctx)
		{
			if (nodes == null) ctx.writer.Write (-1);
			ctx.writer.Write (nodeCount);
			for (int i=0;i<nodeCount;i++) {
				if (nodes[i] == null) ctx.writer.Write (-1);
				else {
					ctx.writer.Write (0);
					nodes[i].SerializeNode(ctx);
				}
			}
		}
		
		public override void DeserializeExtraInfo (GraphSerializationContext ctx) {
			
			int count = ctx.reader.ReadInt32();
			if (count == -1) {
				nodes = null;
				return;
			}
			
			nodes = new PointNode[count];
			nodeCount = count;

			for (int i=0;i<nodes.Length;i++) {
				if (ctx.reader.ReadInt32() == -1) continue;
				nodes[i] = new PointNode(active);
				nodes[i].DeserializeNode(ctx);
			}
		}
			
	}
}