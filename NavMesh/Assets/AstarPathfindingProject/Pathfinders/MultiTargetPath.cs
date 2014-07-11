using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;

namespace Pathfinding {
	/** A path which searches from one point to a number of different targets in one search or from a number of different start points to a single target.
	 * \ingroup paths
	  * \astarpro
	  * \see Seeker.StartMultiTargetPath
	  * \see \ref MultiTargetPathExample.cs "Example of how to use multi-target-paths" */
	public class MultiTargetPath : ABPath {
		
		//public Node startNode;
		
		public OnPathDelegate[] callbacks; /**< Callbacks to call for each individual path */
		
		public GraphNode[] targetNodes; /**< Nearest nodes to the #targetPoints */
		protected int targetNodeCount;
		
		public bool[] targetsFound; /**< Indicates if the target has been found. Also true if the target cannot be reached (is in another area) */
		public Vector3[] targetPoints; /**< Target points specified when creating the path. These are snapped to the nearest nodes */
		public Vector3[] originalTargetPoints; /**< Target points specified when creating the path. These are not snapped to the nearest nodes */
		public List<Vector3>[] vectorPaths; /**< Stores all vector paths to the targets. Elements are null if no path was found */
		public List<GraphNode>[] nodePaths; /**< Stores all paths to the targets. Elements are null if no path was found */
		
		public int endsFound = 0;
		
		public bool pathsForAll = true; /**< If true, a path to all targets will be returned, otherwise just the one to the closest one */
		public int  chosenTarget = -1;   /**< The closest target (if any was found) when #pathsForAll is false */
		public int sequentialTarget = 0; /** Current target for Sequential #heuristicMode. Refers to an item in the targetPoints array */
		
		/** How to calculate the heuristic. The \link #hTarget heuristic target point \endlink can be calculated in different ways, by taking the Average position of all targets, or taking the mid point of them (i.e center of the AABB encapsulating all targets).\n
		 * The one which works best seems to be Sequential though, it sets #hTarget to the first target, and when that target is found, it moves on to the next one.\n
		 * Some modes have the option to be 'moving' (e.g 'MovingAverage'), that means that it is updated every time a target is found.\n
		 * The H score is calculated according to AstarPath.heuristic */
		public HeuristicMode heuristicMode = HeuristicMode.Sequential;
		
		public enum HeuristicMode {
			None,
			Average,
			MovingAverage,
			Midpoint,
			MovingMidpoint,
			Sequential
		}
		
		/** False if the path goes from one point to multiple targets. True if it goes from multiple start points to one target point */
		public bool inverted = true;
		
		public MultiTargetPath () {}
		
		[System.ObsoleteAttribute ("Please use the Construct method instead")]
		public MultiTargetPath (Vector3[] startPoints, Vector3 target, OnPathDelegate[] callbackDelegates, OnPathDelegate callbackDelegate = null) : this (target,startPoints,callbackDelegates, callbackDelegate) {
			inverted = true;
		}
		
		[System.ObsoleteAttribute ("Please use the Construct method instead")]
		public MultiTargetPath (Vector3 start, Vector3[] targets, OnPathDelegate[] callbackDelegates, OnPathDelegate callbackDelegate = null) {
			
			
		}
		
		public static MultiTargetPath Construct (Vector3[] startPoints, Vector3 target, OnPathDelegate[] callbackDelegates, OnPathDelegate callback = null) {
			MultiTargetPath p = Construct (target, startPoints, callbackDelegates, callback);
			p.inverted = true;
			return p;
		}

		public static MultiTargetPath Construct (Vector3 start, Vector3[] targets, OnPathDelegate[] callbackDelegates, OnPathDelegate callback = null) {
			MultiTargetPath p = PathPool<MultiTargetPath>.GetPath ();
			p.Setup (start,targets,callbackDelegates,callback);
			return p;
		}
		
		protected void Setup (Vector3 start, Vector3[] targets, OnPathDelegate[] callbackDelegates, OnPathDelegate callback) {
			inverted = false;
			this.callback = callback;
			callbacks = callbackDelegates;
			
			targetPoints = targets;
			
			originalStartPoint = start;
			//originalEndPoint = end;
			
			startPoint = start;
			startIntPoint = (Int3)start;
			
			if (targets.Length == 0) {
				Error ();
				LogError ("No targets were assigned to the MultiTargetPath");
				return;
			}
			
			endPoint = targets[0];
			
			originalTargetPoints = new Vector3[targetPoints.Length];
			for (int i=0;i<targetPoints.Length;i++) {
				originalTargetPoints[i] = targetPoints[i];
			}
		}
		
		protected override void Recycle () {
			PathPool<MultiTargetPath>.Recycle (this);
		}
		
		public override void OnEnterPool () {
			
			if (vectorPaths != null)
				for (int i=0;i<vectorPaths.Length;i++)
					if (vectorPaths[i] != null) Util.ListPool<Vector3>.Release (vectorPaths[i]);
				
			vectorPaths = null;
			vectorPath = null;
			
			if (nodePaths != null)
				for (int i=0;i<nodePaths.Length;i++)
					if (nodePaths[i] != null) Util.ListPool<GraphNode>.Release (nodePaths[i]);
				
			nodePaths = null;
			path = null;
			
			base.OnEnterPool ();
		}
		
		public override void ReturnPath () {
			
			if (error) {
				
				if (callbacks != null) {
					for (int i=0;i<callbacks.Length;i++)
						if (callbacks[i] != null) callbacks[i] (this);
				}
				
				if (callback != null) callback(this);
				
				return;
			}
			
			bool anySucceded = false;
			
			Vector3 _originalStartPoint = originalStartPoint;
			Vector3 _startPoint = startPoint;
			GraphNode _startNode = startNode;
			
			for (int i=0;i<nodePaths.Length;i++) {
				
				path = nodePaths[i];
				
				if (path != null) {
					CompleteState = PathCompleteState.Complete;
					anySucceded = true;
				} else {
					CompleteState = PathCompleteState.Error;
				}
				
				if (callbacks != null && callbacks[i] != null) {
					
					vectorPath = vectorPaths[i];
					
					//=== SEGMENT - should be identical to the one a few rows below (except "i")
					if (inverted) {
						endPoint = _startPoint;
						endNode = _startNode;//path[0];
						startNode = targetNodes[i];//path[path.Length-1];
						startPoint = targetPoints[i];
						originalEndPoint = _originalStartPoint;
						originalStartPoint = originalTargetPoints[i];
					} else {
						endPoint = targetPoints[i];
						originalEndPoint = originalTargetPoints[i];
						endNode = targetNodes[i];//path[path.Length-1];
					}
					
					callbacks[i] (this);
					
					//In case a modifier changed the vectorPath, update the array of all vectorPaths
					vectorPaths[i] = vectorPath;
				}
			}
			
			if (anySucceded) {
				CompleteState = PathCompleteState.Complete;
				
				if (!pathsForAll) {
					
					path = nodePaths[chosenTarget];
					vectorPath = vectorPaths[chosenTarget];
					
					//=== SEGMENT - should be identical to the one a few rows above (except "chosenTarget")
					if (inverted) {
						endPoint = _startPoint;
						endNode = _startNode;
						startNode = targetNodes[chosenTarget];
						startPoint = targetPoints[chosenTarget];
						originalEndPoint = _originalStartPoint;
						originalStartPoint = originalTargetPoints[chosenTarget];
					} else {
						endPoint = targetPoints[chosenTarget];
						originalEndPoint = originalTargetPoints[chosenTarget];
						endNode = targetNodes[chosenTarget];
					}
				}
			} else {
				CompleteState = PathCompleteState.Error;
			}
			
			if (callback != null) {
				callback (this);
			}
			
		}
		
		public void FoundTarget (PathNode nodeR, int i) {
			nodeR.flag1 = false;//Reset bit 8
			
			Trace (nodeR);
			vectorPaths[i] = vectorPath;
			nodePaths[i] = path;
			vectorPath = Util.ListPool<Vector3>.Claim ();
			path = Util.ListPool<GraphNode>.Claim ();
			
			targetsFound[i] = true;
			
			targetNodeCount--;
				
			if (!pathsForAll) {
				CompleteState = PathCompleteState.Complete;
				chosenTarget = i; //Mark which path was found
				targetNodeCount = 0;
				return;
			}
			
			
			//If there are no more targets to find, return here and avoid calculating a new hTarget
			if (targetNodeCount <= 0) {
				CompleteState = PathCompleteState.Complete;
				return;
			}
			
			//No need to check for if pathsForAll is true since the function would have returned by now then
			
			if (heuristicMode == HeuristicMode.MovingAverage) {
				Vector3 avg = Vector3.zero;
				int count = 0;
				for (int j=0;j<targetPoints.Length;j++) {
					if (!targetsFound[j]) {
						avg += (Vector3)targetNodes[j].position;
						count++;
					}
				}
				
				if (count > 0) {
					avg /= count;
				}
				hTarget = (Int3)avg;
				
				RebuildOpenList ();
			} else if (heuristicMode == HeuristicMode.MovingMidpoint) {
				
				Vector3 min = Vector3.zero;
				Vector3 max = Vector3.zero;
				bool set = false;
				
				for (int j=0;j<targetPoints.Length;j++) {
					if (!targetsFound[j]) {
						
						if (!set) {
							min = (Vector3)targetNodes[j].position;
							max = (Vector3)targetNodes[j].position;
							set = true;
						} else {
							min = Vector3.Min ((Vector3)targetNodes[j].position,min);
							max = Vector3.Max ((Vector3)targetNodes[j].position,max);
						}
					}
				}
				
				Int3 midpoint = (Int3)((min+max)*0.5F);
				hTarget = (Int3)midpoint;
				
				RebuildOpenList ();
				
			} else if (heuristicMode == HeuristicMode.Sequential) {
				
				//If this was the target hTarget was set to at the moment
				if (sequentialTarget == i) {
					
					//Pick a new hTarget
					float dist = 0;
					
					for (int j=0;j<targetPoints.Length;j++) {
						if (!targetsFound[j]) {
							float d = (targetNodes[j].position-startNode.position).sqrMagnitude;
							if (d > dist) {
								dist = d;
								hTarget = (Int3)targetPoints[j];
								sequentialTarget = j;
							}
						}
					}
					
					RebuildOpenList ();
				}
			}
			
		}
		
		protected void RebuildOpenList () {
			BinaryHeapM heap = pathHandler.GetHeap();
			for (int j=0;j<heap.numberOfItems;j++) {
				PathNode nodeR = heap.GetNode(j);
				nodeR.H = this.CalculateHScore (nodeR.node);
			}
			
			pathHandler.RebuildHeap();
		}
		
		public override void Prepare () {
			
			nnConstraint.tags = enabledTags;
			NNInfo startNNInfo 	= AstarPath.active.GetNearest (startPoint,nnConstraint, startHint);
			startNode = startNNInfo.node;
			
			if (startNode == null) {
				LogError ("Could not find start node for multi target path");
				Error ();
				return;
			}
			
			if (!startNode.Walkable) {
				LogError ("Nearest node to the start point is not walkable");
				Error ();
				return;
			}
			
			//Tell the NNConstraint which node was found as the start node if it is a PathNNConstraint and not a normal NNConstraint
			PathNNConstraint pathNNConstraint = nnConstraint as PathNNConstraint;
			if (pathNNConstraint != null) {
				pathNNConstraint.SetStart (startNNInfo.node);
			}
			
			vectorPaths = new List<Vector3>[targetPoints.Length];
			nodePaths = new List<GraphNode>[targetPoints.Length];
			targetNodes = new GraphNode[targetPoints.Length];
			targetsFound = new bool[targetPoints.Length];
			targetNodeCount = targetPoints.Length;
			
			bool anyWalkable = false;
			bool anySameArea = false;
			bool anyNotNull = false;
			
			for (int i=0;i<targetPoints.Length;i++) {
				NNInfo endNNInfo = AstarPath.active.GetNearest (targetPoints[i],nnConstraint);
				
				targetNodes[i] = endNNInfo.node;
				//Debug.DrawLine (targetPoints[i],targetNodes[i].position,Color.red);
				targetPoints[i] = endNNInfo.clampedPosition;
				if (targetNodes[i] != null) {
					anyNotNull = true;
					endNode = targetNodes[i];
				}
				
				bool notReachable = false;
				
				if (endNNInfo.node != null && endNNInfo.node.Walkable) {
					anyWalkable = true;
				} else {
					notReachable = true;
				}
				
				if (endNNInfo.node != null && endNNInfo.node.Area == startNode.Area) {
					anySameArea = true;
				} else {
					notReachable = true;
				}
				
				if (notReachable) {
					targetsFound[i] = true; //Signal that the pathfinder should not look for this node
					targetNodeCount--;
				}
				
			}
			
			startPoint = startNNInfo.clampedPosition;
			
			startIntPoint = (Int3)startPoint;
			//hTarget = (Int3)endPoint;
			
#if ASTARDEBUG
			Debug.DrawLine (startNode.position,startPoint,Color.blue);
			//Debug.DrawLine (endNode.position,endPoint,Color.blue);
#endif
			
			if (startNode == null || !anyNotNull) {
				LogError ("Couldn't find close nodes to either the start or the end (start = "+(startNode != null ? "found":"not found")+" end = "+(anyNotNull?"at least one found":"none found")+")");
				Error ();
				return;
			}
			
			if (!startNode.Walkable) {
				LogError ("The node closest to the start point is not walkable");
				Error ();
				return;
			}
			
			if (!anyWalkable) {
				LogError ("No target nodes were walkable");
				Error ();
				return;
			}
			
			if (!anySameArea) {
				LogError ("There are no valid paths to the targets");
				Error ();
				return;
			}
			
			//=== Calcuate hTarget ===
			
			if (pathsForAll) {
				if (heuristicMode == HeuristicMode.None) {
					heuristic = Heuristic.None;
					heuristicScale = 0F;
				} else if (heuristicMode == HeuristicMode.Average || heuristicMode == HeuristicMode.MovingAverage) {
					
					Vector3 avg = Vector3.zero;
					
					for (int i=0;i<targetNodes.Length;i++) {
						avg += (Vector3)targetNodes[i].position;
					}
					avg /= targetNodes.Length;
					hTarget = (Int3)avg;
				} else if (heuristicMode == HeuristicMode.Midpoint || heuristicMode == HeuristicMode.MovingMidpoint) {
					
					Vector3 min = Vector3.zero;
					Vector3 max = Vector3.zero;
					bool set = false;
					
					for (int j=0;j<targetPoints.Length;j++) {
						if (!targetsFound[j]) {
							
							if (!set) {
								min = (Vector3)targetNodes[j].position;
								max = (Vector3)targetNodes[j].position;
								set = true;
							} else {
								min = Vector3.Min ((Vector3)targetNodes[j].position,min);
								max = Vector3.Max ((Vector3)targetNodes[j].position,max);
							}
						}
					}
					
					Vector3 midpoint = (min+max)*0.5F;
					hTarget = (Int3)midpoint;
					
				} else if (heuristicMode == HeuristicMode.Sequential) {
					
					float dist = 0;
					
					for (int j=0;j<targetNodes.Length;j++) {
						if (!targetsFound[j]) {
							float d = (targetNodes[j].position-startNode.position).sqrMagnitude;
							if (d > dist) {
								dist = d;
								hTarget = (Int3)targetPoints[j];
								sequentialTarget = j;
							}
						}
					}
				}
			} else {
				heuristic = Heuristic.None;
				heuristicScale = 0.0F;
			}
		}
		
		public override void Initialize () {
			
			
			
			for (int j=0;j < targetNodes.Length;j++) {
				if (startNode == targetNodes[j]) {
					PathNode r = pathHandler.GetPathNode(startNode);
					FoundTarget (r, j);
				} else if (targetNodes[j] != null) {
					pathHandler.GetPathNode (targetNodes[j]).flag1 = true;
				}
			}
			
			//Reset flag1 on all nodes after the pathfinding has completed (no matter if an error occurs or if the path is canceled)
			AstarPath.OnPathPostSearch += ResetFlags;
			
			//If all paths have either been invalidated or found already because they were at the same node as the start node
			if (targetNodeCount <= 0) {
				CompleteState = PathCompleteState.Complete;
				return;
			}
			
			PathNode startRNode = pathHandler.GetPathNode(startNode);
			startRNode.node = startNode;
			startRNode.pathID = pathID;
			startRNode.parent = null;
			startRNode.cost = 0;
			startRNode.G = GetTraversalCost (startNode);
			startRNode.H = CalculateHScore (startNode);
			
			//if (recalcStartEndCosts) {
			//	startNode.InitialOpen (open,hTarget,startIntPoint,this,true);
			//} else {
				startNode.Open (this,startRNode,pathHandler);
			//}
			searchedNodes++;
			
			//any nodes left to search?
			if (pathHandler.HeapEmpty ()) {
				LogError ("No open points, the start node didn't open any nodes");
				Error();
				return;
			}
			
			currentR = pathHandler.PopNode ();
		}
		
		public void ResetFlags (Path p) {
			
			AstarPath.OnPathPostSearch -= ResetFlags;
			if (p != this) {
				Debug.LogError ("This should have been cleared after it was called on 'this' path. Was it not called? Or did the delegate reset not work?");
			}
			
			// Reset all flags
			for ( int i = 0; i < targetNodes.Length; i++ ) {
				if (targetNodes[i] != null) pathHandler.GetPathNode (targetNodes[i]).flag1 = false;
			}
		}
		
		public override void CalculateStep (long targetTick) {
			
			int counter = 0;
			
			//Continue to search while there hasn't ocurred an error and the end hasn't been found
			while (CompleteState == PathCompleteState.NotCalculated) {
				
				//@Performance Just for debug info
				searchedNodes++;
				
				if (currentR.flag1) {
					
					//Close the current node, if the current node is the target node then the path is finnished
					for (int i=0;i<targetNodes.Length;i++) {
						if (!targetsFound[i] && currentR.node == targetNodes[i]) {
							FoundTarget (currentR, i);
							if (CompleteState != PathCompleteState.NotCalculated) {
								break;
							}
						}
					}
					
					if (targetNodeCount <= 0) {
						CompleteState = PathCompleteState.Complete;
						break;
					}
				}
				
				//Loop through all walkable neighbours of the node and add them to the open list.
				currentR.node.Open (this,currentR,pathHandler);
				
				//any nodes left to search?
				if (pathHandler.HeapEmpty()) {
					CompleteState = PathCompleteState.Complete;
					break;
				}
				
				//Select the node with the lowest F score and remove it from the open list
				AstarProfiler.StartFastProfile (7);
				currentR = pathHandler.PopNode ();
				AstarProfiler.EndFastProfile (7);
				
				//Check for time every 500 nodes, roughly every 0.5 ms usually
				if (counter > 500) {
					
					//Have we exceded the maxFrameTime, if so we should wait one frame before continuing the search since we don't want the game to lag
					if (System.DateTime.UtcNow.Ticks >= targetTick) {
						
						//Return instead of yield'ing, a separate function handles the yield (CalculatePaths)
						return;
					}
					
					counter = 0;
				}
				
				counter++;
			
			}
		}
		
		protected override void Trace (PathNode node) {
			base.Trace (node);
			
			if (inverted) {
				
				//Invert the paths
				int half = path.Count/2;
				
				for (int i=0;i<half;i++) {
					GraphNode tmp = path[i];
					path[i] = path[path.Count-i-1];
					path[path.Count-i-1] = tmp;
				}
				
				for (int i=0;i<half;i++) {
					Vector3 tmp = vectorPath[i];
					vectorPath[i] = vectorPath[vectorPath.Count-i-1];
					vectorPath[vectorPath.Count-i-1] = tmp;
				}
			}
		}
		
		public override string DebugString (PathLog logMode) {
			
			if (logMode == PathLog.None || (!error && logMode == PathLog.OnlyErrors)) {
				return "";
			}
			
			System.Text.StringBuilder text = pathHandler.DebugStringBuilder;
			text.Length = 0;
			
			text.Append (error ? "Path Failed : " : "Path Completed : ");
			text.Append ("Computation Time ");
			
			text.Append ((duration).ToString (logMode == PathLog.Heavy ? "0.000" : "0.00"));
			text.Append (" ms Searched Nodes ");
			text.Append (searchedNodes);
			
			if (!error) {
				text.Append ("\nLast Found Path Length ");
				text.Append (path == null ? "Null" : path.Count.ToString ());
			
				if (logMode == PathLog.Heavy) {
					text.Append ("\nSearch Iterations "+searchIterations);
					
					text.Append ("\nPaths (").Append (targetsFound.Length).Append ("):");
					for (int i=0;i<targetsFound.Length;i++) {
						
						text.Append ("\n\n	Path "+i).Append (" Found: ").Append (targetsFound[i]);
						
						GraphNode node = nodePaths[i] == null ? null : nodePaths[i][nodePaths[i].Count-1];
						
						text.Append ("\n		Length: ");
						text.Append (nodePaths[i].Count);
						
						if (node != null) {
							PathNode nodeR = pathHandler.GetPathNode (endNode);
							if (nodeR != null) {
								text.Append ("\n		End Node");
								text.Append ("\n			G: ");
								text.Append (nodeR.G);
								text.Append ("\n			H: ");
								text.Append (nodeR.H);
								text.Append ("\n			F: ");
								text.Append (nodeR.F);
								text.Append ("\n			Point: ");
								text.Append (((Vector3)endPoint).ToString ());
								text.Append ("\n			Graph: ");
								text.Append (endNode.GraphIndex);
							} else {
								text.Append ("\n		End Node: Null");
							}
						}
					}
					
					text.Append ("\nStart Node");
					text.Append ("\n	Point: ");
					text.Append (((Vector3)endPoint).ToString ());
					text.Append ("\n	Graph: ");
					text.Append (startNode.GraphIndex);
					text.Append ("\nBinary Heap size at completion: ");
					text.AppendLine (pathHandler.GetHeap() == null ? "Null" : (pathHandler.GetHeap().numberOfItems-2).ToString ());// -2 because numberOfItems includes the next item to be added and item zero is not used
				}
				
			}
			
			if (error) {
				text.Append ("\nError: ");
				text.Append (errorLog);
				text.AppendLine();
			}
				
			if (logMode == PathLog.Heavy && !AstarPath.IsUsingMultithreading ) {
				text.Append ("\nCallback references ");
				if (callback != null) text.Append(callback.Target.GetType().FullName).AppendLine();
				else text.AppendLine ("NULL");
			}
			
			text.Append ("\nPath Number ");
			text.Append (pathID);
			
			return text.ToString ();
			
		}
		
	}
}