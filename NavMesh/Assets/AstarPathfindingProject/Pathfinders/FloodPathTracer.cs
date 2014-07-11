using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Pathfinding
{
	/** Restrict suitable nodes by if they have been searched by a FloodPath.
	 * 
	  * Suitable nodes are in addition to the basic contraints, only the nodes which return true on a FloodPath.HasPathTo (node) call.
	  * \see Pathfinding.FloodPath
	  * \see Pathfinding.FloodPathTracer
	  * 
	  * \astarpro
	  */
	public class FloodPathConstraint : NNConstraint {
		
		private FloodPath path;
		
		public FloodPathConstraint (FloodPath path) {
			if (path == null) { Debug.LogWarning ("FloodPathConstraint should not be used with a NULL path"); }
			this.path = path;
		}
		
		public override bool Suitable (GraphNode node)
		{
			return base.Suitable (node) && path.HasPathTo (node);
		}
	}
	
	/** Traces a path created with the Pathfinding.FloodPath.
	 * 
	 * See Pathfinding.FloodPath for examples on how to use this path type
	 * 
	 * \shadowimage{floodPathExample.png}
	 * \astarpro
	 * \ingroup paths */
	public class FloodPathTracer : ABPath
	{
		
		/** Reference to the FloodPath which searched the path originally */
		protected FloodPath flood;
		
		[System.Obsolete("Use the Construct method instead")]
		public FloodPathTracer (Vector3 start, FloodPath flood, OnPathDelegate callbackDelegate) {
			throw new System.Exception ("This constructor is obsolete");
		}
		
		public static FloodPathTracer Construct (Vector3 start, FloodPath flood, OnPathDelegate callback = null) {
			FloodPathTracer p = PathPool<FloodPathTracer>.GetPath ();
			p.Setup (start, flood, callback);
			return p;
		}
		
		protected void Setup (Vector3 start, FloodPath flood, OnPathDelegate callback) {
			this.flood = flood;
			
			if (flood == null || flood.GetState () < PathState.Returned) {
				throw new System.ArgumentException ("You must supply a calculated FloodPath to the 'flood' argument");
			}
			
			base.Setup (start, flood.originalStartPoint, callback);
			nnConstraint = new FloodPathConstraint (flood);
			hasEndPoint = false;
		}
		
		public FloodPathTracer () {}
		
		public override void Reset () {
			base.Reset ();
			flood = null;
		}
		
		protected override void Recycle () {
			PathPool<FloodPathTracer>.Recycle (this);
		}
		
		/** Initializes the path.
		  * Traces the path from the start node.
		  */
		public override void Initialize () {
			
			if (startNode != null && flood.HasPathTo (startNode)) {
				Trace (startNode);
				CompleteState = PathCompleteState.Complete;
			} else {
				Error ();
				LogError ("Could not find valid start node");
			}
		}
		
		public override void CalculateStep (long targetTick) {
			if (!IsDone ()) {
				Error ();
				LogError ("Something went wrong. At this point the path should be completed");
			}
		}
		
		/** Traces the calculated path from the start node to the end.
		 * This will build an array (#path) of the nodes this path will pass through and also set the #vectorPath array to the #path arrays positions.
		 * This implementation will use the #flood (FloodPath) to trace the path from precalculated data.
		 */
		public void Trace (GraphNode from) {
			
			GraphNode c = from;
			int count = 0;
			
			while (c != null) {
				path.Add (c);
				vectorPath.Add ((Vector3)c.position);
				c = flood.GetParent(c);
				
				count++;
				if (count > 1024) {
					Debug.LogWarning ("Inifinity loop? >1024 node path. Remove this message if you really have that long paths (FloodPathTracer.cs, Trace function)");
					break;
				}
			}
		}
	}
}

