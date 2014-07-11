using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.RVO;

namespace Pathfinding.RVO {
	/** Adds a navmesh as RVO obstacles.
	 * Add this to a scene in which has a navmesh based graph, when scanning (or loading from cache) the graph
	 * it will be added as RVO obstacles to the RVOSimulator (which must exist in the scene).
	 * 
	 * \todo Support for grid based graphs will be added in future versions
	 * 
	 * \astarpro 
	 */
	[AddComponentMenu("Pathfinding/Local Avoidance/RVO Navmesh")]
	public class RVONavmesh : GraphModifier {
		
		/** Height of the walls added for each obstacle edge.
		 * If a graph contains overlapping you should set this low enough so
		 * that edges on different levels do not interfere, but high enough so that
		 * agents cannot move over them by mistake.
		 */
		public float wallHeight = 5;
		
		/** Obstacles currently added to the simulator */
		private List<ObstacleVertex> obstacles = new List<ObstacleVertex>();
		
		/** Last simulator used */
		private Simulator lastSim = null;
		
		public override void OnPostCacheLoad ()
		{
			OnLatePostScan ();
		}
		
		public override void OnLatePostScan () {
			if (!Application.isPlaying) return;
			
			RemoveObstacles ();
			
			NavGraph[] graphs = AstarPath.active.graphs;
			
			RVOSimulator rvosim = FindObjectOfType(typeof(RVOSimulator)) as RVOSimulator;
			if (rvosim == null) throw new System.NullReferenceException ("No RVOSimulator could be found in the scene. Please add one to any GameObject");
			
			Pathfinding.RVO.Simulator sim = rvosim.GetSimulator ();
			
			for (int i=0;i<graphs.Length;i++) {
				AddGraphObstacles (sim, graphs[i]);
			}
			
			sim.UpdateObstacles ();
		}
		
		/** Removes obstacles which were added with AddGraphObstacles */
		public void RemoveObstacles () {
			if (lastSim == null) return;
			
			Pathfinding.RVO.Simulator sim = lastSim;
			lastSim = null;
			
			for (int i=0;i<obstacles.Count;i++) sim.RemoveObstacle (obstacles[i]);
			
			obstacles.Clear ();
		}
		
		/** Adds obstacles for a graph */
		public void AddGraphObstacles (Pathfinding.RVO.Simulator sim, NavGraph graph) {
			if (obstacles.Count > 0 && lastSim != null && lastSim != sim) {
				Debug.LogError ("Simulator has changed but some old obstacles are still added for the previous simulator. Deleting previous obstacles.");
				RemoveObstacles ();
			}
			
			//Remember which simulator these obstacles were added to
			lastSim = sim;
			
			INavmesh ng = graph as INavmesh;
			
			if (ng == null) return;
			
			//Assume less than 20 vertices per node (actually assumes 3, but I will change that some day)
			int[] uses = new int[20];
			
			ng.GetNodes (delegate(GraphNode _node) {
				TriangleMeshNode node = _node as TriangleMeshNode;
				
				uses[0] = uses[1] = uses[2] = 0;
				
				if (node != null) {
					
					//Find out which edges are shared with other nodes
					for (int j=0;j<node.connections.Length;j++) {
						TriangleMeshNode other = node.connections[j] as TriangleMeshNode;
						
						// Not necessarily a TriangleMeshNode
						if (other != null) {
							int a = node.SharedEdge(other);
							if (a != -1) uses[a] = 1;
						}
					}
					
					//Loop through all edges on the node
					for (int j=0;j<3;j++) {
						//The edge is not shared with any other node
						//I.e it is an exterior edge on the mesh
						if (uses[j] == 0) {
							//The two vertices of the edge
							Vector3 v1 = (Vector3)node.GetVertex(j);
							Vector3 v2 = (Vector3)node.GetVertex((j+1) % node.GetVertexCount());
							
							//I think node vertices always should be clockwise, but it's good to be certain
							/*if (!Polygon.IsClockwise (v1,v2,(Vector3)node.GetVertex((j+2) % node.GetVertexCount()))) {
								Vector3 tmp = v2;
								v2 = v1;
								v1 = tmp;
							}*/
							
		#if ASTARDEBUG
							Debug.DrawLine (v1,v2,Color.red);
							Debug.DrawRay (v1,Vector3.up*wallHeight,Color.red);
		#endif
							
							//Find out the height of the wall/obstacle we are about to add
							float height = System.Math.Abs(v1.y-v2.y);
							height = System.Math.Max (height,5);
							
							//Add the edge as a line obstacle
							obstacles.Add (sim.AddObstacle (v1, v2, wallHeight));
						}
					}
				}
				
				return true;
			});
			
		}
	}
}