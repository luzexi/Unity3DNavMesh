using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.RVO;

namespace Pathfinding.RVO {
	/** Base class for simple RVO colliders.
	 * 
	 * This is a helper base class for RVO colliders. It provides automatic gizmos
	 * and helps with the winding order of the vertices as well as automatically updating the obstacle when moved.
	 * 
	 * Extend this class to create custom RVO obstacles.
	 * 
	 * \see writing-rvo-colliders
	 * \see RVOSquareObstacle
	 * 
	 * \astarpro 
	 */
	public abstract class RVOObstacle : MonoBehaviour {
		
		/** Mode of the obstacle.
		 * Determines winding order of the vertices */
		public ObstacleVertexWinding obstacleMode;
		
		/** RVO Obstacle Modes.
		  * Determines winding order of obstacle vertices */
		public enum ObstacleVertexWinding {
			KeepOut, /**< Keeps agents from entering the obstacle */
			KeepIn,  /**< Keeps agents inside the obstacle */
			Both     /**< Keeps agents from crossing the obstacle border in either direction */
		}
		
		/** Reference to simulator */
		protected Pathfinding.RVO.Simulator sim;
		
		/** All obstacles added */
		private List<ObstacleVertex> addedObstacles;
		
		/** Original vertices for the obstacles */
		private List<Vector3[]> sourceObstacles;
		
		/** Create Obstacles.
		 * Override this and add logic for creating obstacles.
		 * You should not use the simulator's function calls directly.
		 * 
		 * \see AddObstacle
		 */
		protected abstract void CreateObstacles ();
		
		/** Enable executing in editor to draw gizmos.
		 * If enabled, the CreateObstacles function will be executed in the editor as well
		 * in order to draw gizmos.
		 */
		protected abstract bool ExecuteInEditor {get;}
		
		/** If enabled, all coordinates are handled as local.
		 */
		protected abstract bool LocalCoordinates {get;}
		
		/** Static or dynamic.
		 * This determines if the obstacle can be updated by e.g moving the transform
		 * around in the scene.
		 */
		protected abstract bool StaticObstacle {get; }
		
		/** Called in the editor.
		 * This function should return true if any variables which can change the shape or position of the obstacle
		 * has changed since the last call to this function. Take a look at the RVOSquareObstacle for an example.
		 */
		protected abstract bool AreGizmosDirty ();
		
		/** Enabled if currently in OnDrawGizmos */
		private bool gizmoDrawing = false;
		
		/** Vertices for gizmos */
		private List<Vector3[]> gizmoVerts;
		
		/** Last obstacle mode.
		 * Used to check if the gizmos should be updated
		 */
		private ObstacleVertexWinding _obstacleMode;
		
		/** Last matrix the obstacle was updated with.
		  * Used to check if the obstacle should be updated */
		private Matrix4x4 prevUpdateMatrix;
		
		/** Draws Gizmos */
		public void OnDrawGizmos () {
			OnDrawGizmos (false);
		}
		
		/** Draws Gizmos */
		public void OnDrawGizmosSelected () {
			OnDrawGizmos (true);
		}
		
		/** Draws Gizmos */
		public void OnDrawGizmos (bool selected) {
			gizmoDrawing = true;
			
			Gizmos.color = new Color (0.615f,1,0.06f,selected ? 1.0f : 0.7f);
			
			if (gizmoVerts == null || AreGizmosDirty() || _obstacleMode != obstacleMode) {
				_obstacleMode = obstacleMode;
				
				if (gizmoVerts == null) gizmoVerts = new List<Vector3[]>();
				else gizmoVerts.Clear ();
				
				CreateObstacles ();
			}
			
			Matrix4x4 m = GetMatrix();
			
			for (int i=0;i<gizmoVerts.Count;i++) {
				Vector3[] verts = gizmoVerts[i];
				for (int j=0,q=verts.Length-1;j<verts.Length;q=j++) {
					
					Gizmos.DrawLine (m.MultiplyPoint3x4(verts[j]),m.MultiplyPoint3x4(verts[q]));
					
				}
				
				if (selected && obstacleMode != ObstacleVertexWinding.Both) {
					for (int j=0,q=verts.Length-1;j<verts.Length;q=j++) {
						Vector3 a = m.MultiplyPoint3x4(verts[q]);
						Vector3 b = m.MultiplyPoint3x4(verts[j]);
						
						Vector3 avg = (a + b) * 0.5f;
						Vector3 tang = (b - a).normalized;
						if (tang == Vector3.zero) continue;
						
						Vector3 normal = Vector3.Cross (Vector3.up,tang);
						
						Gizmos.DrawLine (avg,avg+normal);
						Gizmos.DrawLine (avg+normal,avg+normal*0.5f+tang*0.5f);
						Gizmos.DrawLine (avg+normal,avg+normal*0.5f-tang*0.5f);
					}
				}
			}
			
			gizmoDrawing = false;
		}
		
		/** Get's the matrix to use for vertices.
		  * Can be overriden for custom matrices.
		  * \returns transform.localToWorldMatrix if LocalCoordinates is true, otherwise Matrix4x4.identity
		  */
		protected virtual Matrix4x4 GetMatrix () {
			if (LocalCoordinates) return transform.localToWorldMatrix;
			else return Matrix4x4.identity;
		}
		
		/** Disables the obstacle.
		 * Do not override this function
		 */
		public void OnDisable () {
			if (addedObstacles != null) {
				if (sim == null) throw new System.Exception ("This should not happen! Make sure you are not overriding the OnEnable function");
				
				for (int i=0;i<addedObstacles.Count;i++) {
					sim.RemoveObstacle (addedObstacles[i]);
				}
			}
		}
		
		/** Enabled the obstacle.
		 * Do not override this function
		 */
		public void OnEnable () {
			if (addedObstacles != null) {
				if (sim == null) throw new System.Exception ("This should not happen! Make sure you are not overriding the OnDisable function");
				
				for (int i=0;i<addedObstacles.Count;i++) {
					sim.AddObstacle (addedObstacles[i]);
				}
			}
		}
		
		/** Creates obstacles */
		public void Start () {
			addedObstacles = new List<ObstacleVertex>();
			sourceObstacles = new List<Vector3[]>();
			prevUpdateMatrix = GetMatrix();
			CreateObstacles ();
		}
		
		/** Updates obstacle if required.
		  * Checks for if the obstacle should be updated (e.g if it has moved) */
		public void Update () {
			Matrix4x4 m = GetMatrix();
			
			if (m != prevUpdateMatrix) {
				
				for (int i=0;i<addedObstacles.Count;i++) {
					sim.UpdateObstacle (addedObstacles[i],sourceObstacles[i],m);
				}
				prevUpdateMatrix = m;
			}
		}
		
		
		/** Finds a simulator in the scene.
		 * 
		 * Saves found simulator in #sim.
		 * 
		 * \throws System.InvalidOperationException When no RVOSimulator could be found.
		 */
		protected void FindSimulator () {
			RVOSimulator rvosim = FindObjectOfType(typeof(RVOSimulator)) as RVOSimulator;
			if (rvosim == null) throw new System.InvalidOperationException ("No RVOSimulator could be found in the scene. Please add one to any GameObject");
			
			sim = rvosim.GetSimulator ();
		}
		
		/** Adds an obstacle with the specified vertices.
		  * The vertices array might be changed by this function. */
		protected void AddObstacle (Vector3[] vertices, float height) {
			if (vertices == null) throw new System.ArgumentNullException ("Vertices Must Not Be Null");
			if (height < 0) throw new System.ArgumentOutOfRangeException ("Height must be non-negative");
			if (vertices.Length < 2) throw new System.ArgumentException ("An obstacle must have at least two vertices");
			
			if (gizmoDrawing) {
				//Matrix4x4 m = GetMatrix();
				//if (!m.isIdentity) for (int i=0;i<vertices.Length;i++) vertices[i] = m.MultiplyPoint3x4(vertices[i]);
				
				Vector3[] v = new Vector3[vertices.Length];
				WindCorrectly (vertices);
				System.Array.Copy(vertices,v,vertices.Length);
				gizmoVerts.Add (v);
				return;
			}
			
			if (sim == null) FindSimulator ();
			
			if (vertices.Length == 2) {
				AddObstacleInternal (vertices, height);
				return;
			}
			
			if (obstacleMode == ObstacleVertexWinding.Both) {
				for (int i=0,j=vertices.Length-1;i<vertices.Length;j = i++) {
					AddObstacleInternal (new Vector3[] {vertices[j],vertices[i]},height);
				}
			} else {
				WindCorrectly (vertices);
				AddObstacleInternal (vertices,height);
			}
		}
		
		/** Adds an obstacle.
		 * Winding is assumed to be correct and very little error checking is done.
		 */
		private void AddObstacleInternal (Vector3[] vertices, float height) {
			addedObstacles.Add (sim.AddObstacle (vertices,height, GetMatrix()));
			sourceObstacles.Add (vertices);
		}
		
		/** Winds the vertices correctly.
		 * Winding order is determined from #obstacleMode.
		 */
		private void WindCorrectly (Vector3[] vertices) {
			if (obstacleMode == ObstacleVertexWinding.Both) return;
				
			int leftmost = 0;
			float leftmostX = float.PositiveInfinity;
			for (int i=0;i<vertices.Length;i++) {
				if (vertices[i].x < leftmostX) {
					leftmost = i;
					leftmostX = vertices[i].x;
				}
			}
			
			if (Polygon.IsClockwise (vertices[(leftmost-1 + vertices.Length) % vertices.Length], vertices[leftmost], vertices[(leftmost+1) % vertices.Length])) {
				if (obstacleMode == ObstacleVertexWinding.KeepOut) System.Array.Reverse (vertices);
			} else {
				if (obstacleMode == ObstacleVertexWinding.KeepIn) System.Array.Reverse (vertices);
			}
		}
	}
}