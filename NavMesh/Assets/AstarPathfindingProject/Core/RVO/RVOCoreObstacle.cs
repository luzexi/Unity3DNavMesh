using UnityEngine;
using System.Collections;

namespace Pathfinding.RVO {
	
	/** One vertex in an obstacle.
	  * This is a linked list and one vertex can therefore be used to reference the whole obstacle
	  * \astarpro 
	  */
	public class ObstacleVertex {
		public bool convex;
		
		/** Position of the vertex */
		public Vector3 position;
		public Vector2 dir;
		
		/** Height of the obstacle in this vertex */
		public float height;
		
		/** True if this vertex was created by the KDTree for internal reasons */
		public bool split = false;
		
		public bool thin;
		
		/** Next vertex in the obstacle */
		public ObstacleVertex next;
		/** Previous vertex in the obstacle */
		public ObstacleVertex prev;
	}
}