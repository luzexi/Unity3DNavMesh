using UnityEngine;
using System.Collections;
using Pathfinding.RVO;

namespace Pathfinding.RVO {
	
	/**
	 * Square Obstacle for RVO Simulation.
	 * 
	 * \astarpro 
	 */
	[AddComponentMenu("Pathfinding/Local Avoidance/Square Obstacle (disabled)")]
	public class RVOSquareObstacle : RVOObstacle {
		
		/** Height of the obstacle */
		public float height = 1;
		
		/** Size of the square */
		public Vector2 size = Vector3.one;
		
		protected override bool StaticObstacle { get { return false; }}
		protected override bool ExecuteInEditor { get { return true; }}
		protected override bool LocalCoordinates { get { return true; }}
		
		//If UNITY_EDITOR to save a few bytes, these are only needed in the editor
	#if UNITY_EDITOR
		private Vector2 _size;
		private float _height;
	#endif
		
		protected override bool AreGizmosDirty () {
	#if UNITY_EDITOR
			bool ret = _size != size || _height != height;
			_size = size;
			_height = height;
			return ret;
	#else
			return false;
	#endif
		}
		
		protected override void CreateObstacles ()
		{
			size.x = Mathf.Abs(size.x);
			size.y = Mathf.Abs(size.y);
			height = Mathf.Abs(height);
			Vector3[] verts = new Vector3[] {new Vector3(1,0,-1), new Vector3(1,0,1), new Vector3 (-1,0,1), new Vector3 (-1,0,-1)};
			for (int i=0;i<verts.Length;i++) verts[i].Scale (new Vector3(size.x,0,size.y));
			
			AddObstacle (verts, height);
		}
	}
}