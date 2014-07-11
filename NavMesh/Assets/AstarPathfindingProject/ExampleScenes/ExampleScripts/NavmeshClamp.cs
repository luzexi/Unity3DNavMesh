using UnityEngine;
using System.Collections;
using Pathfinding;

namespace Pathfinding {
	/** Attach to any GameObject and the object will be clamped to the navmesh.
	 * If a GameObject has this component attached, one or more graph linecasts will be carried out every frame to ensure that the object
	 * does not leave the navmesh area.\n
	 * It can be used with GridGraphs, but Navmesh based ones are prefered.
	 * 
	 * \note This has partly been replaced by using an RVOController along with RVONavmesh.
	 * It will not yield exactly the same results though, so this script is still useful in some cases.
	 * 
	 * \astarpro */
	public class NavmeshClamp : MonoBehaviour {
		
		GraphNode prevNode;
		Vector3 prevPos;
		
		// Update is called once per frame
		void LateUpdate () {
			
			if (prevNode == null) {
				NNInfo nninfo = AstarPath.active.GetNearest (transform.position);
				prevNode = nninfo.node;
				prevPos = transform.position;
				
			}
			
			if (prevNode == null) {
				return;
			}
			
			if (prevNode != null) {
				
					
				IRaycastableGraph graph = AstarData.GetGraph (prevNode) as IRaycastableGraph;
				if (graph != null) {
					GraphHitInfo hit;
					if (graph.Linecast (prevPos,transform.position,prevNode, out hit)) {
						hit.point.y = transform.position.y;
						Vector3 closest = AstarMath.NearestPoint (hit.tangentOrigin,hit.tangentOrigin+hit.tangent,transform.position);
						Vector3 ohit = hit.point;
						ohit = ohit + Vector3.ClampMagnitude((Vector3)hit.node.position-ohit,0.008f);
						if (graph.Linecast (ohit,closest,hit.node, out hit)) {
							hit.point.y = transform.position.y;
							transform.position = hit.point;
						} else {
							closest.y = transform.position.y;
							
							transform.position = closest;
						}
					}
					prevNode = hit.node;
				}
			}
			
			prevPos = transform.position;
		}
	}
}