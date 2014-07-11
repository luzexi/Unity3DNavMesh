using UnityEngine;
using System.Collections;
using Pathfinding.RVO.Sampled;

namespace Pathfinding.RVO {

	/** Quadtree for quick nearest neighbour search of agents.
	 */
	public class RVOQuadtree
	{

		const int LeafSize = 15;

		float maxRadius = 0;

		struct Node {
			public int child00;
			public int child01;
			public int child10;
			public int child11;
			public byte count;
			public Agent linkedList;

			public void Add ( Agent agent ) {
				agent.next = linkedList;
				linkedList = agent;
			}

			public void Distribute ( Node[] nodes, Rect r ) {
				Vector2 c = r.center;
				while (linkedList != null) {
					Agent nx = linkedList.next;
					if (linkedList.position.x > c.x) {
						if (linkedList.position.z > c.y) {
							nodes[child11].Add (linkedList);
						} else {
							nodes[child10].Add (linkedList);
						}
					} else {
						if (linkedList.position.z > c.y) {
							nodes[child01].Add (linkedList);
						} else {
							nodes[child00].Add (linkedList);
						}
					}
					linkedList = nx;
				}
				count = 0;
			}
		}

		Node[] nodes = new Node[42];
		int filledNodes = 1;

		Rect bounds;

		public void Clear () {
			nodes[0] = new Node();
			filledNodes = 1;
			maxRadius = 0;
		}

		public void SetBounds ( Rect r ) {
			bounds = r;
		}

		public int GetNodeIndex () {
			if ( filledNodes == nodes.Length ) {
				var nds = new Node[nodes.Length*2];
				for ( int i = 0; i < nodes.Length; i++ ) nds[i] = nodes[i];
				nodes = nds;
			}
			nodes[filledNodes] = new Node();
			nodes[filledNodes].child00 = filledNodes;
			filledNodes++;
			return filledNodes-1;
		}

		public void Insert ( Agent agent ) {
			int i = 0;
			Rect r = bounds;
			Vector2 p = new Vector2(agent.position.x, agent.position.z);

			agent.next = null;

			maxRadius = System.Math.Max (agent.radius, maxRadius);

			int depth = 0;

			while (true) {
				depth++;

				if ( nodes[i].child00 == i ) {
					// Leaf node. Break at depth 10 in case lots of agents ( > LeafSize ) are in the same spot
					if ( nodes[i].count < LeafSize || depth > 10 ) {
						nodes[i].Add (agent);
						nodes[i].count++;
						break;
					} else {
						// Split
						Node node = nodes[i];
						node.child00 = GetNodeIndex();
						node.child01 = GetNodeIndex();
						node.child10 = GetNodeIndex();
						node.child11 = GetNodeIndex();
						nodes[i] = node;

						nodes[i].Distribute ( nodes, r );
					}
				}
				// Note, no else
				if ( nodes[i].child00 != i ) {
					// Not a leaf node
					Vector2 c = r.center;
					if (p.x > c.x) {
						if (p.y > c.y) {
							i = nodes[i].child11;
							r = Rect.MinMaxRect ( c.x, c.y, r.xMax, r.yMax );
						} else {
							i = nodes[i].child10;
							r = Rect.MinMaxRect ( c.x, r.yMin, r.xMax, c.y );
						}
					} else {
						if (p.y > c.y) {
							i = nodes[i].child01;
							r = Rect.MinMaxRect ( r.xMin, c.y, c.x, r.yMax );
						} else {
							i = nodes[i].child00;
							r = Rect.MinMaxRect ( r.xMin, r.yMin, c.x, c.y );
						}
					}
				}
			}
		}

		public void Query ( Vector2 p, float radius, Agent agent ) {
			QueryRec ( 0, p, radius, agent, bounds );
		}

		float QueryRec ( int i, Vector2 p, float radius, Agent agent, Rect r ) {

			if ( nodes[i].child00 == i ) {
				// Leaf node
				Agent a = nodes[i].linkedList;
				while (a != null) {
					float v = agent.InsertAgentNeighbour ( a, radius*radius );
					if ( v < radius*radius ) {
						radius = Mathf.Sqrt(v);
					}

					//Debug.DrawLine (a.position, new Vector3(p.x,0,p.y),Color.black);
					/*float dist = (new Vector2(a.position.x, a.position.z) - p).sqrMagnitude;
					if ( dist < radius*radius && a != agent ) {

					}*/
					a = a.next;
				}
			} else {

				// Not a leaf node
				Vector2 c = r.center;
				if (p.x-radius < c.x) {
					if (p.y-radius < c.y) {
						radius = QueryRec ( nodes[i].child00, p, radius, agent, Rect.MinMaxRect ( r.xMin, r.yMin, c.x, c.y ) );
					}
					if (p.y+radius > c.y) {
						radius = QueryRec ( nodes[i].child01, p, radius, agent, Rect.MinMaxRect ( r.xMin, c.y, c.x, r.yMax ) );
					}
				}

				if (p.x+radius > c.x) {
					if (p.y-radius < c.y) {
						radius = QueryRec ( nodes[i].child10, p, radius, agent, Rect.MinMaxRect ( c.x, r.yMin, r.xMax, c.y ) );
					}
					if (p.y+radius > c.y) {
						radius = QueryRec ( nodes[i].child11, p, radius, agent, Rect.MinMaxRect ( c.x, c.y, r.xMax, r.yMax ) );
					}
				}
			}

			return radius;
		}

		public void DebugDraw () {
			DebugDrawRec ( 0, bounds );
		}

		void DebugDrawRec ( int i, Rect r ) {

			Debug.DrawLine ( new Vector3(r.xMin,0,r.yMin), new Vector3(r.xMax,0,r.yMin), Color.white );
			Debug.DrawLine ( new Vector3(r.xMax,0,r.yMin), new Vector3(r.xMax,0,r.yMax), Color.white );
			Debug.DrawLine ( new Vector3(r.xMax,0,r.yMax), new Vector3(r.xMin,0,r.yMax), Color.white );
			Debug.DrawLine ( new Vector3(r.xMin,0,r.yMax), new Vector3(r.xMin,0,r.yMin), Color.white );

			if ( nodes[i].child00 != i ) {
				// Not a leaf node
				Vector2 c = r.center;
				DebugDrawRec ( nodes[i].child11, Rect.MinMaxRect ( c.x, c.y, r.xMax, r.yMax ) );
				DebugDrawRec ( nodes[i].child10, Rect.MinMaxRect ( c.x, r.yMin, r.xMax, c.y ) );
				DebugDrawRec ( nodes[i].child01, Rect.MinMaxRect ( r.xMin, c.y, c.x, r.yMax ) );
				DebugDrawRec ( nodes[i].child00, Rect.MinMaxRect ( r.xMin, r.yMin, c.x, c.y ) );
			}

			Agent a = nodes[i].linkedList;
			while ( a != null ) {
				Debug.DrawLine (nodes[i].linkedList.position+Vector3.up, a.position+Vector3.up, new Color(1,1,0,0.5f));
				a = a.next;
			}
		}
	}
}
