using System;
using UnityEngine;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.Voxels;

namespace Pathfinding
{
	/** Axis Aligned Bounding Box Tree.
	 * Holds a bounding box tree of triangles.\n
	 * \b Performance: Insertion - Practically O(1) - About 0.003 ms
	 * \astarpro
	 */
	public class RecastBBTree
	{
		public RecastBBTreeBox root;
		
		/*public NNInfo Query (Vector3 p) {
			
			RecastBBTreeBox c = root;
			
			if (c == null) {
				return new NNInfo();
			}
			
			NNInfo nnInfo = new NNInfo ();
			
			SearchBox (c,p, constraint, ref nnInfo);
			
			nnInfo.UpdateInfo ();
			
			return nnInfo;
		}*/
		
		/** Queries the tree for the best node, searching within a circle around \a p with the specified radius.
		  * Will fill in both the constrained node and the not constrained node in the NNInfo.
		  * 
		  * \see QueryClosest
		  */
		/*public NNInfo QueryCircle (Vector3 p, float radius) {
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
		}*/
		
		/** Queries the tree for the closest node to \a p constrained by the NNConstraint.
		  * Note that this function will, unlike QueryCircle, only fill in the constrained node.
		  * If you want a node not constrained by any NNConstraint, do an additional search with constraint = NNConstraint.None
		  * 
		  * \see QueryCircle
		  */
		/*public NNInfo QueryClosest (Vector3 p, out float distance) {
			distance = float.PositiveInfinity;
			return QueryClosest (p, constraint, ref distance, new NNInfo (null));
		}*/
		
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
		public void QueryInBounds (Rect bounds, List<RecastMeshObj> buffer) {
			RecastBBTreeBox c = root;
			
			if (c == null) return;
			
			QueryBoxInBounds (c, bounds, buffer);
		}
		
		public void QueryBoxInBounds (RecastBBTreeBox box, Rect bounds, List<RecastMeshObj> boxes) {
			
			if (box.mesh != null) {
				//Leaf node
				if (RectIntersectsRect (box.rect,bounds)) {
					//Update the NNInfo
					
#if ASTARDEBUG
					Debug.DrawLine ((Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(1) + Vector3.up*0.2f,Color.red);
					Debug.DrawLine ((Vector3)box.node.GetVertex(2) + Vector3.up*0.2f,(Vector3)box.node.GetVertex(0) + Vector3.up*0.2f,Color.red);
#endif
					
					boxes.Add (box.mesh);
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
				if (RectIntersectsRect (box.c1.rect,bounds)) {
					QueryBoxInBounds (box.c1, bounds, boxes);
				}
				
				if (RectIntersectsRect (box.c2.rect,bounds)) {
					QueryBoxInBounds (box.c2, bounds, boxes);
				}
			}
		}
		
		/*public void SearchBoxCircle (BBTreeBox box, Vector3 p, float radius, NNConstraint constraint, ref NNInfo nnInfo) {//, int intendentLevel = 0) {
			
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
					if (constraint.Suitable (box.node)) {
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
				SearchBoxCircle (box.c1,p, radius, ref nnInfo);
			}
			
			if (RectIntersectsCircle (box.c2.rect,p,radius)) {
				SearchBoxCircle (box.c2,p, radius, ref nnInfo);
			}
		}*/
		
		/*public void SearchBox (BBTreeBox box, Vector3 p, NNConstraint constraint, ref NNInfo nnInfo) {//, int intendentLevel = 0) {
			
			if (box.node != null) {
				//Leaf node
				if (box.node.ContainsPoint ((Int3)p)) {
					//Update the NNInfo
					
					if (nnInfo.node == null) {
						nnInfo.node = box.node;
					} else if (Mathf.Abs(((Vector3)box.node.Position).y - p.y) < Mathf.Abs (((Vector3)nnInfo.node.Position).y - p.y)) {
						nnInfo.node = box.node;
					}
					if (constraint.Suitable (box.node)) {
						if (nnInfo.constrainedNode == null) {
							nnInfo.constrainedNode = box.node;
						} else if (Mathf.Abs(box.node.Position.y - p.y) < Mathf.Abs (nnInfo.constrainedNode.Position.y - p.y)) {
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
		}*/
		
		/** Removes the specified mesh from the tree.
		 * Assumes that it has the correct bounds information.
		 * 
		 * \returns True if the mesh was removed from the tree, false otherwise.
		 */
		public bool Remove (RecastMeshObj mesh) {
			
			if (mesh == null) throw new System.ArgumentNullException("mesh");
			
			if (root == null) {
				return false;
			}
			
			bool found = false;
			Bounds b = mesh.GetBounds();
			//Convert to top down rect
			Rect r = Rect.MinMaxRect (b.min.x, b.min.z, b.max.x, b.max.z);
			
			root = RemoveBox (root, mesh, r, ref found);
			
			return found;
		}
		
		private RecastBBTreeBox RemoveBox (RecastBBTreeBox c, RecastMeshObj mesh, Rect bounds, ref bool found) {
			
			if (!RectIntersectsRect (c.rect,bounds)) {
				return c;
			}
			
			if (c.mesh == mesh) {
				found = true;
				return null;
			} else {
				if (c.mesh == null && !found) {
					c.c1 = RemoveBox (c.c1, mesh, bounds, ref found);
					if (c.c1 == null) {
						return c.c2;
					}
					
					if (!found) {
						c.c2 = RemoveBox (c.c2, mesh, bounds, ref found);
						if (c.c2 == null) {
							return c.c1;
						}
					}
					
					if (found) {
						c.rect = ExpandToContain (c.c1.rect, c.c2.rect);
					}
				}
				return c;
			}
		}
			
		/** Inserts a mesh node in the tree */
		public void Insert (RecastMeshObj mesh) {
			RecastBBTreeBox box = new RecastBBTreeBox (this,mesh);
			
			if (root == null) {
				root = box;
				return;
			}
			
			RecastBBTreeBox c = root;
			while (true) {
				
				c.rect = ExpandToContain (c.rect,box.rect);
				if (c.mesh != null) {
					//Is Leaf
					c.c1 = box;
					RecastBBTreeBox box2 = new RecastBBTreeBox (this,c.mesh);
					//Console.WriteLine ("Inserted "+box.node+", rect "+box.rect.ToString ());
					c.c2 = box2;
					
					
					c.mesh = null;
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
		
		public void OnDrawGizmos () {
			//Gizmos.color = new Color (1,1,1,0.01F);
			//OnDrawGizmos (root);
		}
		
		public void OnDrawGizmos (RecastBBTreeBox box) {
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
		
		public void TestIntersections (Vector3 p, float radius) {
			
			RecastBBTreeBox box = root;
			
			
			TestIntersections (box,p,radius);
		}
		
		public void TestIntersections (RecastBBTreeBox box, Vector3 p, float radius) {
			
			if (box == null) {
				return;
			}
			
			RectIntersectsCircle (box.rect,p,radius);
			
			TestIntersections (box.c1,p,radius);
			TestIntersections (box.c2,p,radius);
		}

		public bool RectIntersectsRect (Rect r, Rect r2) {
			return (r.xMax > r2.xMin && r.yMax > r2.yMin && r2.xMax > r.xMin && r2.yMax > r.yMin);
		}
		
		public bool RectIntersectsCircle (Rect r, Vector3 p, float radius) {
			
			if (float.IsPositiveInfinity(radius)) return true;
			
			if (RectContains (r,p)) {
				return true;
			}
			
			return XIntersectsCircle (r.xMin,r.xMax,r.yMin,p,radius) ||
			XIntersectsCircle (r.xMin,r.xMax,r.yMax,p,radius) ||
			ZIntersectsCircle (r.yMin,r.yMax,r.xMin,p,radius) ||
			ZIntersectsCircle (r.yMin,r.yMax,r.xMax,p,radius);
			
		}
		
		/** Returns if a rect contains the 3D point in XZ space */
		public bool RectContains (Rect r, Vector3 p) {
			return p.x >= r.xMin && p.x <= r.xMax && p.z >= r.yMin && p.z <= r.yMax;
		}
		
		public bool ZIntersectsCircle (float z1, float z2, float xpos, Vector3 circle, float radius) {
			double f = Math.Abs (xpos-circle.x)/radius;
			if (f > 1.0 || f < -1.0) {
				return false;
			}
			
			//double s = Math.Acos (f);
			
			//float s1 = (float)Math.Sin (s)*radius;
			float s1 = (float)Math.Sqrt (1.0 - f*f)*radius;
			
			float s2 = circle.z - s1;
				  s1 += circle.z;
			
			float min = Math.Min (s1,s2);
			float max = Math.Max (s1,s2);
			
			min = Mathf.Max (z1,min);
			max = Mathf.Min (z2,max);
			
			return max > min;
		}
		
		public bool XIntersectsCircle (float x1, float x2, float zpos, Vector3 circle, float radius) {
			double f = Math.Abs (zpos-circle.z)/radius;
			if (f > 1.0 || f < -1.0) {
				return false;
			}
			
			//double s = Math.Asin (f);
			
			//float s1 = (float)Math.Cos (s)*radius;
			float s1 = (float)Math.Sqrt (1.0 - f*f)*radius;
			
			float s2 = circle.x - s1;
			s1 += circle.x;
			
			float min = Math.Min (s1,s2);
			float max = Math.Max (s1,s2);
			
			min = Mathf.Max (x1,min);
			max = Mathf.Min (x2,max);
			
			return max > min;
		}
		
		/** Returns the difference in area between \a r and \a r expanded to contain \a r2 */
		public float ExpansionRequired (Rect r, Rect r2) {
			float xMin = Mathf.Min (r.xMin,r2.xMin);
			float xMax = Mathf.Max (r.xMax,r2.xMax);
			float yMin = Mathf.Min (r.yMin,r2.yMin);
			float yMax = Mathf.Max (r.yMax,r2.yMax);
			
			return (xMax-xMin)*(yMax-yMin)-RectArea (r);
		}
		
		/** Returns a new rect which contains both \a r and \a r2 */
		public Rect ExpandToContain (Rect r, Rect r2) {
			float xMin = Mathf.Min (r.xMin,r2.xMin);
			float xMax = Mathf.Max (r.xMax,r2.xMax);
			float yMin = Mathf.Min (r.yMin,r2.yMin);
			float yMax = Mathf.Max (r.yMax,r2.yMax);
			
			return Rect.MinMaxRect (xMin,yMin,xMax,yMax);
		}
		
		/** Returns the area of a rect */
		public float RectArea (Rect r) {
			return r.width*r.height;
		}
		
		public new void ToString () {
			//Console.WriteLine ("Root "+(root.node != null ? root.node.ToString () : ""));
			
			RecastBBTreeBox c = root;
			
			Stack<RecastBBTreeBox> stack = new Stack<RecastBBTreeBox>();
			stack.Push (c);
			
			c.WriteChildren (0);
		}
	}
	
	public class RecastBBTreeBox {
		public Rect rect;
		public RecastMeshObj mesh;
		
		public RecastBBTreeBox c1;
		public RecastBBTreeBox c2;
		
		public RecastBBTreeBox (RecastBBTree tree, RecastMeshObj mesh) {
			this.mesh = mesh;
			
			Vector3 min = mesh.bounds.min;
			Vector3 max = mesh.bounds.max;
			rect = Rect.MinMaxRect (min.x,min.z,max.x,max.z);
		}
		
		public bool Contains (Vector3 p) {
			return rect.Contains (p);
		}
		
		public void WriteChildren (int level) {
			for (int i=0;i<level;i++) {
#if !NETFX_CORE
				Console.Write ("  ");
#endif
			}
			if (mesh != null) {
#if !NETFX_CORE
				Console.WriteLine ("Leaf ");//+triangle.ToString ());
#endif
			} else {
#if !NETFX_CORE
				Console.WriteLine ("Box ");//+rect.ToString ());
#endif
				c1.WriteChildren (level+1);
				c2.WriteChildren (level+1);
			}
		}
	}
}



