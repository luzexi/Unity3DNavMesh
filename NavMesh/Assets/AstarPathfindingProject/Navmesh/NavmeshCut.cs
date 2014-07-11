
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;

namespace Pathfinding {
	/** Navmesh cutting for fast recast graph updating.
	 * 
	\htmlonly
	<iframe width="640" height="480" src="//www.youtube.com/embed/qXi5qhhGNIw" frameborder="0" allowfullscreen>
	</iframe>
	\endhtmlonly
	
	\astarpro
	 
	Usually you want a TileHandlerHelper somewhere in your scene which handles applying navmesh cutting when these components are enabled or disabled.
	
	\see http://www.arongranberg.com/2013/08/navmesh-cutting/
	
	*/
	[AddComponentMenu("Pathfinding/Navmesh/Navmesh Cut")]
	public class NavmeshCut : MonoBehaviour {
		
		public enum MeshType {
			Rectangle,
			Circle,
			CustomMesh
		}
		
		private static List<NavmeshCut> allCuts = new List<NavmeshCut>();
	
		/** Called every time a NavmeshCut component is destroyed. */
		public static event System.Action<NavmeshCut> OnDestroyCallback;
	
		private static void AddCut (NavmeshCut obj) {
			allCuts.Add (obj);
		}
		
		private static void RemoveCut (NavmeshCut obj) {
			allCuts.Remove (obj);
		}
		
		/** Get all active instances which intersect the bounds */
		public static List<NavmeshCut> GetAllInRange(Bounds b) {
			List<NavmeshCut> cuts = Pathfinding.Util.ListPool<NavmeshCut>.Claim ();
			for (int i=0;i<allCuts.Count;i++) {
				if (allCuts[i].enabled && Intersects(b, allCuts[i].GetBounds())) {
					cuts.Add (allCuts[i]);
				}
			}
			return cuts;
		}
	
		/** True if \a b1 and \a b2 intersects.
		 * 
		 * \note Faster than Unity's built in version. See http://forum.unity3d.com/threads/204243-Slow-Unity-Math-Please-Unity-Tech-keep-core-math-fast?p=1404070#post1404070
		 */
		private static bool Intersects (Bounds b1, Bounds b2)        {
			Vector3 min1 = b1.min;
			Vector3 max1 = b1.max;
			Vector3 min2 = b2.min;
			Vector3 max2 = b2.max;
			return min1.x <= max2.x && max1.x >= min2.x && min1.y <= max2.y && max1.y >= min2.y && min1.z <= max2.z && max1.z >= min2.z;
		}
	
		/** Returns a list with all NavmeshCut components in the scene.
		 * \warning Do not modify this array
		 */
		public static List<NavmeshCut> GetAll () {
			return allCuts;
		}
		
		public MeshType type;
		
		/** Custom mesh to use.
		 * The contour(s) of the mesh will be extracted.
		 * If you get the "max perturbations" error when cutting with this, check the normals on the mesh.
		 * They should all point in the same direction. Try flipping them if that does not help.
		 */
		public Mesh mesh;
		
		/** Size of the rectangle */
		public Vector2 rectangleSize = new Vector2(1,1);
		
		/** Radius of the circle */
		public float circleRadius = 1;
		
		/** Number of vertices on the circle */
		public int circleResolution = 6;
		public float height = 1;
		
		/** Scale of the custom mesh, if used */
		public float meshScale = 1;
		
		public Vector3 center;
		
		/** Distance between positions to require an update of the navmesh.
		 * A smaller distance gives better accuracy, but requires more updates when moving the object over time,
		 * so it is often slower.
		 * 
		 * \note Dynamic updating requires a TileHandlerHelper somewhere in the scene.
		 */
		public float updateDistance = 0.4f;
		
		/** Only makes a split in the navmesh, but does not remove the geometry to make a hole.
		 * This is slower than a normal cut
		 */
		public bool isDual = false;
		
		/** Cuts geometry added by a NavmeshAdd component.
		  * You rarely need to change this 
		  */
		public bool cutsAddedGeom = true;
	
		/** How many degrees rotation that is required for an update to the navmesh.
		 * Should be between 0 and 180.
		 * 
		 * \note Dynamic updating requires a Tile Handler Helper somewhere in the scene.
		 */
		public float updateRotationDistance = 10;
		
		/** Includes rotation in calculations.
		 * This is slower since a lot more matrix multiplications are needed but gives more flexibility.
		 */
		public bool useRotation = false;
		
		Vector3[][] contours;
		
		/** cached transform component */
		protected Transform tr;
		Mesh lastMesh;
		Vector3 lastPosition;
		Quaternion lastRotation;
		bool wasEnabled;
		Bounds bounds;
		Bounds lastBounds;
		
		public Bounds LastBounds {
			get {
				return lastBounds;
			}
		}
		
		public void Awake () {
			AddCut(this);
		}
		
		public void OnEnable () {
			tr = transform;
			lastPosition = new Vector3(float.PositiveInfinity,float.PositiveInfinity,float.PositiveInfinity);
			lastRotation = tr.rotation;
		}
		
		public void OnDestroy () {
			if (OnDestroyCallback != null) OnDestroyCallback (this);
			RemoveCut(this);
		}
		
		/** Cached variable, do avoid allocations */
		static readonly Dictionary<Pathfinding.Int2,int> edges = new Dictionary<Pathfinding.Int2, int>();
		/** Cached variable, do avoid allocations */
		static readonly Dictionary<int,int> pointers = new Dictionary<int, int>();
		
		/** Forces this navmesh cut to update the navmesh.
		 * 
		 * \note Dynamic updating requires a Tile Handler Helper somewhere in the scene.
		 * This update is not instant, it is done the next time the TileHandlerHelper checks this instance for
		 * if it needs updating.
		 * 
		 * \see TileHandlerHelper.ForceUpdate()
		 */
		public void ForceUpdate () {
			lastPosition = new Vector3(float.PositiveInfinity,float.PositiveInfinity,float.PositiveInfinity);
		}
		
		/** Returns true if this object has moved so much that it requires an update.
		 * When an update to the navmesh has been done, call NotifyUpdated to be able to get
		 * relavant output from this method again.
		 */
		public bool RequiresUpdate () {
			return wasEnabled != enabled || (wasEnabled && ((tr.position-lastPosition).sqrMagnitude > updateDistance*updateDistance || (useRotation && (Quaternion.Angle (lastRotation, tr.rotation) > updateRotationDistance))));
		}
		
		public virtual void UsedForCut () {
		}
		
		public void NotifyUpdated () {
			wasEnabled = enabled;
			
			if (wasEnabled) {
				lastPosition = tr.position;
				lastBounds = GetBounds();
				
				if (useRotation) {
					lastRotation = tr.rotation;
				}
			}
		}
		
		void CalculateMeshContour () {
			if (mesh == null) return;
			
			edges.Clear();
			pointers.Clear();
			
			Vector3[] verts = mesh.vertices;
			int[] tris = mesh.triangles;
			for (int i=0;i<tris.Length;i+=3) {
				
				// Make sure it is clockwise
				if ( Polygon.IsClockwise ( verts[tris[i+0]], verts[tris[i+1]], verts[tris[i+2]] ) ) {
					int tmp = tris[i+0];
					tris[i+0] = tris[i+2];
					tris[i+2] = tmp;
				}
				
				edges[new Pathfinding.Int2(tris[i+0],tris[i+1])] = i;
				edges[new Pathfinding.Int2(tris[i+1],tris[i+2])] = i;
				edges[new Pathfinding.Int2(tris[i+2],tris[i+0])] = i;
			}
			
			for (int i=0;i<tris.Length;i+=3) {
				for (int j=0;j<3;j++) {
					if (!edges.ContainsKey(new Pathfinding.Int2(tris[i+((j+1)%3)], tris[i+((j+0)%3)]))) {
						pointers[tris[i+((j+0)%3)]] = tris[i+((j+1)%3)];
					}
				}
			}
			
			List<Vector3[]> contourBuffer = new List<Vector3[]>();
			
			List<Vector3> buffer = Pathfinding.Util.ListPool<Vector3>.Claim();
			for (int i=0;i<verts.Length;i++) {
				if (pointers.ContainsKey(i)) {
					buffer.Clear();
					
					int s = i;
					do {
						int tmp = pointers[s];
						
						//This path has been taken before
						if (tmp == -1) break;
						
						pointers[s] = -1;
						buffer.Add (verts[s]);
						//Debug.Log ("From " + s + " to " + tmp);
						s = tmp;
						
						if (s == -1) {
							Debug.LogError ("Invalid Mesh '"  + mesh.name + " in " + gameObject.name);
							break;
						}
					} while (s != i);
					
					if (buffer.Count > 0) contourBuffer.Add (buffer.ToArray());
				}
			}
			
			
			Pathfinding.Util.ListPool<Vector3>.Release(buffer);
			contours = contourBuffer.ToArray();
		}
		
		public Bounds GetBounds () {
			switch (type) {
			case MeshType.Rectangle:
				if (useRotation) {
					Matrix4x4 m = tr.localToWorldMatrix;//Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
					bounds = new Bounds(m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,-height,-rectangleSize.y)*0.5f), Vector3.zero);
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,-height,-rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,-height,rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,-height,rectangleSize.y)*0.5f));
					
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,height,-rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,height,-rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,height,rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,height,rectangleSize.y)*0.5f));
					
				} else {
					bounds = new Bounds(tr.position+center, new Vector3(rectangleSize.x,height,rectangleSize.y));
				}
				break;
			case MeshType.Circle:
				if (useRotation) {
					Matrix4x4 m = tr.localToWorldMatrix;//Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
					bounds = new Bounds(m.MultiplyPoint3x4 (center), new Vector3(circleRadius*2,height,circleRadius*2));
				} else {
					bounds = new Bounds(transform.position+center, new Vector3(circleRadius*2,height,circleRadius*2));
				}
				break;
			case MeshType.CustomMesh:
				if (mesh == null) break;
				
				Bounds b = mesh.bounds;
				if (useRotation) {
					Matrix4x4 m = tr.localToWorldMatrix;//Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
					b.center *= meshScale;
					b.size *= meshScale;
					
					bounds = new Bounds ( m.MultiplyPoint3x4 ( center + b.center ), Vector3.zero );
					
					Vector3 mx = b.max;
					Vector3 mn = b.min;
					
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mx.x,mx.y,mx.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mn.x,mx.y,mx.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mn.x,mx.y,mn.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mx.x,mx.y,mn.z )) );
					
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mx.x,mn.y,mx.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mn.x,mn.y,mx.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mn.x,mn.y,mn.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mx.x,mn.y,mn.z )) );
					
					Vector3 size = bounds.size;
					size.y = Mathf.Max(size.y, height * tr.lossyScale.y);
					bounds.size = size;
				} else {
					Vector3 size = b.size*meshScale;
					size.y = Mathf.Max(size.y, height);
					bounds = new Bounds(transform.position+center+b.center*meshScale,size);
				}
				break;
			}
			return bounds;
		}
		
		// Use this for initialization
		public void GetContour (List<List<Pathfinding.ClipperLib.IntPoint>> buffer) {
			
			if (circleResolution < 3) circleResolution = 3;
			
			Vector3 woffset = tr.position;
			switch (type) {
			case MeshType.Rectangle:
				List<Pathfinding.ClipperLib.IntPoint> buffer0 = Pathfinding.Util.ListPool<Pathfinding.ClipperLib.IntPoint>.Claim();
				if (useRotation) {
					Matrix4x4 m = tr.localToWorldMatrix;//Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
					buffer0.Add (V3ToIntPoint(m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,0,-rectangleSize.y)*0.5f)));
					buffer0.Add (V3ToIntPoint(m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,0,-rectangleSize.y)*0.5f)));
					buffer0.Add (V3ToIntPoint(m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,0,rectangleSize.y)*0.5f)));
					buffer0.Add (V3ToIntPoint(m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,0,rectangleSize.y)*0.5f)));
				} else {
					woffset += center;
					buffer0.Add (V3ToIntPoint(woffset + new Vector3(-rectangleSize.x,0,-rectangleSize.y)*0.5f));
					buffer0.Add (V3ToIntPoint(woffset + new Vector3(rectangleSize.x,0,-rectangleSize.y)*0.5f));
					buffer0.Add (V3ToIntPoint(woffset + new Vector3(rectangleSize.x,0,rectangleSize.y)*0.5f));
					buffer0.Add (V3ToIntPoint(woffset + new Vector3(-rectangleSize.x,0,rectangleSize.y)*0.5f));
				}
				buffer.Add(buffer0);
				break;
			case MeshType.Circle:
				buffer0 = Pathfinding.Util.ListPool<Pathfinding.ClipperLib.IntPoint>.Claim(circleResolution);
				if (useRotation) {
					Matrix4x4 m = tr.localToWorldMatrix;//Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
					for (int i=0;i<circleResolution;i++) {
						buffer0.Add (V3ToIntPoint(m.MultiplyPoint3x4(center + new Vector3(Mathf.Cos((i*2*Mathf.PI)/circleResolution),0,Mathf.Sin((i*2*Mathf.PI)/circleResolution))*circleRadius)));
					}
				} else {
					woffset += center;
					for (int i=0;i<circleResolution;i++) {
						buffer0.Add (V3ToIntPoint(woffset + new Vector3(Mathf.Cos((i*2*Mathf.PI)/circleResolution),0,Mathf.Sin((i*2*Mathf.PI)/circleResolution))*circleRadius));
					}
				}
				buffer.Add(buffer0);
				break;
			case MeshType.CustomMesh:
				if (mesh != lastMesh || contours == null) {
					CalculateMeshContour();
					lastMesh = mesh;
				}
				
				if (contours != null) {
					woffset += center;
					
					bool reverse = Vector3.Dot ( tr.up, Vector3.up ) < 0;
					
					for (int i=0;i<contours.Length;i++) {
						Vector3[] contour = contours[i];
						
						buffer0 = Pathfinding.Util.ListPool<Pathfinding.ClipperLib.IntPoint>.Claim(contour.Length);
						if (useRotation) {
							Matrix4x4 m = tr.localToWorldMatrix;//Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
							for (int x=0;x<contour.Length;x++) {
								buffer0.Add(V3ToIntPoint(m.MultiplyPoint3x4(center + contour[x]*meshScale)));
							}
						} else {
							for (int x=0;x<contour.Length;x++) {
								buffer0.Add(V3ToIntPoint(woffset + contour[x]*meshScale));
							}
						}
						
						if ( reverse ) buffer0.Reverse ();
						
						buffer.Add (buffer0);
					}
				}
				break;
			}
		}
		
		public Pathfinding.ClipperLib.IntPoint V3ToIntPoint (Vector3 p) {
			Int3 ip = (Int3)p;
			return new Pathfinding.ClipperLib.IntPoint(ip.x,ip.z);
		}
		
		public Vector3 IntPointToV3 (Pathfinding.ClipperLib.IntPoint p) {
			Int3 ip = new Int3((int)p.X,0,(int)p.Y);
			return (Vector3)ip;
		}
		
		public static readonly Color GizmoColor = new Color(37.0f/255,184.0f/255,239.0f/255);
		
		public void OnDrawGizmos () {
			
			if (tr == null) tr = transform;
			
			List<List<Pathfinding.ClipperLib.IntPoint>> buffer = Pathfinding.Util.ListPool<List<Pathfinding.ClipperLib.IntPoint>>.Claim();
			GetContour(buffer);
			Gizmos.color = GizmoColor;
			Bounds bounds = GetBounds();
			float ymin = bounds.min.y;
			Vector3 yoffset = Vector3.up * (bounds.max.y - ymin);
			
			for (int i=0;i<buffer.Count;i++) {
				List<Pathfinding.ClipperLib.IntPoint> cont = buffer[i];
				for (int j=0;j<cont.Count;j++) {
					Vector3 p1 = IntPointToV3(cont[j]);
					p1.y = ymin;
					Vector3 p2 = IntPointToV3(cont[(j+1) % cont.Count]);
					p2.y = ymin;
					Gizmos.DrawLine (p1,p2);
					Gizmos.DrawLine (p1+yoffset, p2+yoffset);
					Gizmos.DrawLine (p1, p1+yoffset);
					Gizmos.DrawLine (p2, p2+yoffset);
				}
			}
			
			Pathfinding.Util.ListPool<List<Pathfinding.ClipperLib.IntPoint>>.Release(buffer);
		}
		
		public void OnDrawGizmosSelected () {
			Gizmos.color = Color.Lerp (GizmoColor, new Color (1,1,1,0.2f), 0.9f);
				
			Bounds b = GetBounds ();
			Gizmos.DrawCube (b.center, b.size);
			Gizmos.DrawWireCube (b.center, b.size);
		}
		
	}
}