
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;

namespace Pathfinding {
	/** Adds new geometry to a recast graph.
	 * 
	 * This component will add new geometry to a recast graph similar
	 * to how a NavmeshCut component removes it.
	 * 
	 * There are quite a few limitations to this component though.
	 * This navmesh geometry will not be connected to the rest of the navmesh
	 * in the same tile unless very exactly positioned so that the 
	 * triangles line up exactly.
	 * It will be connected to neighbouring tiles if positioned so that
	 * it lines up with the tile border.
	 * 
	 * 
	 * This component has a few very specific use-cases.
	 * For example if you have a tiled recast graph
	 * this component could be used to add bridges
	 * in that world.
	 * You would create a NavmeshCut object cutting out a hole for the bridge.
	 * then add a NavmeshAdd object which fills that space.
	 * Make sure NavmeshCut.CutsAddedGeom is disabled on the NavmeshCut, otherwise it will
	 * cut away the NavmeshAdd object.
	 * Then you can add links between the added geometry and the rest of the world, preferably using NodeLink3.
	 */
	public class NavmeshAdd : MonoBehaviour {
	
		public enum MeshType {
			Rectangle,
			CustomMesh
		}
	
		private static List<NavmeshAdd> allCuts = new List<NavmeshAdd>();
	
		private static void Add (NavmeshAdd obj) {
			allCuts.Add (obj);
		}
		
		private static void Remove (NavmeshAdd obj) {
			allCuts.Remove (obj);
		}
		
		/** Get all active instances which intersect the bounds */
		public static List<NavmeshAdd> GetAllInRange(Bounds b) {
			List<NavmeshAdd> cuts = Pathfinding.Util.ListPool<NavmeshAdd>.Claim ();
			for (int i=0;i<allCuts.Count;i++) {
				if (allCuts[i].enabled && Intersects(b, allCuts[i].GetBounds())) {
					cuts.Add (allCuts[i]);
				}
			}
			return cuts;
		}
	
		/** True if \a b1 and \a b2 intersects.
		 * \note This method ignores the Y axis
		 */
		private static bool Intersects (Bounds b1, Bounds b2)        {
			Vector3 min1 = b1.min;
			Vector3 max1 = b1.max;
			Vector3 min2 = b2.min;
			Vector3 max2 = b2.max;
			return min1.x <= max2.x && max1.x >= min2.x && min1.z <= max2.z && max1.z >= min2.z;
		}
	
		/** Returns a list with all NavmeshAdd components in the scene.
		 * \warning Do not modify this array
		 */
		public static List<NavmeshAdd> GetAll () {
			return allCuts;
		}
	
	
		public MeshType type;
		
		/** Custom mesh to use.
		 * The contour(s) of the mesh will be extracted.
		 * If you get the "max perturbations" error when cutting with this, check the normals on the mesh.
		 * They should all point in the same direction. Try flipping them if that does not help.
		 */
		public Mesh mesh;
	
		/** Cached vertices */
		Vector3[] verts;
	
		/** Cached triangles */
		int[] tris;
	
		/** Size of the rectangle */
		public Vector2 rectangleSize = new Vector2(1,1);
	
		public float meshScale = 1;
		
		public Vector3 center;
		
		Bounds bounds;
		
		/** Includes rotation in calculations.
		 * This is slower since a lot more matrix multiplications are needed but gives more flexibility.
		 */
		public bool useRotation = false;
	
		/** cached transform component */
		protected Transform tr;
	
		public void Awake () {
			Add(this);
		}
		
		public void OnEnable () {
			tr = transform;
		}
		
		public void OnDestroy () {
			Remove(this);
		}
	
		public Vector3 Center {
			get {
				return tr.position + (useRotation ? tr.TransformPoint ( center ) : center);
			}
		}
		
		[ContextMenu("Rebuild Mesh")]
		public void RebuildMesh () {
			if ( type == MeshType.CustomMesh) {
				if ( mesh == null ) {
					verts = null;
					tris = null;
				} else {
					verts = mesh.vertices;
					tris = mesh.triangles;
				}
			} else { // Rectangle
				if ( verts == null || verts.Length != 4 || tris == null || tris.Length != 6 ) {
					verts = new Vector3[4];
					tris = new int[6];
				}
				
				tris[0] = 0;
				tris[1] = 1;
				tris[2] = 2;
				tris[3] = 0;
				tris[4] = 2;
				tris[5] = 3;
	
				verts[0] =  new Vector3 (-rectangleSize.x*0.5f, 0, -rectangleSize.y*0.5f);
				verts[1] =  new Vector3 ( rectangleSize.x*0.5f, 0, -rectangleSize.y*0.5f);
				verts[2] =  new Vector3 ( rectangleSize.x*0.5f, 0,  rectangleSize.y*0.5f);
				verts[3] =  new Vector3 (-rectangleSize.x*0.5f, 0,  rectangleSize.y*0.5f);
			}
		}
	
		public Bounds GetBounds () {
			switch (type) {
			case MeshType.Rectangle:
				if (useRotation) {
					Matrix4x4 m = Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one);
					bounds = new Bounds(m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,0,-rectangleSize.y)*0.5f), Vector3.zero);
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,0,-rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(rectangleSize.x,0,rectangleSize.y)*0.5f));
					bounds.Encapsulate (m.MultiplyPoint3x4(center + new Vector3(-rectangleSize.x,0,rectangleSize.y)*0.5f));
				} else {
					bounds = new Bounds(tr.position+center, new Vector3(rectangleSize.x,0,rectangleSize.y));
				}
				break;
			case MeshType.CustomMesh:
				if (mesh == null) break;
				
				Bounds b = mesh.bounds;
				if (useRotation) {
					Matrix4x4 m = Matrix4x4.TRS (tr.position, tr.rotation, Vector3.one * meshScale);
					//b.center *= meshScale;
					//b.size *= meshScale;
					
					bounds = new Bounds ( m.MultiplyPoint3x4 ( center + b.center ), Vector3.zero );
					
					Vector3 mx = b.max;
					Vector3 mn = b.min;
					
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mx.x, mn.y ,mx.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mn.x, mn.y ,mx.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mn.x, mx.y ,mn.z )) );
					bounds.Encapsulate (m.MultiplyPoint3x4 ( center + new Vector3 (mx.x, mx.y ,mn.z )) );
	
				} else {
					Vector3 size = b.size*meshScale;
					bounds = new Bounds(transform.position+center+b.center*meshScale,size);
				}
				break;
			}
			return bounds;
		}
	
		public void GetMesh ( Int3 offset, ref Int3[] vbuffer, out int[] tbuffer ) {
	
			if ( verts == null ) RebuildMesh ();
			
			if ( verts == null ) {
				tbuffer = new int[0];
				return;
			}
			
			if ( vbuffer == null || vbuffer.Length < verts.Length ) vbuffer = new Int3[verts.Length];
			tbuffer = tris;
	
			if ( useRotation ) {
				Matrix4x4 m = Matrix4x4.TRS ( tr.position + center, tr.rotation, tr.localScale * meshScale );
	
				for ( int i=0;i<verts.Length;i++) {
					vbuffer[i] = offset + (Int3)m.MultiplyPoint3x4 ( verts[i] );
				}
			} else {
				Vector3 voffset = tr.position + center;
				for ( int i=0;i<verts.Length;i++) {
					vbuffer[i] = offset + (Int3)(voffset + verts[i]*meshScale);
				}
			}
		}
	
		public static readonly Color GizmoColor = new Color(94.0f/255,239.0f/255,37.0f/255);
	
	#if UNITY_EDITOR
		public static Int3[] gizmoBuffer;
	
		public void OnDrawGizmos () {
			
			if (tr == null) tr = transform;
	
	
			int[] tbuffer;
			GetMesh( Int3.zero, ref gizmoBuffer, out tbuffer);
	
			Gizmos.color = GizmoColor;
	
			for ( int i=0;i<tbuffer.Length; i += 3 ){
				Vector3 v1 = (Vector3)gizmoBuffer[tbuffer[i+0]];
				Vector3 v2 = (Vector3)gizmoBuffer[tbuffer[i+1]];
				Vector3 v3 = (Vector3)gizmoBuffer[tbuffer[i+2]];
	
				Gizmos.DrawLine ( v1, v2 );
				Gizmos.DrawLine ( v2, v3 );
				Gizmos.DrawLine ( v3, v1 );
			}
		}
		
		public void OnDrawGizmosSelected () {
	
			Gizmos.color = Color.Lerp (GizmoColor, new Color (1,1,1,0.2f), 0.9f);
			
			Bounds b = GetBounds ();
			Gizmos.DrawCube (b.center, b.size);
			Gizmos.DrawWireCube (b.center, b.size);
			Debug.Log ( mesh.bounds );
		}
	#endif
	}
}