
//#define ASTARDEBUG

using UnityEngine;
using System.Collections;

namespace Pathfinding {
	/** \astarpro */
	public class DebugUtility : MonoBehaviour {
		
		public Material defaultMaterial;
		
		public new static DebugUtility active;
		
		public float offset = 0.2F;
		
		public bool optimizeMeshes = false;
		
		public void Awake () {
			active = this;
		}
		
		public static void DrawCubes (Vector3[] topVerts, Vector3[] bottomVerts, Color[] vertexColors, float width) {
			
			if (active == null) {
				active = GameObject.FindObjectOfType(typeof(DebugUtility)) as DebugUtility;
			}
			if (active == null) throw new System.NullReferenceException ();
			
			if (topVerts.Length != bottomVerts.Length || topVerts.Length != vertexColors.Length) {
				Debug.LogError ("Array Lengths are not the same");
				return;
			}
			
			//65000 limit divided by 4*6 = 24
			if (topVerts.Length > 2708) {
				Vector3[] topVerts2 = new Vector3[topVerts.Length-2708];
				Vector3[] bottomVerts2 = new Vector3[topVerts.Length-2708];
				Color[] vertexColors2 = new Color[topVerts.Length-2708];
				
				for (int i=2708;i<topVerts.Length;i++) {
					topVerts2[i-2708] = topVerts[i];
					bottomVerts2[i-2708] = bottomVerts[i];
					vertexColors2[i-2708] = vertexColors[i];
				}
				
				Vector3[] topVerts3 = new Vector3[2708];
				Vector3[] bottomVerts3 = new Vector3[2708];
				Color[] vertexColors3 = new Color[2708];
				
				for (int i=0;i<2708;i++) {
					topVerts3[i] = topVerts[i];
					bottomVerts3[i] = bottomVerts[i];
					vertexColors3[i] = vertexColors[i];
				}
				
				DrawCubes (topVerts2,bottomVerts2,vertexColors2, width);
				topVerts = topVerts3;
				bottomVerts = bottomVerts3;
				vertexColors = vertexColors3;
			}
			
			width /= 2F;
			
			Vector3[] vertices = new Vector3[topVerts.Length*4*6];
			int[] tris = new int[topVerts.Length*6*6];
			Color[] colors = new Color[topVerts.Length*4*6];
			
			for (int i=0;i<topVerts.Length;i++) {
				
				Vector3 top = topVerts[i] + new Vector3 (0,active.offset,0);
				Vector3 bottom = bottomVerts[i] - new Vector3 (0,active.offset,0);;
				
				Vector3 top1 = top + new Vector3 (-width,0,-width);
				Vector3 top2 = top + new Vector3 (width,0,-width);
				Vector3 top3 = top + new Vector3 (width,0,width);
				Vector3 top4 = top + new Vector3 (-width,0,width);
				Vector3 bottom1 = bottom + new Vector3 (-width,0,-width);
				Vector3 bottom2 = bottom + new Vector3 (width,0,-width);
				Vector3 bottom3 = bottom + new Vector3 (width,0,width);
				Vector3 bottom4 = bottom + new Vector3 (-width,0,width);
				
				int vIndex = i*4*6;
				
				Color col = vertexColors[i];
				//Color.Lerp (Color.green,Color.red,topVerts[i].y*0.06F);
				//
				
				for (int c=vIndex;c<vIndex+24;c++) {
					colors[c] = col;
				}
				
				//Top
				
				vertices[vIndex] = top1;
				vertices[vIndex+1] = top4;
				vertices[vIndex+2] = top3;
				vertices[vIndex+3] = top2;
				
				int tIndex = i*6*6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
				
				//Bottom
				vIndex += 4;
				vertices[vIndex+3] = bottom1;
				vertices[vIndex+2] = bottom4;
				vertices[vIndex+1] = bottom3;
				vertices[vIndex] = bottom2;
				
				tIndex += 6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
				
				//Right
				vIndex += 4;
				vertices[vIndex] = bottom2;
				vertices[vIndex+1] = top2;
				vertices[vIndex+2] = top3;
				vertices[vIndex+3] = bottom3;
				
				tIndex += 6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
				
				//Left
				vIndex += 4;
				vertices[vIndex+3] = bottom1;
				vertices[vIndex+2] = top1;
				vertices[vIndex+1] = top4;
				vertices[vIndex] = bottom4;
				
				tIndex += 6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
				
				//Forward
				vIndex += 4;
				vertices[vIndex+3] = bottom3;
				vertices[vIndex+2] = bottom4;
				vertices[vIndex+1] = top4;
				vertices[vIndex] = top3;
				
				tIndex += 6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
				
				//Back
				vIndex += 4;
				vertices[vIndex] = bottom2;
				vertices[vIndex+1] = bottom1;
				vertices[vIndex+2] = top1;
				vertices[vIndex+3] = top2;
				
				tIndex += 6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
			}
	
			
			Mesh mesh = new Mesh ();
			mesh.vertices = vertices;
			mesh.triangles = tris;
			mesh.colors = colors;
			
			mesh.name = "VoxelMesh";
			
			mesh.RecalculateNormals ();
			mesh.RecalculateBounds ();
			
			if (active.optimizeMeshes) {
				mesh.Optimize ();
			}
			
			GameObject go = new GameObject ("DebugMesh");
			MeshRenderer rend = go.AddComponent (typeof(MeshRenderer)) as MeshRenderer;
			rend.material = active.defaultMaterial;
			(go.AddComponent (typeof(MeshFilter)) as MeshFilter).mesh = mesh;
		}
		
		public static void DrawQuads (Vector3[] verts, float width) {
			
			//65000 limit divided by 4
			if (verts.Length >= 16250) {
				Vector3[] verts2 = new Vector3[verts.Length-16250];
				
				for (int i=16250;i<verts.Length;i++) {
					verts2[i-16250] = verts[i];
				}
				
				Vector3[] verts3 = new Vector3[16250];
				
				for (int i=0;i<16250;i++) {
					verts3[i] = verts[i];
				}
				DrawQuads (verts2, width);
				verts = verts3;
			}
			
			width /= 2F;
			
			Vector3[] vertices = new Vector3[verts.Length*4];
			int[] tris = new int[verts.Length*6];
			
			for (int i=0;i<verts.Length;i++) {
				Vector3 p = verts[i];
				
				int vIndex = i*4;
				vertices[vIndex] = p + new Vector3 (-width,0,-width);
				vertices[vIndex+1] = p + new Vector3 (-width,0,width);
				vertices[vIndex+2] = p + new Vector3 (width,0,width);
				vertices[vIndex+3] = p + new Vector3 (width,0,-width);
				
				int tIndex = i*6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
			}
			
			Mesh mesh = new Mesh ();
			mesh.vertices = vertices;
			mesh.triangles = tris;
			
			mesh.RecalculateNormals ();
			mesh.RecalculateBounds ();
			
			GameObject go = new GameObject ("DebugMesh");
			MeshRenderer rend = go.AddComponent (typeof(MeshRenderer)) as MeshRenderer;
			rend.material = active.defaultMaterial;
			(go.AddComponent (typeof(MeshFilter)) as MeshFilter).mesh = mesh;
		}
		
		public static void TestMeshLimit () {
			
			Vector3[] vertices = new Vector3[16000*4];
			int[] tris = new int[16000*6];
			
			for (int i=0;i<16000;i++) {
				Vector3 p = Random.onUnitSphere*10;
				
				int vIndex = i*4;
				vertices[vIndex] = p + new Vector3 (-0.1F,0,-0.1F);
				vertices[vIndex+1] = p + new Vector3 (-0.1F,0,0.1F);
				vertices[vIndex+2] = p + new Vector3 (0.1F,0,0.1F);
				vertices[vIndex+3] = p + new Vector3 (0.1F,0,-0.1F);
				
				int tIndex = i*6;
				tris[tIndex] = vIndex;
				tris[tIndex+1] = vIndex+1;
				tris[tIndex+2] = vIndex+2;
				
				tris[tIndex+3] = vIndex;
				tris[tIndex+4] = vIndex+2;
				tris[tIndex+5] = vIndex+3;
			}
			
			Mesh mesh = new Mesh ();
			mesh.vertices = vertices;
			mesh.triangles = tris;
			
			mesh.RecalculateNormals ();
			mesh.RecalculateBounds ();
			
			GameObject go = new GameObject ("DebugMesh");
			go.AddComponent (typeof(MeshRenderer));
			(go.AddComponent (typeof(MeshFilter)) as MeshFilter).mesh = mesh;
		}
	}
}