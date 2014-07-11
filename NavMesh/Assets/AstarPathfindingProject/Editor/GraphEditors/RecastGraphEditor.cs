
#if UNITY_4_0 || UNITY_4_1 || UNITY_4_2 || UNITY_4_3 || UNITY_4_4 || UNITY_4_5 || UNITY_4_6 || UNITY_4_7 || UNITY_4_8 || UNITY_4_9
#define UNITY_4
#endif

using UnityEngine;
using UnityEditor;
using System.Collections;
using Pathfinding;

namespace Pathfinding {
	[CustomGraphEditor (typeof(RecastGraph),"RecastGraph")]
	/** \astarpro */
	public class RecastGraphEditor : GraphEditor {
	#if UNITY_3_5
		public GameObject meshRenderer;
		public MeshFilter meshFilter;
	#endif
		public Mesh navmeshRender;
		public Renderer navmeshRenderer;
		
		/** Material to use for navmeshes in the editor */
		public static Material navmeshMaterial;
		public static bool tagMaskFoldout = false;
		
		public override void OnEnable () {
	#if UNITY_3_5
			CreateDebugMesh ();
	#else
			UpdateDebugMesh (editor.script);
	#endif
			
			//Get a callback when scanning has finished
			AstarPath.OnLatePostScan += UpdateDebugMesh;
		}
		
		public override void OnDestroy () {
	#if UNITY_3_5
			if (meshRenderer != null) {
				GameObject.DestroyImmediate (meshRenderer);
			}
	#else
			//Avoid memory leak
			Mesh.DestroyImmediate (navmeshRender);
	#endif
		}
		
		public override void OnDisable () {
			AstarPath.OnLatePostScan -= UpdateDebugMesh;
			
	#if UNITY_3_5
			if (meshRenderer != null) {
				//GameObject.DestroyImmediate (meshRenderer);
			}
	#endif
		}
		
	#if UNITY_3_5
		public void CreateDebugMesh () {
			RecastGraph graph = target as RecastGraph;
			
			meshRenderer = GameObject.Find ("RecastGraph_"+graph.guid.ToString ());
			
			if (meshRenderer == null || meshFilter == null || navmeshRender == null || navmeshRenderer == null) {
				
				if (meshRenderer == null) {
					meshRenderer = new GameObject ("RecastGraph_"+graph.guid.ToString ());
					meshRenderer.hideFlags = /*HideFlags.NotEditable |*/ HideFlags.DontSave;
				}
				
				if (meshRenderer.GetComponent<NavMeshRenderer>() == null) {
					meshRenderer.AddComponent<NavMeshRenderer>();
				}
				
				MeshFilter filter;
				if ((filter = meshRenderer.GetComponent<MeshFilter>()) == null) {
					filter = meshRenderer.AddComponent<MeshFilter>();
				}
				
				navmeshRenderer = meshRenderer.GetComponent<MeshRenderer>();
				if (navmeshRenderer == null) {
					navmeshRenderer = meshRenderer.AddComponent<MeshRenderer>();
					navmeshRenderer.castShadows = false;
					navmeshRenderer.receiveShadows = false;
				}
				
				if (filter.sharedMesh == null) {
					navmeshRender = new Mesh ();
					filter.sharedMesh = navmeshRender;
				} else {
					navmeshRender = filter.sharedMesh;
				}
				
				navmeshRender.name = "Navmesh_"+graph.guid.ToString ();
			}
			
			if (navmeshMaterial == null) {
				navmeshMaterial = AssetDatabase.LoadAssetAtPath (AstarPathEditor.editorAssets + "/Materials/Navmesh.mat",typeof(Material)) as Material;
				if (navmeshMaterial == null) {
					Debug.LogWarning ("Could not find navmesh material at path "+AstarPathEditor.editorAssets + "/Materials/Navmesh.mat");
				}
				navmeshRenderer.material = navmeshMaterial;
			}
		}
	#endif
		
		public void UpdateDebugMesh (AstarPath astar) {
	#if UNITY_3_5
			CreateDebugMesh ();
			
			meshRenderer.transform.position = Vector3.zero;
			meshRenderer.transform.localScale = Vector3.one;
	#endif
			
			/*
			RecastGraph graph = target as RecastGraph;
			
			if (graph != null && graph.nodes != null && graph.vectorVertices != null) {
				if (navmeshRender == null) navmeshRender = new Mesh();
				
				navmeshRender.Clear ();
				
				navmeshRender.vertices = graph.vectorVertices;
				
				int[] tris = new int[graph.nodes.Length*3];
				Color[] vColors = new Color[graph.vectorVertices.Length];
				
				for (int i=0;i<graph.nodes.Length;i++) {
					TriangleMeshNode node = graph.nodes[i] as TriangleMeshNode;
					tris[i*3] = node.v0;
					tris[i*3+1] = node.v1;
					tris[i*3+2] = node.v2;
					Color col = Mathfx.IntToColor ((int)node.Region,1F);
					vColors[node.v0] = col;
					vColors[node.v1] = col;
					vColors[node.v2] = col;
				}
				navmeshRender.triangles = tris;
				navmeshRender.colors = vColors;
				
				//meshRenderer.transform.position = graph.forcedBoundsCenter-graph.forcedBoundsSize*0.5F;
				//meshRenderer.transform.localScale = Int3.Precision*Voxelize.CellScale;
				navmeshRender.RecalculateNormals ();
				navmeshRender.RecalculateBounds ();
				
				if (navmeshMaterial == null) {
					navmeshMaterial = AssetDatabase.LoadAssetAtPath (AstarPathEditor.editorAssets + "/Materials/Navmesh.mat",typeof(Material)) as Material;
				}
				
				navmeshRender.hideFlags = HideFlags.HideAndDontSave;
				
	#if UNITY_3_5
				navmeshRenderer.material = navmeshMaterial;
	#endif
			}*/
		}
		
		public override void OnSceneGUI (NavGraph target) {
	#if UNITY_3_5
			if (navmeshRenderer != null) {
				navmeshRenderer.enabled = editor.script.showNavGraphs;
			}
	#else
			/*if (editor.script.showNavGraphs) {
				//navmeshMaterial, 0
				if (navmeshRender != null && navmeshMaterial != null) { 
					//Render the navmesh-mesh. The shader has three passes
					if (navmeshMaterial.SetPass (0)) Graphics.DrawMeshNow (navmeshRender, Matrix4x4.identity);
					if (navmeshMaterial.SetPass (1)) Graphics.DrawMeshNow (navmeshRender, Matrix4x4.identity);
					if (navmeshMaterial.SetPass (2)) Graphics.DrawMeshNow (navmeshRender, Matrix4x4.identity);
					
				}
			}*/
	#endif
		}
		
		public enum UseTiles {
			UseTiles = 0,
			DontUseTiles = 1
		}
		
		public override void OnDrawGizmos () {
		}
		public override void OnInspectorGUI (NavGraph target) {
			RecastGraph graph = target as RecastGraph;
			
			bool preEnabled = GUI.enabled;
			//if (graph.forceBounds) {
			
			System.Int64 estWidth = Mathf.RoundToInt (Mathf.Ceil (graph.forcedBoundsSize.x / graph.cellSize));
			System.Int64 estDepth = Mathf.RoundToInt (Mathf.Ceil (graph.forcedBoundsSize.z / graph.cellSize));
			
			if (estWidth*estDepth >= 1024*1024 || estDepth >= 1024*1024 || estWidth >= 1024*1024) {
				GUIStyle helpBox = GUI.skin.FindStyle ("HelpBox");
				if (helpBox == null) helpBox = GUI.skin.FindStyle ("Box");
				
				Color preColor = GUI.color;
				if (estWidth*estDepth >= 2048*2048 || estDepth >= 2048*2048 || estWidth >= 2048*2048) {
					GUI.color = Color.red;
				} else {
					GUI.color = Color.yellow;
				}
				
				GUILayout.Label ("Warning : Might take some time to calculate",helpBox);
				GUI.color = preColor;
			}
			
			GUI.enabled = false;
			EditorGUILayout.LabelField ("Width (samples)",estWidth.ToString ());
			
			EditorGUILayout.LabelField ("Depth (samples)",estDepth.ToString ());
			/*} else {
				GUI.enabled = false;
				EditorGUILayout.LabelField ("Width (samples)","undetermined");
				EditorGUILayout.LabelField ("Depth (samples)","undetermined");
			}*/
			GUI.enabled = preEnabled;
			
			graph.cellSize = EditorGUILayout.FloatField (new GUIContent ("Cell Size","Size of one voxel in world units"),graph.cellSize);
			if (graph.cellSize < 0.001F) graph.cellSize = 0.001F;
			
			graph.cellHeight = EditorGUILayout.FloatField (new GUIContent ("Cell Height","Height of one voxel in world units"),graph.cellHeight);
			if (graph.cellHeight < 0.001F) graph.cellHeight = 0.001F;
			
			graph.useTiles = (UseTiles)EditorGUILayout.EnumPopup ("Use Tiled Graph", graph.useTiles?UseTiles.UseTiles:UseTiles.DontUseTiles) == UseTiles.UseTiles;
			
			EditorGUI.BeginDisabledGroup (!graph.useTiles);
			
			graph.editorTileSize = EditorGUILayout.IntField (new GUIContent ("Tile Size", "Size in voxels of a single tile.\n" +
			 "This is the width of the tile.\n" +
			 "\n" +
			 "A large tile size can be faster to initially scan (but beware of out of memory issues if you try with a too large tile size in a large world)\n" +
			 "smaller tile sizes are (much) faster to update.\n" +
			 "\n" +
			 "Different tile sizes can affect the quality of paths. It is often good to split up huge open areas into several tiles for\n" +
			 "better quality paths, but too small tiles can lead to effects looking like invisible obstacles."), graph.editorTileSize);
			
			EditorGUI.EndDisabledGroup ();
			
			graph.minRegionSize = EditorGUILayout.FloatField (new GUIContent ("Min Region Size", "Small regions will be removed. In square world units"), graph.minRegionSize);
			
			graph.walkableHeight = EditorGUILayout.FloatField (new GUIContent ("Walkable Height","Minimum distance to the roof for an area to be walkable"),graph.walkableHeight);
			graph.walkableClimb = EditorGUILayout.FloatField (new GUIContent ("Walkable Climb","How high can the character climb"),graph.walkableClimb);
			graph.characterRadius = EditorGUILayout.FloatField (new GUIContent ("Character Radius","Radius of the character, it's good to add some margin though"),graph.characterRadius);
			
			graph.maxSlope = EditorGUILayout.Slider (new GUIContent ("Max Slope","Approximate maximum slope"),graph.maxSlope,0F,90F);
			graph.maxEdgeLength = EditorGUILayout.FloatField (new GUIContent ("Max Edge Length","Maximum length of one edge in the completed navmesh before it is split. A lower value can often yield better quality graphs"),graph.maxEdgeLength);
			graph.maxEdgeLength = graph.maxEdgeLength < graph.cellSize ? graph.cellSize : graph.maxEdgeLength;
			
			graph.contourMaxError = EditorGUILayout.FloatField (new GUIContent ("Max Edge Error","Amount of simplification to apply to edges"),graph.contourMaxError);
			
			graph.rasterizeTerrain = EditorGUILayout.Toggle (new GUIContent ("Rasterize Terrain","Should a rasterized terrain be included"), graph.rasterizeTerrain);
			if (graph.rasterizeTerrain) {
				EditorGUI.indentLevel++;
				graph.rasterizeTrees = EditorGUILayout.Toggle (new GUIContent ("Rasterize Trees", "Rasterize tree colliders on terrains. " +
					"If the tree prefab has a collider, that collider will be rasterized. " +
					"Otherwise a simple box collider will be used and the script will " +
					"try to adjust it to the tree's scale, it might not do a very good job though so " +
					"an attached collider is preferable."), graph.rasterizeTrees);
				if (graph.rasterizeTrees) {
					EditorGUI.indentLevel++;
					graph.colliderRasterizeDetail = EditorGUILayout.FloatField (new GUIContent ("Collider Detail", "Controls the detail of the generated collider meshes. Increasing does not necessarily yield better navmeshes, but lowering will speed up scan"), graph.colliderRasterizeDetail);
					EditorGUI.indentLevel--;
				}
				
				graph.terrainSampleSize = EditorGUILayout.IntField (new GUIContent ("Terrain Sample Size","Size of terrain samples. A lower value is better, but slower"), graph.terrainSampleSize);
				graph.terrainSampleSize = graph.terrainSampleSize < 1 ? 1 : graph.terrainSampleSize;//Clamp to at least 1
				EditorGUI.indentLevel--;
			}
			
			graph.rasterizeMeshes = EditorGUILayout.Toggle (new GUIContent ("Rasterize Meshes", "Should meshes be rasterized and used for building the navmesh"), graph.rasterizeMeshes);
			graph.rasterizeColliders = EditorGUILayout.Toggle (new GUIContent ("Rasterize Colliders", "Should colliders be rasterized and used for building the navmesh"), graph.rasterizeColliders);
			if (graph.rasterizeColliders) {
				EditorGUI.indentLevel++;
				graph.colliderRasterizeDetail = EditorGUILayout.FloatField (new GUIContent ("Collider Detail", "Controls the detail of the generated collider meshes. Increasing does not necessarily yield better navmeshes, but lowering will speed up scan"), graph.colliderRasterizeDetail);
				EditorGUI.indentLevel--;
			}
			
	
			Separator ();
			
			graph.forcedBoundsCenter = EditorGUILayout.Vector3Field ("Center",graph.forcedBoundsCenter);
			graph.forcedBoundsSize = EditorGUILayout.Vector3Field ("Size",graph.forcedBoundsSize);
			
			if (GUILayout.Button (new GUIContent ("Snap bounds to scene","Will snap the bounds of the graph to exactly contain all active meshes in the scene"))) {
				graph.SnapForceBoundsToScene ();
				GUI.changed = true;
			}
			
			Separator ();
	
	#if UNITY_4
			EditorGUILayout.HelpBox ("Objects contained in any of these masks will be taken into account.",MessageType.None);
	#endif
			graph.mask = EditorGUILayoutx.LayerMaskField ("Layer Mask",graph.mask);
			tagMaskFoldout = EditorGUILayoutx.UnityTagMaskList (new GUIContent("Tag Mask"), tagMaskFoldout, graph.tagMask);
			
			Separator ();
			
			graph.showMeshOutline = EditorGUILayout.Toggle (new GUIContent ("Show mesh outline","Toggles gizmos for drawing an outline of the mesh"),graph.showMeshOutline);
			graph.showNodeConnections = EditorGUILayout.Toggle (new GUIContent ("Show node connections","Toggles gizmos for drawing node connections"),graph.showNodeConnections);
			
			if (GUILayout.Button ("Export to .obj file")) {
				ExportToFile (graph);
			}
			
			
			Separator ();
			GUILayout.Label (new GUIContent ("Advanced"), EditorStyles.boldLabel);
			
			graph.relevantGraphSurfaceMode = (RecastGraph.RelevantGraphSurfaceMode)EditorGUILayout.EnumPopup (new GUIContent ("Relevant Graph Surface Mode", 
				"Require every region to have a RelevantGraphSurface component inside it.\n" +
				"A RelevantGraphSurface component placed in the scene specifies that\n" +
				"the navmesh region it is inside should be included in the navmesh.\n\n" +
				"If this is set to OnlyForCompletelyInsideTile\n" +
				"a navmesh region is included in the navmesh if it\n" +
				"has a RelevantGraphSurface inside it, or if it\n" +
				"is adjacent to a tile border. This can leave some small regions\n" +
				"which you didn't want to have included because they are adjacent\n" +
				"to tile borders, but it removes the need to place a component\n" +
				"in every single tile, which can be tedious (see below).\n\n" +
				"If this is set to RequireForAll\n" +
				"a navmesh region is included only if it has a RelevantGraphSurface\n" +
				"inside it. Note that even though the navmesh\n" +
				"looks continous between tiles, the tiles are computed individually\n" +
				"and therefore you need a RelevantGraphSurface component for each\n" +
				"region and for each tile."),
				graph.relevantGraphSurfaceMode);
			
			graph.nearestSearchOnlyXZ = EditorGUILayout.Toggle (new GUIContent ("Nearest node queries in XZ space",
				"Recomended for single-layered environments.\nFaster but can be inacurate esp. in multilayered contexts."), graph.nearestSearchOnlyXZ);
			//graph.mask = 1 << EditorGUILayout.LayerField ("Mask",(int)Mathf.Log (graph.mask,2));
		}
		
		/** Exports the INavmesh graph to a file */
		public void ExportToFile (RecastGraph target) {
			
			//INavmesh graph = (INavmesh)target;
			if (target == null) return;
			
			RecastGraph.NavmeshTile[] tiles = target.GetTiles();
			
			if (tiles == null) {
				if (EditorUtility.DisplayDialog	 ("Scan graph before exporting?","The graph does not contain any mesh data. Do you want to scan it?","Ok","Cancel")) {
					AstarPathEditor.MenuScan ();
					tiles = target.GetTiles();
					if (tiles == null) return;
				} else {
					return;
				}
			}
			
			string path = EditorUtility.SaveFilePanel ("Export .obj","","navmesh.obj","obj");
			if (path == "") return;
			
			//Generate .obj
			System.Text.StringBuilder sb = new System.Text.StringBuilder();
			
			string name = System.IO.Path.GetFileNameWithoutExtension (path);
			
			sb.Append ("g ").Append(name).AppendLine();
			
			//Vertices start from 1
			int vCount = 1;
			
			//Define single texture coordinate to zero
			sb.Append ("vt 0 0\n");
			
			for (int t=0;t<tiles.Length;t++) {
				RecastGraph.NavmeshTile tile = tiles[t];
				
				if (tile == null) continue;
				
				Int3[] vertices = tile.verts;
				
				//Write vertices	
				for (int i=0;i<vertices.Length;i++) {
					Vector3 v = (Vector3)vertices[i];
					sb.Append(string.Format("v {0} {1} {2}\n",-v.x,v.y,v.z));
				}
				
				//Write triangles
				TriangleMeshNode[] nodes = tile.nodes;
				for (int i=0;i<nodes.Length;i++) {
					TriangleMeshNode node = nodes[i];
					if (node == null) {
						Debug.LogError ("Node was null or no TriangleMeshNode. Critical error. Graph type " + target.GetType().Name);
						return;
					}
					if (node.GetVertexArrayIndex(0) < 0 || node.GetVertexArrayIndex(0) >= vertices.Length) throw new System.Exception ("ERR");
					
					sb.Append(string.Format("f {0}/1 {1}/1 {2}/1\n", (node.GetVertexArrayIndex(0) + vCount),(node.GetVertexArrayIndex(1) + vCount),(node.GetVertexArrayIndex(2) + vCount)));
				}
				
				vCount += vertices.Length;
			}
			
			string obj = sb.ToString();
			
			using (System.IO.StreamWriter sw = new System.IO.StreamWriter(path)) 
			{
				sw.Write(obj);
			}
		}
	}
}