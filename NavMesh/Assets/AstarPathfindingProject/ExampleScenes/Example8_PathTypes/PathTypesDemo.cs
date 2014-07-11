using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;

/** Demos different path types.
 * This script is an example script demoing a number of different path types included in the project.
 * Since only the Pro version has access to many path types, it is only included in the pro version
 * \astarpro
 * 
 */
public class PathTypesDemo : MonoBehaviour {
	public int activeDemo = 0;
	
	public Transform start;
	public Transform end;
	
	public Vector3 pathOffset;
	
	public Material lineMat;
	public Material squareMat;
	public float lineWidth;
	
	public RichAI[] agents;
	
	public int searchLength = 1000;
	public int spread = 100;
	public float aimStrength = 0;
	//public LineRenderer lineRenderer;
	
	private Path lastPath = null;
	
	List<GameObject> lastRender = new List<GameObject>();
	
	List<Vector3> multipoints = new List<Vector3>();
	
	Vector2 mouseDragStart;
	float mouseDragStartTime;
	
	// Update is called once per frame
	void Update () {
		
		Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
		
		Vector3 zeroIntersect = ray.origin + ray.direction * (ray.origin.y / -ray.direction.y);
		end.position = zeroIntersect;
		
		if (Input.GetMouseButtonDown(0)) {
			mouseDragStart = Input.mousePosition;
			mouseDragStartTime = Time.realtimeSinceStartup;
		}
		
		if (Input.GetMouseButtonUp(0)) {
			Vector2 mouseDragEnd = Input.mousePosition;
			if ((mouseDragEnd - mouseDragStart).sqrMagnitude > 5*5 && (Time.realtimeSinceStartup - mouseDragStartTime) > 0.3f) {
				//Select
				
				Rect r = Rect.MinMaxRect(
					Mathf.Min(mouseDragStart.x,mouseDragEnd.x),
					Mathf.Min(mouseDragStart.y,mouseDragEnd.y),
					Mathf.Max(mouseDragStart.x,mouseDragEnd.x),
					Mathf.Max(mouseDragStart.y,mouseDragEnd.y)
				);
				
				RichAI[] ais = GameObject.FindObjectsOfType (typeof(RichAI)) as RichAI[];
				List<RichAI> ls = new List<RichAI>();
				for (int i=0;i<ais.Length;i++) {
					Vector2 p = Camera.main.WorldToScreenPoint (ais[i].transform.position);
					//p.y = Screen.height - p.y;
					if (r.Contains (p)) {
						ls.Add (ais[i]);
					}
				}
				agents = ls.ToArray();
			} else {
				
				if (Input.GetKey (KeyCode.LeftShift)) {
					
					multipoints.Add (zeroIntersect);
					
				}
				
				if (Input.GetKey (KeyCode.LeftControl)) {
					multipoints.Clear ();
				}
				
				if (Input.mousePosition.x > 225) {
					DemoPath ();
				}
			}
		}
		
		if (Input.GetMouseButton (0) && Input.GetKey (KeyCode.LeftAlt) && lastPath.IsDone()) {
			DemoPath ();
		}
		
		
	}
	
	/** Draw some helpful gui */
	public void OnGUI () {
		GUILayout.BeginArea (new Rect (5,5,220,Screen.height-10),"","Box");
		
		switch (activeDemo) {
		case 0:
			GUILayout.Label ("Basic path. Finds a path from point A to point B."); break;
		case 1:
			GUILayout.Label ("Multi Target Path. Finds a path quickly from one point to many others in a single search."); break;
		case 2:
			GUILayout.Label ("Randomized Path. Finds a path with a specified length in a random direction or biased towards some point when using a larger aim strenggth."); break;
		case 3:
			GUILayout.Label ("Flee Path. Tries to flee from a specified point. Remember to set Flee Strength!"); break;
		case 4:
			GUILayout.Label ("Finds all nodes which it costs less than some value to reach."); break;
		case 5:
			GUILayout.Label ("Searches the whole graph from a specific point. FloodPathTracer can then be used to quickly find a path to that point"); break;
		case 6:
			GUILayout.Label ("Traces a path to where the FloodPath started. Compare the claculation times for this path with ABPath!\nGreat for TD games"); break;
		}
		
		GUILayout.Space (5);
		
		GUILayout.Label ("Note that the paths are rendered without ANY post-processing applied, so they might look a bit edgy");
		
		GUILayout.Space (5);
		
		GUILayout.Label ("Click anywhere to recalculate the path. Hold Alt to continuously recalculate the path while the mouse is pressed.");
		
		if (activeDemo == 2 || activeDemo == 3 || activeDemo == 4) {
			GUILayout.Label ("Search Distance ("+searchLength+")");
			searchLength = Mathf.RoundToInt (GUILayout.HorizontalSlider (searchLength,0,100000));
		}
		
		if (activeDemo == 2 || activeDemo == 3) {
			GUILayout.Label ("Spread ("+spread+")");
			spread = Mathf.RoundToInt (GUILayout.HorizontalSlider (spread,0,40000));
			
			GUILayout.Label ((activeDemo == 2 ? "Aim strength" : "Flee strength") + " ("+aimStrength+")");
			aimStrength = GUILayout.HorizontalSlider (aimStrength,0,1);
		}
		
		if (activeDemo == 1) {
			GUILayout.Label ("Hold shift and click to add new target points. Hold ctr and click to remove all target points");
		}
		
		if (GUILayout.Button ("A to B path")) activeDemo = 0;
		if (GUILayout.Button ("Multi Target Path")) activeDemo = 1;
		if (GUILayout.Button ("Random Path")) activeDemo = 2;
		if (GUILayout.Button ("Flee path")) activeDemo = 3;
		if (GUILayout.Button ("Constant Path")) activeDemo = 4;
		if (GUILayout.Button ("Flood Path")) activeDemo = 5;
		if (GUILayout.Button ("Flood Path Tracer")) activeDemo = 6;
		
		GUILayout.EndArea ();
	}
	
	/** Get the path back */
	public void OnPathComplete (Path p) {
		//System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch ();
		//watch.Start ();
		
		//To prevent it from creating new GameObjects when the application is quitting when using multithreading.
		if(lastRender == null) return;
		
		if (p.error) {
			ClearPrevious ();
			return;
		}
		
		
		if (p.GetType () == typeof (MultiTargetPath)) {
			
			List<GameObject> unused = new List<GameObject> (lastRender);
			lastRender.Clear ();
			
			MultiTargetPath mp = p as MultiTargetPath;
			
			for (int i=0;i<mp.vectorPaths.Length;i++) {
				if (mp.vectorPaths[i] == null) continue;
				
				List<Vector3> vpath = mp.vectorPaths[i];
				
				GameObject ob = null;
				if (unused.Count > i && unused[i].GetComponent<LineRenderer>() != null) {
					ob = unused[i];
					unused.RemoveAt (i);
				} else {
					ob = new GameObject ("LineRenderer_"+i,typeof(LineRenderer));
				}
				
				LineRenderer lr = ob.GetComponent<LineRenderer>();
				lr.sharedMaterial = lineMat;
				lr.SetWidth (lineWidth,lineWidth);
				
				lr.SetVertexCount (vpath.Count);
				for (int j=0;j<vpath.Count;j++) {
					lr.SetPosition (j,vpath[j] + pathOffset);
				}
				
				lastRender.Add (ob);
			}
			
			for (int i=0;i<unused.Count;i++) {
				Destroy (unused[i]);
			}
			
		} else if (p.GetType () == typeof (ConstantPath)) {
			
			ClearPrevious ();
			//The following code will build a mesh with a square for each node visited
			
			ConstantPath constPath = p as ConstantPath;
			List<GraphNode> nodes = constPath.allNodes;
			
			Mesh mesh = new Mesh ();
			
			List<Vector3> verts = new List<Vector3>();
			
			bool drawRaysInstead = false;
			
			List<Vector3> pts = Pathfinding.PathUtilities.GetPointsOnNodes (nodes, 20, 0);
			Vector3 avg = Vector3.zero;
			for (int i=0;i<pts.Count;i++) {
				Debug.DrawRay (pts[i], Vector3.up*5, Color.red, 3);
				avg += pts[i];
			}
			
			if (pts.Count > 0) avg /= pts.Count;
			
			for (int i=0;i<pts.Count;i++) {
				pts[i] -= avg;
			}
			
			Pathfinding.PathUtilities.GetPointsAroundPoint (start.position, AstarPath.active.astarData.graphs[0] as IRaycastableGraph, pts, 0, 1);
			
			for (int i=0;i<pts.Count;i++) {
				Debug.DrawRay (pts[i], Vector3.up*5, Color.blue, 3);
			}
			
			//This will loop through the nodes from furthest away to nearest, not really necessary... but why not :D
			//Note that the reverse does not, as common sense would suggest, loop through from the closest to the furthest away
			//since is might contain duplicates and only the node duplicate placed at the highest index is guarenteed to be ordered correctly.
			for (int i=nodes.Count-1;i>=0;i--) {
				
				Vector3 pos = (Vector3)nodes[i].position+pathOffset;
				if (verts.Count	== 65000 && !drawRaysInstead) {
					Debug.LogError ("Too many nodes, rendering a mesh would throw 65K vertex error. Using Debug.DrawRay instead for the rest of the nodes");
					drawRaysInstead = true;
				}
				
				if (drawRaysInstead) {
					Debug.DrawRay (pos,Vector3.up,Color.blue);
					continue;
				}
				
				//Add vertices in a square
				
				GridGraph gg = AstarData.GetGraph (nodes[i]) as GridGraph;
				float scale = 1F;
				
				if (gg != null) scale = gg.nodeSize;
				
				verts.Add (pos+new Vector3 (-0.5F,0,-0.5F)*scale);
				verts.Add (pos+new Vector3 (0.5F,0,-0.5F)*scale);
				verts.Add (pos+new Vector3 (-0.5F,0,0.5F)*scale);
				verts.Add (pos+new Vector3 (0.5F,0,0.5F)*scale);
			}
			
			//Build triangles for the squares
			Vector3[] vs = verts.ToArray ();
			int[] tris = new int[(3*vs.Length)/2];
			for (int i=0, j=0;i<vs.Length;j+=6, i+=4) {
				tris[j+0] = i;
				tris[j+1] = i+1;
				tris[j+2] = i+2;
				
				tris[j+3] = i+1;
				tris[j+4] = i+3;
				tris[j+5] = i+2;
			}
			
			Vector2[] uv = new Vector2[vs.Length];
			//Set up some basic UV
			for (int i=0;i<uv.Length;i+=4) {
				uv[i] = new Vector2(0,0);
				uv[i+1] = new Vector2(1,0);
				uv[i+2] = new Vector2(0,1);
				uv[i+3] = new Vector2(1,1);
			}
			
			mesh.vertices = vs;
			mesh.triangles = tris;
			mesh.uv = uv;
			mesh.RecalculateNormals ();
			
			GameObject go = new GameObject("Mesh",typeof(MeshRenderer),typeof(MeshFilter));
			MeshFilter fi = go.GetComponent<MeshFilter>();
			fi.mesh = mesh;
			MeshRenderer re = go.GetComponent<MeshRenderer>();
			re.material = squareMat;
			
			lastRender.Add (go);
			
		} else {
			
			ClearPrevious ();
			
			GameObject ob = new GameObject ("LineRenderer",typeof(LineRenderer));
			LineRenderer lr = ob.GetComponent<LineRenderer>();
			lr.sharedMaterial = lineMat;
			lr.SetWidth (lineWidth,lineWidth);
			
			lr.SetVertexCount (p.vectorPath.Count);
			for (int i=0;i<p.vectorPath.Count;i++) {
				lr.SetPosition (i,p.vectorPath[i] + pathOffset);
			}
			
			lastRender.Add (ob);
		}
	}
	
	/** Destroys all previous render objects */
	public void ClearPrevious () {
		for (int i=0;i<lastRender.Count;i++) {
			Destroy (lastRender[i]);
		}
		lastRender.Clear ();
	}
	
	/** Clears renders on application quit */
	public void OnApplicationQuit () {
		ClearPrevious ();
		lastRender = null;
	}
	
	private FloodPath lastFlood = null;
	
	/** Starts a path specified by PathTypesDemo.activeDemo */
	public void DemoPath () {
		
		Path p = null;
		
		if (activeDemo == 0) {
			p = ABPath.Construct (start.position,end.position, OnPathComplete);
			
			if (agents != null && agents.Length > 0) {
				List<Vector3> pts = Pathfinding.Util.ListPool<Vector3>.Claim(agents.Length);
				Vector3 avg = Vector3.zero;
				for (int i=0;i<agents.Length;i++) {
					pts.Add (agents[i].transform.position);
					avg += pts[i];
				}
				avg /= pts.Count;
				for (int i=0;i<agents.Length;i++) pts[i] -= avg;
				//List<Vector3> pts = Pathfinding.PathUtilities.GetSpiralPoints (agents.Length, 0.2f);
				
				Pathfinding.PathUtilities.GetPointsAroundPoint (end.position, AstarPath.active.graphs[0] as IRaycastableGraph, pts, 0, 0.2f);
				//for (int i=0;i<pts.Count;i++) pts[i] += end.position;
				for (int i=0;i<agents.Length;i++) {
					if (agents[i] == null) continue;
					
					agents[i].target.position = pts[i];
					agents[i].UpdatePath();
				}
			}
		} else if (activeDemo == 1) {
			MultiTargetPath mp = MultiTargetPath.Construct (multipoints.ToArray (), end.position, null, OnPathComplete);
			p = mp;
		} else if (activeDemo == 2) {
			RandomPath rp = RandomPath.Construct (start.position,searchLength, OnPathComplete);
			rp.spread = spread;
			rp.aimStrength = aimStrength;
			rp.aim = end.position;
			
			p = rp;
		} else if (activeDemo == 3) {
			FleePath fp = FleePath.Construct (start.position, end.position, searchLength, OnPathComplete);
			fp.aimStrength = aimStrength;
			fp.spread = spread;
			
			p = fp;
		} else if (activeDemo == 4) {
			StartCoroutine(Constant());
			p = null;
		} else if (activeDemo == 5) {
			FloodPath fp = FloodPath.Construct (end.position, null);
			lastFlood = fp;
			p = fp;
		} else if (activeDemo == 6 && lastFlood != null) {
			FloodPathTracer fp = FloodPathTracer.Construct (end.position, lastFlood, OnPathComplete);
			
			
			p = fp;
		}
		
		if (p != null) {
			AstarPath.StartPath (p);
			lastPath = p;
		}
	}
	
	public IEnumerator Constant () {
		ConstantPath constPath = ConstantPath.Construct (end.position, searchLength, OnPathComplete);
		AstarPath.StartPath (constPath);
		lastPath = constPath;
		yield return constPath.WaitForPath();
		Debug.Log (constPath.pathID + " " + constPath.allNodes.Count);
	}
}
