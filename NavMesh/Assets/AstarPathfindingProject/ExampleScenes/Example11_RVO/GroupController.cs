using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.RVO;
using Pathfinding.RVO.Sampled;

/** RVO Example Scene Unit Controller.
 * Controls AIs and camera in the RVO example scene.
 */
public class GroupController : MonoBehaviour {
	
	public GUIStyle selectionBox;
	public bool adjustCamera = true;
	
	Vector2 start, end;
	bool wasDown = false;
	
	
	List<RVOExampleAgent> selection = new List<RVOExampleAgent>();
	
	Simulator sim;
	
	Camera cam;
	
	public void Start () {
		cam = Camera.main;
		var simu = FindObjectOfType(typeof(RVOSimulator)) as RVOSimulator;
		if ( simu == null ) {
			this.enabled = false;
			throw new System.Exception ("No RVOSimulator in the scene. Please add one");
		}

		sim = simu.GetSimulator();
	}
	
	public void Update () {
		if (Screen.fullScreen && Screen.width != Screen.resolutions[Screen.resolutions.Length-1].width) {
			Screen.SetResolution (Screen.resolutions[Screen.resolutions.Length-1].width,Screen.resolutions[Screen.resolutions.Length-1].height,true);
		}
		
		if (adjustCamera) {
			//Adjust camera
			List<Agent> agents = sim.GetAgents ();
			
			float max = 0;
			for (int i=0;i<agents.Count;i++) {
				float d = Mathf.Max(Mathf.Abs(agents[i].InterpolatedPosition.x), Mathf.Abs(agents[i].InterpolatedPosition.z));
				if (d > max) {
					max = d;
				}
			}
			
			float hh = max / Mathf.Tan ((cam.fieldOfView*Mathf.Deg2Rad/2.0f));
			float hv = max / Mathf.Tan (Mathf.Atan(Mathf.Tan(cam.fieldOfView*Mathf.Deg2Rad/2.0f)*cam.aspect));
			
			cam.transform.position = Vector3.Lerp (cam.transform.position, new Vector3 (0,Mathf.Max(hh,hv)*1.1f,0), Time.smoothDeltaTime*2);
		}
		
		if (Input.GetKey (KeyCode.A) && Input.GetKeyDown (KeyCode.Mouse0)) {
			Order ();
		}
	}
	
	// Update is called once per frame
	void OnGUI () {
		
		if (Event.current.type == EventType.MouseUp	&& Event.current.button == 0 && !Input.GetKey(KeyCode.A)) {
			Select (start, end);
			wasDown = false;
		}
		
		if (Event.current.type == EventType.MouseDrag && Event.current.button == 0) {
			end = Event.current.mousePosition;
			
			if (!wasDown) { start = end; wasDown = true; }
		}
		
		if (Input.GetKey(KeyCode.A)) wasDown = false;
		if (wasDown) {
			Rect r = Rect.MinMaxRect (Mathf.Min (start.x,end.x),Mathf.Min(start.y,end.y),Mathf.Max(start.x,end.x),Mathf.Max(start.y,end.y));
			if (r.width	> 4 && r.height	> 4)
				GUI.Box (r,"",selectionBox);
		}
	}
	
	public void Order () {
		
		Ray ray = cam.ScreenPointToRay (Input.mousePosition);
		RaycastHit hit;
		if (Physics.Raycast (ray, out hit)) {
			
			float radsum = 0;
			for (int i=0;i<selection.Count;i++) radsum += selection[i].GetComponent<RVOController>().radius;
			
			float radius = radsum / (Mathf.PI);
			radius *= 2f;
			
			for (int i=0;i<selection.Count;i++) {
				float deg = 2*Mathf.PI*i/selection.Count;
				Vector3 p = hit.point + new Vector3 (Mathf.Cos(deg), 0,Mathf.Sin (deg))*radius;
				//Debug.DrawRay (p,Vector3.up*4,Color.cyan);
				//Debug.Break();
				selection[i].SetTarget (p);
				selection[i].SetColor (GetColor (deg));
				selection[i].RecalculatePath ();
			}
		}
		
	}
	
	public void Select (Vector2 _start, Vector2 _end) {
		_start.y = Screen.height - _start.y;
		_end.y = Screen.height - _end.y;
		
		Vector2 start = Vector2.Min (_start,_end);
		Vector2 end = Vector2.Max (_start,_end);
		
		if ((end-start).sqrMagnitude < 4*4) return;
		
		selection.Clear ();
		
		RVOExampleAgent[] rvo = FindObjectsOfType (typeof(RVOExampleAgent)) as RVOExampleAgent[];
		for (int i=0;i<rvo.Length;i++) {
			Vector2 sp = cam.WorldToScreenPoint(rvo[i].transform.position);
			if (sp.x > start.x && sp.y > start.y && sp.x < end.x && sp.y < end.y) {
				selection.Add (rvo[i]);
			}
		}
	}
				
	/** Radians to degrees constant */
	const float rad2Deg = 360.0f/ ((float)System.Math.PI*2);
	
	/** Color from an angle */
	public Color GetColor (float angle) {
		return HSVToRGB (angle * rad2Deg, 0.8f, 0.6f);
	}
	
	/**
	 * Converts an HSV color to an RGB color.
	 * According to the algorithm described at http://en.wikipedia.org/wiki/HSL_and_HSV
	 * 
	 * @author Wikipedia
	 * @return the RGB representation of the color.
	 */
	static Color HSVToRGB(float h, float s, float v)
	{
		float Min;
		float Chroma;
		float Hdash;
		float X;
		float r = 0,g = 0,b = 0;
	 
		Chroma = s * v;
		Hdash = h / 60.0f;
		X = Chroma * (1.0f - System.Math.Abs((Hdash % 2.0f) - 1.0f));
	 
		if(Hdash < 1.0f)
		{
			r = Chroma;
			g = X;
		}
		else if(Hdash < 2.0f)
		{
			r = X;
			g = Chroma;
		}
		else if(Hdash < 3.0f)
		{
			g = Chroma;
			b = X;
		}
		else if(Hdash < 4.0f)
		{
			g= X;
			b = Chroma;
		}
		else if(Hdash < 5.0f)
		{
			r = X;
			b = Chroma;
		}
		else if(Hdash < 6.0f)
		{
			r = Chroma;
			b = X;
		}
	 
		Min = v - Chroma;
	 
		r += Min;
		g += Min;
		b += Min;
	 
		return new Color (r,g,b);
	}
}
