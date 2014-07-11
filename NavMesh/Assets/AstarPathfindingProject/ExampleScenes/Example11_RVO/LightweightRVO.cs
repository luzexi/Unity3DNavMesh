using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.RVO;

[RequireComponent(typeof(MeshFilter))]
/** Lightweight RVO Circle Example.
 * Lightweight script for simulating agents in a circle trying to reach their antipodal positions.
 * This script, compared to using lots of RVOAgents shows the real power of the RVO simulator when
 * little other overhead (e.g GameObjects) is present.
 * 
 * For example with this script, I can simulate 5000 agents at around 50 fps on my laptop (with desired simulation fps = 10 and interpolation, 2 threads)
 * however when using prefabs, only instantiating the 5000 agents takes 10 seconds and it runs at around 5 fps.
 * 
 * This script will render the agents by generating a square for each agent combined into a single mesh with appropriate UV.
 * 
 * A few GUI buttons will be drawn by this script with which the user can change the number of agents.
 */
public class LightweightRVO : MonoBehaviour {
	
	/** Number of agents created at start */
	public int agentCount = 100;
	
	/** How large is the area where agents are placed.
	  * For e.g the circle example, it corresponds*/
	public float exampleScale = 100;
	
	
	public enum RVOExampleType {
		Circle,
		Line,
		Point,
		RandomStreams
	}
	
	public RVOExampleType type = RVOExampleType.Circle;
	
	/** Agent radius */
	public float radius = 3;
	
	/** Max speed for an agent */
	public float maxSpeed = 2;
	
	/** How far in the future too look for agents */
	public float agentTimeHorizon = 10;

	[HideInInspector]
	/** How far in the future too look for obstacles */
	public float obstacleTimeHorizon = 10;

	/** Max number of neighbour agents to take into account */
	public int maxNeighbours = 10;

	/** Max distance for other agents to take them into account */
	public float neighbourDist = 15;
	
	/** Offset from the agent position the actual drawn postition.
	  * Used to get rid of z-buffer issues */
	public Vector3 renderingOffset = Vector3.up*0.1f;
	
	/** Mesh for rendering */
	Mesh mesh;
	
	/** Reference to the simulator in the scene */
	Pathfinding.RVO.Simulator sim;
	
	/** All agents handled by this script */
	List<IAgent> agents;
	
	/** Goals for each agent */
	List<Vector3> goals;
	
	/** Color for each agent */
	List<Color> colors;
	
	Vector3[] verts;
	Vector2[] uv;
	int[] tris;
	Color[] meshColors;
	Vector3[] interpolatedVelocities;

	public void Start () {
		mesh = new Mesh();
		RVOSimulator rvoSim = FindObjectOfType (typeof(RVOSimulator)) as RVOSimulator;
		if (rvoSim == null) {
			Debug.LogError ("No RVOSimulator could be found in the scene. Please add a RVOSimulator component to any GameObject");
			return;
		}
		sim = rvoSim.GetSimulator();
		GetComponent<MeshFilter>().mesh = mesh;
		
		CreateAgents (agentCount);
	}
	
	public void OnGUI () {
		if (GUILayout.Button ("2")) CreateAgents (2);
		if (GUILayout.Button ("10")) CreateAgents (10);
		if (GUILayout.Button ("100")) CreateAgents (100);
		if (GUILayout.Button ("500")) CreateAgents (500);
		if (GUILayout.Button ("1000")) CreateAgents (1000);
		if (GUILayout.Button ("5000")) CreateAgents (5000);

		GUILayout.Space (5);

		if (GUILayout.Button ("Random Streams")) {
			type = RVOExampleType.RandomStreams;
			CreateAgents ( agents != null ? agents.Count : 100 );
		}

		if (GUILayout.Button ("Line")) {
			type = RVOExampleType.Line;
			CreateAgents ( agents != null ? Mathf.Min (agents.Count, 100) : 10 );
		}

		if (GUILayout.Button ("Circle")) {
			type = RVOExampleType.Circle;
			CreateAgents ( agents != null ? agents.Count : 100 );
		}

		if (GUILayout.Button ("Point")) {
			type = RVOExampleType.Point;
			CreateAgents ( agents != null ? agents.Count : 100 );
		}
	}
	
	private float uniformDistance (float radius) {
		float v = Random.value + Random.value;
		if (v > 1) return radius * (2-v);
		else return radius * v;
	}
	
	/** Create a number of agents in circle and restart simulation */
	public void CreateAgents (int num) {
		this.agentCount = num;
		
		agents = new List<IAgent> (agentCount);
		goals = new List<Vector3>(agentCount);
		colors = new List<Color>(agentCount);
		
		sim.ClearAgents ();
		
		if (type == RVOExampleType.Circle) {
			float circleRad = Mathf.Sqrt(agentCount*radius*radius*4 / Mathf.PI) * exampleScale * 0.05f;
			
			for (int i=0;i<agentCount;i++) {
				Vector3 pos = new Vector3 (Mathf.Cos (i*Mathf.PI*2.0f/agentCount), 0, Mathf.Sin (i*Mathf.PI*2.0f/agentCount)) * circleRad;
				IAgent agent = sim.AddAgent (pos);
				agents.Add (agent);
				goals.Add (-pos);
				colors.Add (HSVToRGB (i*360.0f/agentCount,0.8f,0.6f));
			}
		} else if (type == RVOExampleType.Line) {
			
			for (int i=0;i<agentCount;i++) {
				Vector3 pos = new Vector3 ((i % 2 == 0 ?1:-1) * exampleScale, 0, (i/2) * radius * 2.5f);
				IAgent agent = sim.AddAgent (pos);
				agents.Add (agent);
				goals.Add (new Vector3 (-pos.x,pos.y,pos.z));
				colors.Add (i % 2 == 0 ? Color.red : Color.blue);
			}
		} else if (type == RVOExampleType.Point) {
			for (int i=0;i<agentCount;i++) {
				Vector3 pos = new Vector3 (Mathf.Cos (i*Mathf.PI*2.0f/agentCount), 0, Mathf.Sin (i*Mathf.PI*2.0f/agentCount)) * exampleScale;
				IAgent agent = sim.AddAgent (pos);
				agents.Add (agent);
				goals.Add (new Vector3 (0,pos.y,0));
				colors.Add (HSVToRGB (i*360.0f/agentCount,0.8f,0.6f));
			}
		} else if (type == RVOExampleType.RandomStreams) {
			float circleRad = Mathf.Sqrt(agentCount*radius*radius*4 / Mathf.PI) * exampleScale * 0.05f;
			
			for (int i=0;i<agentCount;i++) {
				float angle = Random.value*Mathf.PI*2.0f;
				float targetAngle = Random.value*Mathf.PI*2.0f;
				Vector3 pos = new Vector3 (Mathf.Cos (angle), 0, Mathf.Sin (angle)) * uniformDistance(circleRad);
				IAgent agent = sim.AddAgent (pos);
				agents.Add (agent);
				goals.Add (new Vector3 (Mathf.Cos (targetAngle), 0, Mathf.Sin (targetAngle)) * uniformDistance(circleRad));
				colors.Add (HSVToRGB (targetAngle*Mathf.Rad2Deg,0.8f,0.6f));
			}
			
		}
		
		for (int i=0;i<agents.Count;i++) {
			IAgent agent = agents[i];
			agent.Radius = radius;
			agent.MaxSpeed = maxSpeed;
			agent.AgentTimeHorizon = agentTimeHorizon;
			agent.ObstacleTimeHorizon = obstacleTimeHorizon;
			agent.MaxNeighbours = maxNeighbours;
			agent.NeighbourDist = neighbourDist;
		}
		
		verts = new Vector3[4*agents.Count];
		uv = new Vector2[verts.Length];
		tris = new int[agents.Count*2*3];
		meshColors = new Color[verts.Length];
	}
	
	public void Update () {
		if (agents == null || mesh == null) return;
		
		if (agents.Count != goals.Count) {
			Debug.LogError ("Agent count does not match goal count");
			return;
		}
		
		//Set the desired velocity for all agents
		for (int i=0;i<agents.Count;i++) {
			Vector3 pos = agents[i].InterpolatedPosition;
			Vector3 dir = goals[i] - pos;
			dir = Vector3.ClampMagnitude (dir,1);
			agents[i].DesiredVelocity = dir * agents[i].MaxSpeed;
		}

		if ( interpolatedVelocities == null || interpolatedVelocities.Length < agents.Count ) {
			var vel = new Vector3[agents.Count];
			if ( interpolatedVelocities != null ) for ( int i = 0; i < interpolatedVelocities.Length; i++ ) {
				vel[i] = interpolatedVelocities[i];
			}
			interpolatedVelocities = vel;
		}

		for (int i=0;i<agents.Count;i++) {
			IAgent agent = agents[i];
			
			interpolatedVelocities[i] = Vector3.Lerp (interpolatedVelocities[i], agent.Velocity, agent.Velocity.magnitude * Time.deltaTime*4f);

			//Create a square with the "forward" direction along the agent's velocity
			Vector3 forward = interpolatedVelocities[i].normalized * agent.Radius;
			if (forward == Vector3.zero) forward = new Vector3(0,0,agent.Radius);
			Vector3 right = Vector3.Cross (Vector3.up, forward);
			Vector3 orig = agent.InterpolatedPosition + renderingOffset;


			int vc = 4*i;
			int tc = 2*3*i;
			verts[vc+0] = (orig + forward - right);
			verts[vc+1] = (orig + forward + right);
			verts[vc+2] = (orig - forward + right);
			verts[vc+3] = (orig - forward - right);
			
			uv[vc+0] = (new Vector2 (0,1));
			uv[vc+1] = (new Vector2 (1,1));
			uv[vc+2] = (new Vector2 (1,0));
			uv[vc+3] = (new Vector2 (0,0));
			
			meshColors[vc+0] = colors[i];
			meshColors[vc+1] = colors[i];
			meshColors[vc+2] = colors[i];
			meshColors[vc+3] = colors[i];
			
			tris[tc+0] = (vc + 0);
			tris[tc+1] = (vc + 1);
			tris[tc+2] = (vc + 2);
			
			tris[tc+3] = (vc + 0);
			tris[tc+4] = (vc + 2);
			tris[tc+5] = (vc + 3);
		}
		
		//Update the mesh
		mesh.Clear ();
		mesh.vertices = verts;
		mesh.uv = uv;
		mesh.colors = meshColors;
		mesh.triangles = tris;
		mesh.RecalculateNormals ();
	}
	
	/**
	 * Converts an HSV color to an RGB color, according to the algorithm described at http://en.wikipedia.org/wiki/HSL_and_HSV
	 *
	 * \param h,s,v the color to convert.
	 * \returns the RGB representation of the color.
	 */
	static Color HSVToRGB(float h, float s, float v) {
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
