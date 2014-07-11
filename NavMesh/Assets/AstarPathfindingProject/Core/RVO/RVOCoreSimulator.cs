using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using Pathfinding;
using Pathfinding.RVO.Sampled;

#if NETFX_CORE
using Thread = Pathfinding.WindowsStore.Thread;
using ParameterizedThreadStart = Pathfinding.WindowsStore.ParameterizedThreadStart;
using ThreadStart = Pathfinding.WindowsStore.ThreadStart;
#else
using Thread = System.Threading.Thread;
using ParameterizedThreadStart = System.Threading.ParameterizedThreadStart;
using ThreadStart = System.Threading.ThreadStart;
#endif

/** Local avoidance related classes */
namespace Pathfinding.RVO {

	/** Exposes properties of an Agent class.
	  * \astarpro */
	public interface IAgent {
		/** Interpolated position of agent.
		 * Will be interpolated if the interpolation setting is enabled on the simulator.
		 */
		Vector3 InterpolatedPosition {get;}
		
		/** Position of the agent.
		 * This can be changed manually if you need to reposition the agent, but if you are reading from InterpolatedPosition, it will not update interpolated position
		 * until the next simulation step.
		 * \see Position
		 */
		Vector3 Position {get; set;}
		
		/** Desired velocity of the agent.
		 * Usually you set this once per frame. The agent will try move as close to the desired velocity as possible.
		 * Will take effect at the next simulation step.
		 */
		Vector3 DesiredVelocity {get; set;}
		/** Velocity of the agent.
		 * Can be used to set the rotation of the rendered agent.
		 * But smoothing is recommended if you do so since it might be a bit unstable when the velocity is very low.
		 * 
		 * You can set this variable manually,but it is not recommended to do so unless
		 * you have good reasons since it might degrade the quality of the simulation.
		 */
		Vector3 Velocity {get; set;}
		
		/** Locked agents will not move */
		bool Locked {get; set;}
		
		/** Radius of the agent.
		 * Agents are modelled as circles/cylinders */
		float Radius {get; set;}
		
		/** Height of the agent */
		float Height {get; set;}
		
		/** Max speed of the agent. In units per second  */
		float MaxSpeed {get; set;}
		
		/** Max distance to other agents to take them into account.
		 * Decreasing this value can lead to better performance, increasing it can lead to better quality of the simulation.
		 */
		float NeighbourDist {get; set;}
		
		/** Max number of estimated seconds to look into the future for collisions with agents.
		  * As it turns out, this variable is also very good for controling agent avoidance priorities.
		  * Agents with lower values will avoid other agents less and thus you can make 'high priority agents' by
		  * giving them a lower value.
		  */
		float AgentTimeHorizon {get; set;}
		/** Max number of estimated seconds to look into the future for collisions with obstacles */
		float ObstacleTimeHorizon {get; set;}

		/** Specifies the avoidance layer for this agent.
		 * The #CollidesWith mask on other agents will determine if they will avoid this agent.
		 */
		RVOLayer Layer {get; set;}

		/** Layer mask specifying which layers this agent will avoid.
		 * You can set it as CollidesWith = RVOLayer.DefaultAgent | RVOLayer.Layer3 | RVOLayer.Layer6 ...
		 * 
		 * \see http://en.wikipedia.org/wiki/Mask_(computing)
		 */
		RVOLayer CollidesWith {get; set;}

		/** Debug drawing */
		bool DebugDraw {get; set;}
		
		/** Max number of agents to take into account.
		 * Decreasing this value can lead to better performance, increasing it can lead to better quality of the simulation.
		 */
		int MaxNeighbours {get; set;}
		
		/** List of obstacle segments which were close to the agent during the last simulation step.
		 * Can be used to apply additional wall avoidance forces for example.
		 * Segments are formed by the obstacle vertex and its .next property.
		 */
		List<ObstacleVertex> NeighbourObstacles {get; }
		
		/** Teleports the agent to a new position.
		 * Just setting the position can cause strange effects when using interpolation.
		 */
		void Teleport (Vector3 pos);


	}

	[System.Flags]
	public enum RVOLayer {
		DefaultAgent = 1 << 0,
		DefaultObstacle = 1 << 1,
		Layer2 = 1 << 2,
		Layer3 = 1 << 3,
		Layer4 = 1 << 4,
		Layer5 = 1 << 5,
		Layer6 = 1 << 6,
		Layer7 = 1 << 7,
		Layer8 = 1 << 8,
		Layer9 = 1 << 9,
		Layer10 = 1 << 10,
		Layer11 = 1 << 11,
		Layer12 = 1 << 12,
		Layer13 = 1 << 13,
		Layer14 = 1 << 14,
		Layer15 = 1 << 15,
		Layer16 = 1 << 16,
		Layer17 = 1 << 17,
		Layer18 = 1 << 18,
		Layer19 = 1 << 19,
		Layer20 = 1 << 20,
		Layer21 = 1 << 21,
		Layer22 = 1 << 22,
		Layer23 = 1 << 23,
		Layer24 = 1 << 24,
		Layer25 = 1 << 25,
		Layer26 = 1 << 26,
		Layer27 = 1 << 27,
		Layer28 = 1 << 28,
		Layer29 = 1 << 29,
		Layer30 = 1 << 30
	}

	/** Local Avoidance %Simulator.
	 * This class handles local avoidance simulation for a number of agents using
	 * Reciprocal Velocity Obstacles (RVO) and Optimal Reciprocal Collision Avoidance (ORCA).
	 * 
	 * This class will handle calculation of velocities from desired velocities supplied by a script.
	 * It is, however, not responsible for moving any objects in a Unity Scene. For that there are other scripts (see below).
	 * 
	 * Obstacles can be added and removed from the simulation, agents can also be added and removed at any time.
	 * \see
	 * RVOSimulator
	 * RVOAgent
	 * Pathfinding.RVO.IAgent
	 * 
	 * The implementation is based on the RVO2 Library (http://gamma.cs.unc.edu/RVO2/) extended with many new features.
	 * 
	 * You will most likely mostly use the wrapper class RVOSimulator.
	 *
	 * \astarpro
	 */
	public class Simulator {
		
		/** Use Double Buffering.
		 * \see DoubleBuffering */
		private bool doubleBuffering = true;
		
		/** Inverse desired simulation fps.
		 * \see DesiredDeltaTime
		 */
		private float desiredDeltaTime = 0.05f;
		
		/** Use Interpolation.
		 * \see Interpolation */
		private bool interpolation = true;
		
		/** Worker threads */
		Worker[] workers;
		
		/** Agents in this simulation */
		List<Agent> agents;
		
		/** Obstacles in this simulation */
		List<ObstacleVertex> obstacles;



		RVOQuadtree quadtree = new RVOQuadtree();

		public float qualityCutoff = 0.05f;
		public float stepScale = 1.5f;


		/** Quadtree for this simulation.
		 * Used internally by the simulation to perform fast neighbour lookups for each agent.
		 * Please only read from this tree, do not rebuild it since that can interfere with the simulation.
		 * It is rebuilt when needed.
		 */
		public RVOQuadtree Quadtree { get { return quadtree; } }

		private float frameDeltaTime;
		private float deltaTime;
		private float prevDeltaTime = 0;
		
		private float lastStep = -99999;
		private float lastFrame = 0;
		
		private bool doUpdateObstacles = false;
		private bool doCleanObstacles = false;
		
		private int frameTimeBufferIndex = 0;
		private float[] frameTimeBuffer = new float[5];
		
		public float FrameDeltaTime { get { return frameDeltaTime; }}
		public float DeltaTime { get { return deltaTime; } }
		public float PrevDeltaTime { get { return prevDeltaTime; } }
		
		/** Is using multithreading */
		public bool Multithreading { get { return workers != null && workers.Length > 0; }}
		
		/** Time in seconds between each simulation step.
		 * This is the desired delta time, the simulation will never run at a higher fps than
		 * the rate at which the Update function is called.
		 */
		public float DesiredDeltaTime { get { return desiredDeltaTime; } set { desiredDeltaTime = System.Math.Max (value,0.0f); }}
		
		/** Use Interpolation.
		 * If interpolation is enabled, agent positions will be interpolated on frames when no rvo calculation is done.
		 * This has a very small overhead, but usually yields much smoother looking movement.
		 */
		public bool Interpolation { get { return interpolation; } set { interpolation = value; }}



		//internal int textureWidth;
		//internal Texture2D tex;
		//internal float textureSize;
		//internal float colorScale = 0.05f;
		//Color[] colors;
		
		//bool dirtyColors = false;

		/** Get a list of all agents.
		 * 
		 * This is an internal list.
		 * I'm not going to be restrictive so you may access it since it is better for performance
		 * but please do not modify it since that can cause errors in the simulation.
		 * 
		 * \warning Do not modify this list! */
		public List<Agent> GetAgents () {
			return agents;
		}
		
		/** Get a list of all obstacles.
		 * This is a list of obstacle vertices.
		 * Each vertex is part of a doubly linked list loop
		 * forming an obstacle polygon.
		 * 
		 * \warning Do not modify this list!
		 * 
		 * \see AddObstacle
		 * \see RemoveObstacle
		 */
		public List<ObstacleVertex> GetObstacles () {
			return obstacles;
		}
		
		/** Create a new simulator.
		 * 
		 * \param workers Use the specified number of worker threads.\n
		 * When the number zero is specified, no multithreading will be used.
		 * A good number is the number of cores on the machine.
		 * 
		 * \param doubleBuffering Use Double Buffering for calculations.
		 * Testing done with 5000 agents and 0.1 desired delta time showed that with double buffering enabled
		 * the game ran at 50 fps for most frames, dropping to 10 fps during calculation frames. But without double buffering
		 * it ran at around 10 fps all the time.\n
		 * This will let threads calculate while the game progresses instead of waiting for the calculations
		 * to finish.
		 * \note Will only have effect if using multithreading
		 * 
		 * \see #Multithreading
		 */
		public Simulator (int workers, bool doubleBuffering) {
			this.workers = new Simulator.Worker[workers];
			this.doubleBuffering = doubleBuffering;
			
			for (int i=0;i<workers;i++) this.workers[i] = new Simulator.Worker(this);
			
			//kdTree = new KDTree(this);
			agents = new List<Agent>();
			obstacles = new List<ObstacleVertex>();
			
		}

		/*internal void DebugPlot ( Vector2 p, Color col ) {
			if ( colors == null ) {
				tex = new Texture2D(textureWidth,textureWidth);
				//mat.mainTexture = tex;
				colors = new Color[tex.width*tex.height];
			}

			int x = Mathf.RoundToInt (p.x*tex.width/textureSize);
			int y = Mathf.RoundToInt (p.y*tex.height/textureSize);
			
			if ( x >= 0 && y >= 0 && x < tex.width && y < tex.height ) {
				dirtyColors = true;
				colors[x+y*tex.width] = col;
			}
		}*/

		/** Removes all agents from the simulation */
		public void ClearAgents () {
			
			//Bad to update agents while processing of current agents might be done
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			for (int i=0;i<agents.Count;i++) {
				agents[i].simulator = null;
			}
			agents.Clear ();

		}
		
		public void OnDestroy () {
			if (workers != null) {
				for (int i=0;i<workers.Length;i++) workers[i].Terminate ();
			}
		}
		
		/** Terminates any worker threads */
		~Simulator () {
			OnDestroy ();
		}
		
		/** Add a previously removed agent to the simulation.
		  * An agent can only be in one simulation at a time, any attempt to add an agent to two simulations
		  * or multiple times to the same simulation will result in an exception being thrown.
		  * 
		  * \see RemoveAgent
		  */
		public IAgent AddAgent (IAgent agent) {
			if (agent == null) throw new System.ArgumentNullException ("Agent must not be null");
			
			Agent agentReal = agent as Agent;
			if (agentReal == null) throw new System.ArgumentException ("The agent must be of type Agent. Agent was of type "+agent.GetType ());
			
			
			if (agentReal.simulator != null && agentReal.simulator == this) throw new System.ArgumentException ("The agent is already in the simulation");
			else if (agentReal.simulator != null) throw new System.ArgumentException ("The agent is already added to another simulation");
			agentReal.simulator = this;
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			agents.Add (agentReal);

			
			return agent;
		}
		
		/** Add an agent at the specified position.
		 * You can use the returned interface to read several parameters such as position and velocity
		 * and set for example radius and desired velocity.
		 * 
		 * \see RemoveAgent
		 */
		public IAgent AddAgent (Vector3 position) {
			
			Agent agent = new Agent (position);
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			agents.Add (agent);
			agent.simulator = this;
			
			return agent;
		}
		
		/** Removes a specified agent from this simulation.
		 * The agent can be added again later by using AddAgent.
		 * 
		 * \see AddAgent(IAgent)
		 * \see ClearAgents
		 */
		public void RemoveAgent (IAgent agent) {
			if (agent == null) throw new System.ArgumentNullException ("Agent must not be null");
			
			Agent agentReal = agent as Agent;
			if (agentReal == null) throw new System.ArgumentException ("The agent must be of type Agent. Agent was of type "+agent.GetType ());
			
			if (agentReal.simulator != this) throw new System.ArgumentException ("The agent is not added to this simulation");
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			agentReal.simulator = null;
			
			if (!agents.Remove (agentReal)) {
				throw new System.ArgumentException ("Critical Bug! This should not happen. Please report this.");
			}
		}
		
		/** Adds a previously removed obstacle.
		 * This does not check if the obstacle is already added to the simulation, so please do not add an obstacle multiple times.
		 * 
		 * It is assumed that this is a valid obstacle.
		 */
		public ObstacleVertex AddObstacle (ObstacleVertex v) {
			if (v == null) throw new System.ArgumentNullException ("Obstacle must not be null");
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			obstacles.Add (v);
			UpdateObstacles ();
			return v;
		}
		
		/** Adds an obstacle described by the vertices.
		 * 
		 * \see RemoveObstacle
		 */
		public ObstacleVertex AddObstacle (Vector3[] vertices, float height) {
			
			return AddObstacle (vertices, height, Matrix4x4.identity);
			
			/*if (vertices == null) throw new System.ArgumentNullException ("Vertices must not be null");
			
			if (vertices.Length < 2) throw new System.ArgumentException ("Less than 2 vertices in an obstacle");
			
			ObstacleVertex first = null;
			ObstacleVertex prev = null;
			
			for (int i=0;i<vertices.Length;i++) {
				ObstacleVertex v = new ObstacleVertex();
				if (first == null) first = v;
				else prev.next = v;
				
				v.prev = prev;
				v.position = vertices[i];
				//v.thin = thin;
				v.height = height;
				prev = v;
			}
			
			prev.next = first;
			first.prev = prev;
			
			ObstacleVertex c = first;
			do {
				Vector3 dir = c.next.position - c.position;
				v.dir =  new Vector2 (dir.x,dir.z).normalized;
				
				if (vertices.Length	== 2) {
					v.convex = true;
				} else {
					v.convex = Polygon.IsClockwiseMargin (c.prev.position,c.position, c.next.position);
				}
				
				c = c.next;
			} while (c != first);
			
			obstacles.Add (first);
			
			UpdateObstacles ();
			return first;*/
		}
		
		/** Adds an obstacle described by the vertices.
		 * 
		 * \see RemoveObstacle
		 */
		public ObstacleVertex AddObstacle (Vector3[] vertices, float height, Matrix4x4 matrix) {
			
			if (vertices == null) throw new System.ArgumentNullException ("Vertices must not be null");
			
			if (vertices.Length < 2) throw new System.ArgumentException ("Less than 2 vertices in an obstacle");
			
			ObstacleVertex first = null;
			ObstacleVertex prev = null;
			
			bool identity = matrix == Matrix4x4.identity;
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			for (int i=0;i<vertices.Length;i++) {
				ObstacleVertex v = new ObstacleVertex();
				if (first == null) first = v;
				else prev.next = v;
				
				v.prev = prev;
				
				//Premature optimization ftw!
				v.position = identity ? vertices[i] : matrix.MultiplyPoint3x4(vertices[i]);
				
				//v.thin = thin;
				v.height = height;
				
				prev = v;
			}
			
			prev.next = first;
			first.prev = prev;
			
			ObstacleVertex c = first;
			do {
				Vector3 dir = c.next.position - c.position;
				c.dir = new Vector2 (dir.x,dir.z).normalized;
				
				if (vertices.Length	== 2) {
					c.convex = true;
				} else {
					c.convex = Polygon.IsClockwiseMargin (c.next.position,c.position, c.prev.position);
				}
				
				c = c.next;
			} while (c != first);
			
			obstacles.Add (first);
			
			UpdateObstacles ();
			return first;
		}
		
		/**
		 * Adds a line obstacle with a specified height.
		 * 
		 * \see RemoveObstacle
		 */
		public ObstacleVertex AddObstacle (Vector3 a, Vector3 b, float height) {
			ObstacleVertex first = new ObstacleVertex ();
			ObstacleVertex second = new ObstacleVertex ();
			
			first.prev = second;
			second.prev = first;
			first.next = second;
			second.next = first;
			
			first.position = a;
			second.position = b;
			first.height = height;
			second.height = height;
			
			first.convex = true;
			second.convex = true;
			
			first.dir = new Vector2 (b.x-a.x,b.z-a.z).normalized;
			second.dir = -first.dir;
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			obstacles.Add (first);
			
			UpdateObstacles ();
			return first;
		}
		
		/** Updates the vertices of an obstacle.
		 * \param obstacle %Obstacle to update
		 * \param vertices New vertices for the obstacle, must have at least the number of vertices in the original obstacle
		 * \param matrix %Matrix to multiply vertices with before updating obstacle
		 * 
		 * The number of vertices in an obstacle cannot be changed, existing vertices can only be moved.
		 */
		public void UpdateObstacle (ObstacleVertex obstacle, Vector3[] vertices, Matrix4x4 matrix) {
			
			if (vertices == null) throw new System.ArgumentNullException ("Vertices must not be null");
			if (obstacle == null) throw new System.ArgumentNullException ("Obstacle must not be null");
			
			if (vertices.Length < 2) throw new System.ArgumentException ("Less than 2 vertices in an obstacle");
			
			if (obstacle.split) throw new System.ArgumentException ("Obstacle is not a start vertex. You should only pass those ObstacleVertices got from AddObstacle method calls");
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			//Compact obstacle and count
			int count = 0;
			
			ObstacleVertex c = obstacle;
			do {
				while (c.next.split) {
					c.next = c.next.next;
					c.next.prev = c;
				}
				
				if (count >= vertices.Length) {
					Debug.DrawLine (c.prev.position, c.position,Color.red);
					throw new System.ArgumentException ("Obstacle has more vertices than supplied for updating (" + vertices.Length+ " supplied)");
				}
				c.position = matrix.MultiplyPoint3x4 (vertices[count]);
				count++;
				c = c.next;
			} while (c != obstacle);
			
			c = obstacle;
			do {
				Vector3 dir = c.next.position - c.position;
				c.dir =  new Vector2 (dir.x,dir.z).normalized;
				
				if (vertices.Length	== 2) {
					c.convex = true;
				} else {
					c.convex = Polygon.IsClockwiseMargin (c.next.position,c.position, c.prev.position);
				}
				
				c = c.next;
			} while (c != obstacle);
			
			ScheduleCleanObstacles ();
			UpdateObstacles();
		}
		
		private void ScheduleCleanObstacles () {
			doCleanObstacles = true;
		}
		
		private void CleanObstacles () {
			
			for (int i=0;i<obstacles.Count;i++) {
				ObstacleVertex first = obstacles[i];
				ObstacleVertex c = first;
				do {
					while (c.next.split) {
						c.next = c.next.next;
						c.next.prev = c;
					}
					c = c.next;
				} while (c != first);
			}
		}
		
		/** Removes the obstacle identified by the vertex.
		  * This must be the same vertex as the one returned by the AddObstacle call.
		  * 
		  * \see AddObstacle
		  */
		public void RemoveObstacle (ObstacleVertex v) {
			if (v == null) throw new System.ArgumentNullException ("Vertex must not be null");
			
			//Don't interfere with ongoing calculations
			if (Multithreading && doubleBuffering) for (int j=0;j<workers.Length;j++) workers[j].WaitOne();
			
			obstacles.Remove (v);
			UpdateObstacles ();
		}
		
		/** Rebuilds the obstacle tree at next simulation frame.
		 * Add and remove obstacle functions call this automatically.
		 */
		public void UpdateObstacles () {
			//Update obstacles at next frame 
			doUpdateObstacles = true;
		}

		void BuildQuadtree () {
			quadtree.Clear ();
			if ( agents.Count > 0 ) {
				Rect bounds = Rect.MinMaxRect (agents[0].position.x, agents[0].position.y, agents[0].position.x, agents[0].position.y);
				for ( int i = 1; i < agents.Count; i++ ) {
					Vector3 p = agents[i].position;
					bounds = Rect.MinMaxRect ( Mathf.Min(bounds.xMin, p.x), Mathf.Min(bounds.yMin, p.z), Mathf.Max(bounds.xMax, p.x), Mathf.Max(bounds.yMax, p.z) );
				}
				quadtree.SetBounds (bounds);

				for (int i=0;i<agents.Count;i++) {
					quadtree.Insert (agents[i]);
				}
				
				//quadtree.DebugDraw ();
			}
		}

		private WorkerContext coroutineWorkerContext = new WorkerContext();

		/** Should be called once per frame */
		public void Update () {
			
			//Initialize last step
			if (lastStep < 0) {
				lastStep = Time.time;
				deltaTime = DesiredDeltaTime;
			}
			
			if (Time.time - lastStep > DesiredDeltaTime) {
				
				prevDeltaTime = DeltaTime;
				deltaTime = Time.time - lastStep;
				lastStep = Time.time;
				
				frameTimeBufferIndex++;
				frameTimeBufferIndex %= frameTimeBuffer.Length;
				frameTimeBuffer[frameTimeBufferIndex] = deltaTime;
				
				float sum = 0;
				for (int i=0;i<frameTimeBuffer.Length;i++) {
					sum += frameTimeBuffer[i];
				}
				sum /= frameTimeBuffer.Length;
				
				deltaTime = sum;
				//deltaTime = DesiredDeltaTime;

				//Debug.Log (frameTimeBufferIndex + " " +(frameTimeBufferIndex-1+frameTimeBuffer.Length)%frameTimeBuffer.Length);
				
				//Calculate smooth delta time
				//deltaTime = (Time.time - frameTimeBuffer[(frameTimeBufferIndex-1+frameTimeBuffer.Length)%frameTimeBuffer.Length]) / frameTimeBuffer.Length;
				
				//Prevent a zero delta time
				deltaTime = System.Math.Max (deltaTime, 1.0f/2000f);
				
				frameDeltaTime = DeltaTime;
				
				if (Multithreading) {
					
					if (doubleBuffering) {
						for (int i=0;i<workers.Length;i++) workers[i].WaitOne();
						if (!Interpolation) for (int i=0;i<agents.Count;i++) agents[i].Interpolate (1.0f);
					}
					
					
					if (doCleanObstacles) {
						CleanObstacles();
						doCleanObstacles = false;
						doUpdateObstacles = true;
					}
					
					if (doUpdateObstacles) {
						doUpdateObstacles = false;
					}
					
					
					//kdTree.BuildAgentTree ();
					BuildQuadtree ();

					for (int i=0;i<workers.Length;i++) {
						workers[i].start = i*agents.Count / workers.Length;
						workers[i].end = (i+1)*agents.Count / workers.Length;
					}

					//Update
					//BufferSwitch
					for (int i=0;i<workers.Length;i++) workers[i].Execute (1);
					for (int i=0;i<workers.Length;i++) workers[i].WaitOne();

					//Calculate New Velocity
					for (int i=0;i<workers.Length;i++) workers[i].Execute (0);
					if (!doubleBuffering) {
						for (int i=0;i<workers.Length;i++) workers[i].WaitOne();
						if (!Interpolation) for (int i=0;i<agents.Count;i++) agents[i].Interpolate (1.0f);
					}
				} else {
					
					if (doCleanObstacles) {
						CleanObstacles();
						doCleanObstacles = false;
						doUpdateObstacles = true;
					}
					
					if (doUpdateObstacles) {
						doUpdateObstacles = false;
					}

					//kdTree.BuildAgentTree ();
					BuildQuadtree ();

					for (int i=0;i<agents.Count;i++) {
						agents[i].Update ();
						agents[i].BufferSwitch ();
					}


					for (int i=0;i<agents.Count;i++) {
						agents[i].CalculateNeighbours ();
						agents[i].CalculateVelocity ( coroutineWorkerContext );
					}

					// Temporarily moved here
					/*for (int i=0;i<agents.Count;i++) {
						agents[i].Update ();
						agents[i].BufferSwitch ();
					}*/

					if (!Interpolation) for (int i=0;i<agents.Count;i++) agents[i].Interpolate (1.0f);
				}
			}
			
			frameDeltaTime = (Time.time - lastFrame);
			lastFrame = Time.time;
			
			if (Interpolation) {
				for (int i=0;i<agents.Count;i++) {
					agents[i].Interpolate ((Time.time  + frameDeltaTime - lastStep)/DeltaTime);
				}
			}
		}

		internal class WorkerContext {
			public Agent.VO[] vos = new Agent.VO[20];
		}

		private class Worker {
			public Thread thread;
			public int start, end;
			public int task = 0;
			
			public AutoResetEvent runFlag = new AutoResetEvent(false);
			
			public ManualResetEvent waitFlag = new ManualResetEvent(true);
			
			public Simulator simulator;
			
			private bool terminate = false;

			private WorkerContext context = new WorkerContext();

			public Worker (Simulator sim) {
				this.simulator = sim;
				thread = new Thread (new ThreadStart (Run));
				thread.IsBackground = true;
				thread.Name = "RVO Simulator Thread";
				thread.Start ();
			}
			
			public void Execute (int task) {
				this.task = task;
				waitFlag.Reset ();
				runFlag.Set ();
			}
			
			public void WaitOne () {
				waitFlag.WaitOne ();
			}
			
			public void Terminate () {
				terminate = true;
			}
			
			public void Run () {
				
				runFlag.WaitOne ();
				
				while (!terminate) {
					try {
						List<Agent> agents = simulator.GetAgents ();
						if (task == 0) {
							for (int i=start;i<end;i++) {
								agents[i].CalculateNeighbours ();
								agents[i].CalculateVelocity ( context );
							}
							
						} else if (task == 1) {
							for (int i=start;i<end;i++) {
								agents[i].Update ();
								agents[i].BufferSwitch ();
							}
						} else if ( task  == 2 ) {
							simulator.BuildQuadtree ();
						/*} else if (task == 2) {
							for (int i=start;i<end;i++) {
								agents[i].BufferSwitch ();
							}*/
						} else {
							Debug.LogError ("Invalid Task Number: " + task);
							throw new System.Exception ("Invalid Task Number: " + task);
						}
					} catch (System.Exception e) {
						Debug.LogError (e);
					}
					waitFlag.Set ();
					runFlag.WaitOne ();
				}
			}
		}
	}
}