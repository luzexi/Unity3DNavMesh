using UnityEngine;
using System.Collections;
using Pathfinding.RVO;

namespace Pathfinding.RVO {
	/** Unity front end for an RVO simulator.
	 * Attached to any GameObject in a scene, scripts such as the RVOController will use the
	 * simulator exposed by this class to handle their movement.
	 * 
	 * You can have more than one of these, however most script which make use of the RVOSimulator
	 * will find it by FindObjectOfType, and thus only one will be used.
	 * 
	 * This is only a wrapper class for a Pathfinding.RVO.Simulator which simplifies exposing it
	 * for a unity scene.
	 * 
	 * \see Pathfinding.RVO.Simulator
	 * 
	 * \astarpro 
	 */
	[AddComponentMenu("Pathfinding/Local Avoidance/RVO Simulator")]
	public class RVOSimulator : MonoBehaviour {

#if UNITY_4_3 || UNITY_4_2 || UNITY_4_1 || UNITY_4_0 || UNITY_4_4
		public class Tooltip : System.Attribute { 
			public Tooltip (string s) {}
		}
#endif

		/** Calculate local avoidance in between frames.
		 * If this is enabled and multithreading is used, the local avoidance calculations will continue to run
		 * until the next frame instead of waiting for them to be done the same frame. This can increase the performance
		 * but it can make the agents seem a little less responsive.
		 * 
		 * This will only be read at Awake.
		 * \see Pathfinding.RVO.Simulator.DoubleBuffering */
		[Tooltip("Calculate local avoidance in between frames")]
		public bool doubleBuffering = true;
		
		/** Interpolate positions between simulation timesteps.
		  * If you are using agents directly, make sure you read from the InterpolatedPosition property. */
		[Tooltip("Interpolate positions between simulation timesteps")]
		public bool interpolation = true;
		
		/** Desired FPS for rvo simulation.
		  * It is usually not necessary to run a crowd simulation at a very high fps.
		  * Usually 10-30 fps is enough, but can be increased for better quality.
		  * The rvo simulation will never run at a higher fps than the game */
		[Tooltip("Desired FPS for rvo simulation. It is usually not necessary to run a crowd simulation at a very high fps.\n" +
		         "Usually 10-30 fps is enough, but can be increased for better quality.\n"+
		         "The rvo simulation will never run at a higher fps than the game")]
		public int desiredSimulatonFPS = 20;
		
		/** Number of RVO worker threads.
		 * If set to None, no multithreading will be used. */
		[Tooltip("Number of RVO worker threads. If set to None, no multithreading will be used.")]
		public ThreadCount workerThreads = ThreadCount.Two;

		/** A higher value will result in lower quality local avoidance but faster calculations.
		 * Valid range is [0...1]
		 */
		[Tooltip("A higher value will result in lower quality local avoidance but faster calculations. [0...1]")]
		public float qualityCutoff = 0.05f;

		public float stepScale = 1.5f;

		/** Higher values will raise the penalty for agent-agent intersection */
		[Tooltip("Higher values will raise the penalty for agent-agent intersection")]
		public float incompressibility = 30;

		public float desiredVelocityWeight = 0.1f;

		/** Reference to the internal simulator */
		Pathfinding.RVO.Simulator simulator;
		
		/** Get the internal simulator.
		 * Will never be null */
		public Simulator GetSimulator () {
			if (simulator == null) {
				Awake ();
			}
			return simulator;
		}
		
		void Awake () {
			if (desiredSimulatonFPS < 1) desiredSimulatonFPS = 1;
			
			if (simulator == null) {
				int threadCount = AstarPath.CalculateThreadCount (workerThreads);
				simulator = new Pathfinding.RVO.Simulator (threadCount, doubleBuffering);
				simulator.Interpolation = interpolation;
				simulator.DesiredDeltaTime = 1.0f / desiredSimulatonFPS;
			}
			
			/*Debug.LogWarning ("RVO Local Avoidance is temporarily disabled in the A* Pathfinding Project due to licensing issues.\n" +
			"I am working to get it back as soon as possible. All agents will fall back to not avoiding other agents.\n" +
			"Sorry for the inconvenience.");*/
		}
		
		/** Update the simulation */
		void Update () {
			if (desiredSimulatonFPS < 1) desiredSimulatonFPS = 1;

			Pathfinding.RVO.Sampled.Agent.DesiredVelocityWeight = desiredVelocityWeight;
			Pathfinding.RVO.Sampled.Agent.GlobalIncompressibility = incompressibility;

			GetSimulator().DesiredDeltaTime = 1.0f / desiredSimulatonFPS;
			GetSimulator().Interpolation = interpolation;
			GetSimulator().stepScale = stepScale;
			GetSimulator().qualityCutoff = qualityCutoff;
			GetSimulator().Update ();
		}
		
		void OnDestroy () {
			simulator.OnDestroy();
		}
		
	}
}