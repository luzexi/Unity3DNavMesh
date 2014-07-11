using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.RVO;

namespace Pathfinding {
	[RequireComponent(typeof(Seeker))]
	[AddComponentMenu("Pathfinding/AI/RichAI (for navmesh)")]
	/** Advanced AI for navmesh based graphs.
	 * \astarpro
	 */
	public class RichAI : MonoBehaviour {
		
		public Transform target;
		
		/** Draw gizmos in the scene view */
		public bool drawGizmos = true;
		
		/** Search for new paths repeatedly */
		public bool repeatedlySearchPaths = false;
		
		/** Delay (seconds) between path searches */
		public float repathRate = 0.5f;
		
		/** Max speed of the agent.
		 * World units per second */
		public float maxSpeed = 1;
		/** Max acceleration of the agent.
		 * World units per second per second */
		public float acceleration = 5;
		/** How much time to use for slowdown in the end of the path.
		 * A lower value give more abrupt stops
		 */
		public float slowdownTime = 0.5f;
		/** Max rotation speed of the agent.
		 * In degrees per second.
		 */
		public float rotationSpeed = 360;
		/** Max distance to the endpoint to consider it reached */
		public float endReachedDistance = 0.01f;
		/** Force to avoid walls with.
		  * The agent will try to steer away from walls slightly. */
		public float wallForce = 3;
		/** Walls within this range will be used for avoidance.
		 * Setting this to zero disables wall avoidance and may improve performance slightly */
		public float wallDist = 1;
		
		/** Gravity to use in case no character controller is attached */
		public Vector3 gravity = new Vector3(0,-9.82f,0);
		
		/** Raycast for ground placement (when not having a CharacterController).
		 * A raycast from position + up*#centerOffset downwards will be done and the agent will be placed at this point.
		 */
		public bool raycastingForGroundPlacement = true;
		
		/** Layer mask to use for ground placement.
		 * Make sure this does not include the layer of any eventual colliders attached to this gameobject.
		 */
		public LayerMask groundMask = -1;
		public float centerOffset = 1;
		
		/** Mode for funnel simplification.
		 * On tiled navmesh maps, but sometimes on normal ones as well, it can be good to simplify
		 * the funnel as a post-processing step.
		 */
		public RichFunnel.FunnelSimplification funnelSimplification = RichFunnel.FunnelSimplification.None;
		public Animation anim;
		
		/** Use a 3rd degree equation for calculating slowdown acceleration instead of a 2nd degree.
		 * A 3rd degree equation can also make sure that the velocity when reaching the target is roughly zero and therefore
		 * it will have a more direct stop. In contrast solving a 2nd degree equation which will just make sure the target is reached but
		 * will usually have a larger velocity when reaching the target and therefore look more "bouncy".
		 */
		public bool preciseSlowdown = true;
		
		/** Slow down when not facing the target direction.
		 * Incurs at a small overhead.
		 */
		public bool slowWhenNotFacingTarget = true;
		
		/** Current velocity of the agent.
		 * Includes eventual velocity due to gravity */
		Vector3 velocity;
		
		/** Current velocity of the agent.
		 * Includes eventual velocity due to gravity */
		public Vector3 Velocity {
			get {
				return velocity;
			}
		}
		
		protected RichPath rp;
		
		protected Seeker seeker;
		protected Transform tr;
		CharacterController controller;
		RVOController rvoController;
		
		Vector3 lastTargetPoint;
		Vector3 currentTargetDirection;
		
		protected bool waitingForPathCalc = false;
		protected bool canSearchPath = false;
		protected bool delayUpdatePath = false;
		protected bool traversingSpecialPath = false;
		protected bool lastCorner = false;
		float distanceToWaypoint = 999;
		
		protected List<Vector3> buffer = new List<Vector3>();
		protected List<Vector3> wallBuffer = new List<Vector3>();
		
		bool startHasRun = false;
		protected float lastRepath = -9999;
		
		void Awake () {
			seeker = GetComponent<Seeker>();
			controller = GetComponent<CharacterController>();
			rvoController = GetComponent<RVOController>();
			if ( rvoController != null ) rvoController.enableRotation = false;
			tr = transform;
		}
			
		/** Starts searching for paths.
		 * If you override this function you should in most cases call base.Start () at the start of it.
		 * \see OnEnable
		 * \see SearchPaths
		 */
		protected virtual void Start () {
			startHasRun = true;
			OnEnable ();
		}
		
		/** Run at start and when reenabled.
		 * Starts RepeatTrySearchPath.
		 * 
		 * \see Start
		 */
		protected virtual void OnEnable () {
			
			lastRepath = -9999;
			waitingForPathCalc = false;
			canSearchPath = true;
			
			if (startHasRun) {
				//Make sure we receive callbacks when paths complete
				seeker.pathCallback += OnPathComplete;
				
				StartCoroutine (SearchPaths ());
			}
		}
		
		public void OnDisable () {
			// Abort calculation of path
			if (seeker != null && !seeker.IsDone()) seeker.GetCurrentPath().Error();
			
			//Make sure we receive callbacks when paths complete
			seeker.pathCallback -= OnPathComplete;
		}
		
		/** Force recalculation of the current path.
		 * If there is an ongoing path calculation, it will be canceled (so make sure you leave time for the paths to get calculated before calling this function again).
		 */
		public virtual void UpdatePath () {
			canSearchPath = true;
			waitingForPathCalc = false;
			Path p = seeker.GetCurrentPath();
			
			//Cancel any eventual pending pathfinding request
			if (p != null && !seeker.IsDone()) {
				p.Error();
				// Make sure it is recycled. We won't receive a callback for this one since we
				// replace the path directly after this
				p.Claim (this);
				p.Release (this);
			}
			
			waitingForPathCalc = true;
			lastRepath = Time.time;
			seeker.StartPath (tr.position, target.position);
		}
		
		IEnumerator SearchPaths () {
			while (true) {
				while (!repeatedlySearchPaths || waitingForPathCalc || !canSearchPath || Time.time - lastRepath < repathRate) yield return null;
				//canSearchPath = false;
				
				//waitingForPathCalc = true;
				//lastRepath = Time.time;
				//seeker.StartPath (tr.position, target.position);
				UpdatePath ();
	
				yield return null;
			}
		}
		
		void OnPathComplete (Path p) {
			waitingForPathCalc = false;
			p.Claim(this);
			
			if (p.error) {
				p.Release(this);
				return;
			}
			
			if (traversingSpecialPath) {
				delayUpdatePath = true;
			} else {
				if (rp == null) rp = new RichPath();
				rp.Initialize (seeker, p,true, funnelSimplification);
			}
			p.Release(this);
		}
		
		public bool TraversingSpecial {
			get {
				return traversingSpecialPath;
			}
		}
		
		/** Current target point.
		 */
		public Vector3 TargetPoint {
			get {
				return lastTargetPoint;
			}
		}
		
		/** True if approaching the last waypoint in the current path */
		public bool ApproachingPartEndpoint {
			get {
				return lastCorner;
			}
		}
		
		/** True if approaching the last waypoint of all parts in the current path */
		public bool ApproachingPathEndpoint {
			get {
				return rp == null ? false : ApproachingPartEndpoint && !rp.PartsLeft();
			}
		}
		
		/** Distance to the next waypoint */
		public float DistanceToNextWaypoint {
			get {
				return distanceToWaypoint;
			}
		}
		
		/** Declare that the AI has completely traversed the current part.
		 * This will skip to the next part, or call OnTargetReached if this was the last part
		 */
		void NextPart () {
			rp.NextPart();
			lastCorner = false;
			if (!rp.PartsLeft()) {
				//End
				OnTargetReached();
			}
		}
		
		/** Smooth delta time to avoid getting overly effected by e.g GC */
		static float deltaTime = 0;
		
		/** Called when the end of the path is reached */
		protected virtual void OnTargetReached () {
		}
	
		protected virtual Vector3 UpdateTarget ( RichFunnel fn ) {
			buffer.Clear ();
			/* Current position. We read and write to tr.position as few times as possible since doing so
					 * is much slower than to read and write from/to a local variable
					 */
			Vector3 position = tr.position;
			bool requiresRepath;
			position = fn.Update (position, buffer, 2, out lastCorner, out requiresRepath);
	
			if (requiresRepath && !waitingForPathCalc) {
				UpdatePath ();
			}
	
			return position;
		}
	
		/** Update is called once per frame */
		protected virtual void Update () {
			deltaTime = Mathf.Min (Time.smoothDeltaTime*2, Time.deltaTime);
			
			if (rp != null) {
				//System.Diagnostics.Stopwatch w = new System.Diagnostics.Stopwatch();
				//w.Start();
				RichPathPart pt = rp.GetCurrentPart();
				RichFunnel fn = pt as RichFunnel;
				if (fn != null) {
					
					//Clear buffers for reuse
					Vector3 position = UpdateTarget ( fn );
					
					//tr.position = ps;
					
					//Only get walls every 5th frame to save on performance
					if (Time.frameCount % 5 == 0) {
						wallBuffer.Clear();
						fn.FindWalls (wallBuffer, wallDist);
					}
					
					/*for (int i=0;i<wallBuffer.Count;i+=2) {
						Debug.DrawLine (wallBuffer[i],wallBuffer[i+1],Color.magenta);
					}*/
					
					//Pick next waypoint if current is reached
					int tgIndex = 0;
					/*if (buffer.Count > 1) {
						if ((buffer[tgIndex]-tr.position).sqrMagnitude < pickNextWaypointDist*pickNextWaypointDist) {
							tgIndex++;
						}
					}*/
					
			
					//Target point
					Vector3 tg = buffer[tgIndex];
					Vector3 dir = tg-position;
					dir.y = 0;
					
					bool passedTarget = Vector3.Dot (dir,currentTargetDirection) < 0;
					//Check if passed target in another way
					if (passedTarget && buffer.Count-tgIndex > 1) {
						tgIndex++;
						tg = buffer[tgIndex];
					}
					
					if (tg != lastTargetPoint) {
						currentTargetDirection = (tg - position);
						currentTargetDirection.y = 0;
						currentTargetDirection.Normalize();
						lastTargetPoint = tg;
						//Debug.DrawRay (tr.position, Vector3.down*2,Color.blue,0.2f);
					}
					
					//Direction to target
					dir = (tg-position);
					dir.y = 0;
					float magn = dir.magnitude;
					
					//Write out for other scripts to read
					distanceToWaypoint = magn;
					
					//Normalize
					dir = magn == 0 ? Vector3.zero : dir/magn;
					Vector3 normdir = dir;
					
					Vector3 force = Vector3.zero;
					
					if (wallForce > 0 && wallDist > 0) {
						float wLeft = 0;
						float wRight = 0;
						
						for (int i=0;i<wallBuffer.Count;i+=2) {
							
							Vector3 closest = AstarMath.NearestPointStrict (wallBuffer[i],wallBuffer[i+1],tr.position);
							float dist = (closest-position).sqrMagnitude;
							
							if (dist > wallDist*wallDist) continue;
							
							Vector3 tang = (wallBuffer[i+1]-wallBuffer[i]).normalized;
							
							//Using the fact that all walls are laid out clockwise (seeing from inside)
							//Then left and right (ish) can be figured out like this
							float dot = Vector3.Dot (dir, tang) * (1 - System.Math.Max (0,(2*(dist / (wallDist*wallDist))-1)));
							if (dot > 0) wRight = System.Math.Max (wRight, dot);
							else wLeft = System.Math.Max (wLeft, -dot);
						}
						
						Vector3 norm = Vector3.Cross (Vector3.up, dir);
						force = norm*(wRight-wLeft);
						
						//Debug.DrawRay (tr.position, force, Color.cyan);
					}
					
					//Is the endpoint of the path (part) the current target point
					bool endPointIsTarget = lastCorner && buffer.Count-tgIndex == 1;
					
					if (endPointIsTarget) {
						//Use 2nd or 3rd degree motion equation to figure out acceleration to reach target in "exact" [slowdownTime] seconds
						
						//Clamp to avoid divide by zero
						if (slowdownTime < 0.001f) {
							slowdownTime = 0.001f;
						}
						
						Vector3 diff = tg - position;
						diff.y = 0;
						
						if (preciseSlowdown) {
							//{ t = slowdownTime
							//{ diff = vt + at^2/2 + qt^3/6
							//{ 0 = at + qt^2/2
							//{ solve for a
							dir = (6*diff - 4*slowdownTime*velocity)/(slowdownTime*slowdownTime);
						} else {
							dir = 2*(  diff -   slowdownTime*velocity)/(slowdownTime*slowdownTime);
						}
						dir = Vector3.ClampMagnitude (dir, acceleration);
						
						force *= System.Math.Min (magn/0.5f,1);
						
						if (magn < endReachedDistance) {
							//END REACHED
							NextPart ();
						}
					} else {
						dir *= acceleration;
					}
					
					//Debug.DrawRay (tr.position+Vector3.up, dir*3, Color.blue);
					
					velocity += (dir + force*wallForce)*deltaTime;
					
					if (slowWhenNotFacingTarget) {
						float dot = (Vector3.Dot (normdir, tr.forward)+0.5f)*(1.0f/1.5f);
						//velocity = Vector3.ClampMagnitude (velocity, maxSpeed * Mathf.Max (dot, 0.2f) );
						float xzmagn = Mathf.Sqrt (velocity.x*velocity.x + velocity.z*velocity.z);
						float prevy = velocity.y;
						velocity.y = 0;
						float mg = Mathf.Min (xzmagn, maxSpeed * Mathf.Max (dot, 0.2f) );
						velocity = Vector3.Lerp ( tr.forward * mg, velocity.normalized * mg, Mathf.Clamp ( endPointIsTarget ? (magn*2) : 0, 0.5f, 1.0f) );
	
						velocity.y = prevy;
					} else {
						// Clamp magnitude on the XZ axes
						float xzmagn = Mathf.Sqrt (velocity.x*velocity.x + velocity.z*velocity.z);
						xzmagn = maxSpeed/xzmagn;
						if ( xzmagn < 1 ) {
							velocity.x *= xzmagn;
							velocity.z *= xzmagn;
							//Vector3.ClampMagnitude (velocity, maxSpeed);
						}
					}
					
					//Debug.DrawLine (tr.position, tg, lastCorner ? Color.red : Color.green);
					
					
					if (endPointIsTarget) {
						Vector3 trotdir = Vector3.Lerp(velocity,currentTargetDirection, System.Math.Max (1 - magn*2,0));
						RotateTowards(trotdir);
					} else {
						RotateTowards(velocity);
					}
					
					//Applied after rotation to enable proper checks on if velocity is zero
					velocity += deltaTime * gravity;
					
					if (rvoController != null && rvoController.enabled) {

						//Use RVOController
						tr.position = position;
						rvoController.Move (velocity);
						
					} else
					if (controller != null && controller.enabled) {
						
						//Use CharacterController
						tr.position = position;
						controller.Move (velocity * deltaTime);
						
					} else {
						
						//Use Transform
						float lasty = position.y;
						position += velocity*deltaTime;
						
						position = RaycastPosition (position, lasty);
	
						tr.position = position;
					}
				}
				else {
					if (rvoController != null && rvoController.enabled) {
						//Use RVOController
						rvoController.Move (Vector3.zero);
					}
				}
				
				if (pt is RichSpecial) {
					RichSpecial rs = pt as RichSpecial;
					
					if (!traversingSpecialPath) {
						StartCoroutine(TraverseSpecial(rs));
					}
				}
				//w.Stop();
				//Debug.Log ((w.Elapsed.TotalMilliseconds*1000));
				
			} else {
				if (rvoController != null && rvoController.enabled) {
					//Use RVOController
					rvoController.Move (Vector3.zero);
				} else
				if (controller != null && controller.enabled) {
				} else {
					tr.position = RaycastPosition (tr.position, tr.position.y);
				}
			}
		}
		
		Vector3 RaycastPosition (Vector3 position, float lasty) {
			if (raycastingForGroundPlacement) {
				RaycastHit hit;
				float up = Mathf.Max (centerOffset, lasty-position.y+centerOffset);
	
				if (Physics.Raycast (position+Vector3.up*up,Vector3.down,out hit,up,groundMask)) {
					//Debug.DrawRay (tr.position+Vector3.up*centerOffset,Vector3.down*centerOffset, Color.red);
					if (hit.distance < up) {
						//grounded
						position = hit.point;//.up * -(hit.distance-centerOffset);
						velocity.y = 0;
					}
				} else {
					//Debug.DrawRay (tr.position+Vector3.up*centerOffset,Vector3.down*centerOffset, Color.green);
				}
			}
			return position;
		}
		
		/** Rotates along the Y-axis the transform towards \a trotdir */
		bool RotateTowards (Vector3 trotdir) {
			Quaternion rot = tr.rotation;
			
			trotdir.y = 0;
			if (trotdir != Vector3.zero) {
				Vector3 trot = Quaternion.LookRotation (trotdir).eulerAngles;
				Vector3 eul = rot.eulerAngles;
				eul.y = Mathf.MoveTowardsAngle(eul.y,trot.y,rotationSpeed*deltaTime);
				tr.rotation = Quaternion.Euler(eul);
				//Magic number, should expose as variable
				return Mathf.Abs(eul.y-trot.y) < 5f;
			}
			return false;
		}
		
		public static readonly Color GizmoColorRaycast = new Color(118.0f/255,206.0f/255,112.0f/255);
		public static readonly Color GizmoColorPath = new Color(8.0f/255,78.0f/255,194.0f/255);
		
		public void OnDrawGizmos () {
			if (drawGizmos) {
				if (raycastingForGroundPlacement) {
					Gizmos.color = GizmoColorRaycast;
					Gizmos.DrawLine (transform.position, transform.position+Vector3.up*centerOffset);
					Gizmos.DrawLine (transform.position + Vector3.left*0.1f, transform.position + Vector3.right*0.1f);
					Gizmos.DrawLine (transform.position + Vector3.back*0.1f, transform.position + Vector3.forward*0.1f);
				}
				
				if (tr != null && buffer != null) {
					Gizmos.color = GizmoColorPath;
					Vector3 p=tr.position;
					for (int i=0;i<buffer.Count;p=buffer[i], i++) {
						Gizmos.DrawLine (p, buffer[i]);
					}
				}
			}
		}
			
		IEnumerator TraverseSpecial (RichSpecial rs) {
			traversingSpecialPath = true;
			velocity = Vector3.zero;
			
			AnimationLink al = rs.nodeLink as AnimationLink;
			if (al == null) {
				Debug.LogError ("Unhandled RichSpecial");
				yield break;
			}
			
			//Rotate character to face the correct direction
			while (!RotateTowards(rs.first.forward)) yield return null;
			
			//Reposition
			tr.parent.position = tr.position;
			
			tr.parent.rotation = tr.rotation;
			tr.localPosition = Vector3.zero;
			tr.localRotation = Quaternion.identity;
			
			//Set up animation speeds
			if (rs.reverse && al.reverseAnim) {
				anim[al.clip].speed = -al.animSpeed;
				anim[al.clip].normalizedTime = 1;
				anim.Play(al.clip);
				anim.Sample();
			} else {
				anim[al.clip].speed = al.animSpeed;
				anim.Rewind(al.clip);
				anim.Play(al.clip);
			}
			
			//Fix required for animations in reverse direction
			tr.parent.position -= tr.position-tr.parent.position;
			
			//Wait for the animation to finish
			yield return new WaitForSeconds(Mathf.Abs(anim[al.clip].length/al.animSpeed));
			
			traversingSpecialPath = false;
			NextPart ();
			
			//If a path completed during the time we traversed the special connection, we need to recalculate it
			if (delayUpdatePath) {
				delayUpdatePath = false;
				UpdatePath();
			}
		}
	}
}