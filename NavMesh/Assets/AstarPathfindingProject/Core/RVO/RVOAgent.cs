using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.RVO;

namespace Pathfinding.RVO.Sampled {

	public class Agent : IAgent {

		Vector3 smoothPos;
		
		public Vector3 Position {
			get;
			set;
		}
		
		public Vector3 InterpolatedPosition {
			get { return smoothPos; }
		}
		
		public Vector3 DesiredVelocity { get; set; }
		
		public void Teleport (Vector3 pos) {
			Position = pos;
			smoothPos = pos;
			prevSmoothPos = pos;
		}
		
		//Current values for double buffer calculation
		
		public float radius, height, maxSpeed, neighbourDist, agentTimeHorizon, obstacleTimeHorizon, weight;
		public bool locked = false;

		RVOLayer layer, collidesWith;

		public int maxNeighbours;
		public Vector3 position, desiredVelocity, prevSmoothPos;

		public RVOLayer Layer {get; set;}
		public RVOLayer CollidesWith {get; set;}

		public bool Locked {get; set;}
		public float Radius {get; set;}
		public float Height {get; set;}
		public float MaxSpeed {get; set;}
		public float NeighbourDist {get; set;}
		public float AgentTimeHorizon {get; set;}
		public float ObstacleTimeHorizon {get; set;}
		public Vector3 Velocity {get; set;}
		public bool DebugDraw {get; set;}
	
		public int MaxNeighbours {get; set;}

		/** Used internally for a linked list */
		internal Agent next;

		private Vector3 velocity;
		private Vector3 newVelocity;
		
		/** Simulator which handles this agent.
		 * Used by this script as a reference and to prevent
		 * adding this agent to multiple simulations.
		 */
		public Simulator simulator;
		
		public List<Agent> neighbours = new List<Agent>();
		public List<float> neighbourDists = new List<float>();
		List<ObstacleVertex> obstaclesBuffered = new List<ObstacleVertex>();
		List<ObstacleVertex> obstacles = new List<ObstacleVertex>();
		List<float> obstacleDists = new List<float>();

		public List<ObstacleVertex> NeighbourObstacles {
			get {
				return null;
			}
		}

		public Agent (Vector3 pos) {
			MaxSpeed = 2;
			NeighbourDist = 15;
			AgentTimeHorizon = 2;
			ObstacleTimeHorizon = 2;
			Height = 5;
			Radius = 5;
			MaxNeighbours = 10;
			Locked = false;
			
			position = pos;
			Position = position;
			prevSmoothPos = position;
			smoothPos = position;
			CollidesWith = (RVOLayer)(-1);
		}
		
		public void BufferSwitch () {
			// <==
			radius = Radius;
			height = Height;
			maxSpeed = MaxSpeed;
			neighbourDist = NeighbourDist;
			agentTimeHorizon = AgentTimeHorizon;
			obstacleTimeHorizon = ObstacleTimeHorizon;
			maxNeighbours = MaxNeighbours;
			desiredVelocity = DesiredVelocity;
			locked = Locked;
			collidesWith = CollidesWith;
			layer = Layer;

			//position = Position;
			
			// ==>
			Velocity = velocity;
			List<ObstacleVertex> tmp = obstaclesBuffered;
			obstaclesBuffered = obstacles;
			obstacles = tmp;
		}

		// Update is called once per frame
		public void Update () {
			velocity = newVelocity;
			
			prevSmoothPos = smoothPos;
			
			//Note the case P/p
			position = Position;
			position = position + velocity * simulator.DeltaTime;
			Position = position;
		}
		
		public void Interpolate (float t) {
			if (t == 1.0f) smoothPos = Position;
			else smoothPos = prevSmoothPos + (Position-prevSmoothPos)*t;
		}
		
		public static System.Diagnostics.Stopwatch watch1 = new System.Diagnostics.Stopwatch();
		public static System.Diagnostics.Stopwatch watch2 = new System.Diagnostics.Stopwatch();
		
		public void CalculateNeighbours () {

			neighbours.Clear ();
			neighbourDists.Clear ();

			float rangeSq;

			if ( locked ) return;

			//watch1.Start ();
			if (MaxNeighbours > 0) {
				rangeSq = neighbourDist*neighbourDist;// 
				
				//simulator.KDTree.GetAgentNeighbours (this, rangeSq);
				simulator.Quadtree.Query (new Vector2(position.x,position.z), neighbourDist, this);

			}
			//watch1.Stop ();

			obstacles.Clear ();
			obstacleDists.Clear ();

			rangeSq = (obstacleTimeHorizon * maxSpeed + radius);
			rangeSq *= rangeSq;
			// Obstacles disabled at the moment
			//simulator.KDTree.GetObstacleNeighbours (this, rangeSq);
			
		}

		float Sqr ( float x ) {
			return x*x;
		}

		public float InsertAgentNeighbour (Agent agent, float rangeSq) {
			if (this == agent) return rangeSq;

			if ( (agent.layer & collidesWith) == 0 ) return rangeSq;

			//2D Dist
			float dist = Sqr(agent.position.x-position.x) + Sqr(agent.position.z - position.z);
			
			if (dist < rangeSq) {
				if (neighbours.Count < maxNeighbours) {
					neighbours.Add (agent);
					neighbourDists.Add (dist);
				}

				int i = neighbours.Count-1;
				if ( dist < neighbourDists[i] ) {
					while ( i != 0 && dist < neighbourDists[i-1]) {
						neighbours[i] = neighbours[i-1];
						neighbourDists[i] = neighbourDists[i-1];
						i--;
					}
					neighbours[i] = agent;
					neighbourDists[i] = dist;
				}

				if (neighbours.Count == maxNeighbours) {
					rangeSq = neighbourDists[neighbourDists.Count-1];
				}
			}
			return rangeSq;
		}

		/*public void UpdateNeighbours () {
			neighbours.Clear ();
			float sqrDist = neighbourDistance*neighbourDistance;
			for ( int i = 0; i < simulator.agents.Count; i++ ) {
				float dist = (simulator.agents[i].position - position).sqrMagnitude;
				if ( dist <= sqrDist ) {
					neighbours.Add ( simulator.agents[i] );
				}
			}
		}*/

		public void InsertObstacleNeighbour (ObstacleVertex ob1, float rangeSq) {
			ObstacleVertex ob2 = ob1.next;
			
			float dist = AstarMath.DistancePointSegmentStrict (ob1.position,ob2.position, Position);
			
			if (dist < rangeSq) {
				obstacles.Add (ob1);
				obstacleDists.Add (dist);
				
				int i = obstacles.Count-1;
				while ( i != 0 && dist < obstacleDists[i-1]) {
					obstacles[i] = obstacles[i-1];
					obstacleDists[i] = obstacleDists[i-1];
					i--;
				}
				obstacles[i] = ob1;
				obstacleDists[i] = dist;
			}
		}

		static Vector3 To3D ( Vector2 p ) {
			return new Vector3 ( p.x, 0, p.y );
		}

		static void DrawCircle ( Vector2 _p, float radius, Color col ) {
			DrawCircle ( _p, radius, 0, 2*Mathf.PI, col );
		}

		static void DrawCircle ( Vector2 _p, float radius, float a0, float a1, Color col ) {
			Vector3 p = To3D (_p);

			while ( a0 > a1 ) a0 -= 2*Mathf.PI;

			Vector3 prev = new Vector3 (Mathf.Cos(a0)*radius, 0, Mathf.Sin(a0)*radius);
			const float steps = 40.0f;
			for ( int i = 0; i <= steps; i++ ) {
				Vector3 c = new Vector3 (Mathf.Cos(Mathf.Lerp(a0,a1,i/steps))*radius, 0, Mathf.Sin(Mathf.Lerp(a0,a1,i/steps))*radius);
				Debug.DrawLine ( p+prev, p+c, col );
				prev = c;
			}
		}

		static void DrawVO ( Vector2 circleCenter, float radius, Vector2 origin ) {
			float alpha = Mathf.Atan2 ( (origin - circleCenter).y, (origin - circleCenter).x );
			float gamma = radius/(origin-circleCenter).magnitude;
			float delta = gamma <= 1.0f ? Mathf.Abs(Mathf.Acos (gamma)) : 0;

			DrawCircle ( circleCenter, radius, alpha-delta, alpha+delta, Color.black );
			Vector2 p1 = new Vector2 ( Mathf.Cos (alpha-delta), Mathf.Sin (alpha-delta) ) * radius;
			Vector2 p2 = new Vector2 ( Mathf.Cos (alpha+delta), Mathf.Sin (alpha+delta) ) * radius;

			Vector2 p1t = -new Vector2(-p1.y, p1.x);
			Vector2 p2t = new Vector2(-p2.y, p2.x);
			p1 += circleCenter;
			p2 += circleCenter;

			Debug.DrawRay ( To3D(p1), To3D(p1t).normalized*100, Color.black );
			Debug.DrawRay ( To3D(p2), To3D(p2t).normalized*100, Color.black );
		}

		static void DrawCross ( Vector2 p, float size = 1 ) {
			DrawCross ( p, Color.white, size );
		}

		static void DrawCross ( Vector2 p, Color col, float size = 1 ) {
			size *= 0.5f;
			Debug.DrawLine ( new Vector3(p.x,0,p.y) - Vector3.right*size, new Vector3(p.x,0,p.y) + Vector3.right*size, col );
			Debug.DrawLine ( new Vector3(p.x,0,p.y) - Vector3.forward*size, new Vector3(p.x,0,p.y) + Vector3.forward*size, col );
		}

		public struct VO {

			//public 
			public Vector2 origin;

			//public float radius;

			Vector2 line1, line2, dir1, dir2;

			Vector2 cutoffLine, cutoffDir;

			float sqrCutoffDistance;
			bool leftSide;

			bool colliding;

			public VO ( Vector2 center, Vector2 offset, float radius, Vector2 sideChooser, float inverseDt ) {
				//this.radius = radius;
				Vector2 globalCenter;
				this.origin = offset;
				// Collision?
				if ( center.magnitude < radius ) {
					colliding = true;
					leftSide = false;

					line1 = center.normalized * (center.magnitude - radius);
					dir1 = new Vector2(line1.y,-line1.x).normalized;
					line1 += offset;
					return;
				}

				colliding = false;

				center *= inverseDt;
				radius *= inverseDt;
				globalCenter = center+offset;

				sqrCutoffDistance = center.magnitude - radius;

				cutoffLine = center.normalized * sqrCutoffDistance;
				cutoffDir = new Vector2(-cutoffLine.y,cutoffLine.x).normalized;
				cutoffLine += offset;

				sqrCutoffDistance *= sqrCutoffDistance;
				float alpha = Mathf.Atan2 ( -center.y, -center.x );
				
				float delta = Mathf.Abs(Mathf.Acos (radius/center.magnitude));

				// Bounding Lines

				leftSide = Polygon.Left ( Vector2.zero, center, sideChooser );

				// Point on circle
				line1 = new Vector2 ( Mathf.Cos (alpha+delta), Mathf.Sin (alpha+delta) ) * radius;
				// Vector tangent to circle which is the correct line tangent
				dir1 = new Vector2(line1.y,-line1.x).normalized;

				// Point on circle
				line2 = new Vector2 ( Mathf.Cos (alpha-delta), Mathf.Sin (alpha-delta) ) * radius;
				// Vector tangent to circle which is the correct line tangent
				dir2 = new Vector2(line2.y,-line2.x).normalized;

				line1 += globalCenter;
				line2 += globalCenter;

				//Debug.DrawRay ( To3D(line1), To3D(dir1), Color.cyan );
				//Debug.DrawRay ( To3D(line2), To3D(dir2), Color.cyan );
			}

			/** Returns if \a p lies on the left side of a line which with one point in \a a and has a tangent in the direction of \a dir.
			 * Also returns true if the points are colinear */
			static bool Left (Vector2 a, Vector2 dir, Vector2 p) {
				return (dir.x) * (p.y - a.y) - (p.x - a.x) * (dir.y) <= 0;
			}

			/** Returns if \a p lies on the left side of a line which with one point in \a a and has a tangent in the direction of \a dir.
			 * Also returns true if the points are colinear */
			static float Det (Vector2 a, Vector2 dir, Vector2 p) {
				return (p.x - a.x) * (dir.y) - (dir.x) * (p.y - a.y);
			}

			public Vector2 Sample ( Vector2 p, out float weight ) {

				if ( colliding ) {
					// Calculate double signed area of the triangle consisting of the points
					// {line1, line1+dir1, p}
					float l1 = Det (line1, dir1, p );

					// Serves as a check for which side of the line the point p is
					if ( l1 >= 0 ) {
						/*float dot1 = Vector2.Dot ( p - line1, dir1 );
						
						Vector2 c1 = dot1 * dir1 + line1;
						return (c1-p);*/
						weight = l1*0.5f;
						return new Vector2(-dir1.y, dir1.x)*weight*GlobalIncompressibility; // 10 is an arbitrary constant signifying incompressability
						// (the higher the value, the more the agents will avoid penetration)
					} else {
						weight = 0;
						return new Vector2(0,0);
					}
				}

				{
					float det1 = Det (line1, dir1, p );
					float det2 = Det (line2, dir2, p );
					if ( det1 >= 0 && det2 >= 0 ) {
						/*float magn = ( p - origin ).sqrMagnitude;
						if ( magn < sqrCutoffDistance ) {
							weight = 0;
							return Vector2.zero;
						}*/
						float det3 = Det (cutoffLine, cutoffDir, p );
						if ( det3 <= 0 ) {
							weight = 0;
							return Vector2.zero;
						}

						if (leftSide) {
							if ( det3 < det1 ) {
								weight = det3*0.5f;
								return new Vector2(-cutoffDir.y, cutoffDir.x)*weight;
							}

							weight = det1*0.5f;
							return new Vector2(-dir1.y, dir1.x)*weight;
						} else {
							if ( det2 < det1 ) {
								weight = det2*0.5f;
								return new Vector2(-cutoffDir.y, cutoffDir.x)*weight;
							}

							weight = det2*0.5f;
							return new Vector2(-dir2.y, dir2.x)*weight;
						}
					}
				}

				weight = 0;
				return new Vector2(0,0);
			}
		}

		internal void CalculateVelocity ( Pathfinding.RVO.Simulator.WorkerContext context ) {

			if ( locked ) {
				newVelocity = Vector2.zero;
				return;
			}

			if ( context.vos.Length < neighbours.Count ) {
				context.vos = new VO[Mathf.Max(context.vos.Length*2, neighbours.Count)];
			}

			Vector2 position2D = new Vector2(position.x,position.z);

			var vos = context.vos;
			var voCount = 0;

			Vector2 optimalVelocity = new Vector2(velocity.x, velocity.z);

			float inverseAgentTimeHorizon = 1.0f/agentTimeHorizon;

			for ( int o = 0; o < neighbours.Count; o++ ) {

				Agent other = neighbours[o];

				if ( other == this ) continue;

				float maxY = System.Math.Min (position.y+height,other.position.y+other.height);
				float minY = System.Math.Max (position.y,other.position.y);
				
				//The agents cannot collide since they
				//are on different y-levels
				if (maxY - minY < 0) {
					continue;
				}

				Vector2 otherOptimalVelocity = new Vector2(other.Velocity.x, other.velocity.z);


				float totalRadius = radius + other.radius;

				// Describes a circle on the border of the VO
				//float boundingRadius = totalRadius * inverseAgentTimeHorizon;
				Vector2 voBoundingOrigin = new Vector2(other.position.x,other.position.z) - position2D;

				//float boundingDist = voBoundingOrigin.magnitude;

				Vector2 relativeVelocity = optimalVelocity - otherOptimalVelocity;

				{
					//voBoundingOrigin *= inverseAgentTimeHorizon;
					//boundingDist *= inverseAgentTimeHorizon;
					
					// Common case, no collision
					
					vos[voCount] = new VO( voBoundingOrigin, (optimalVelocity + otherOptimalVelocity)*0.5f, totalRadius, relativeVelocity, inverseAgentTimeHorizon);
					voCount++;
				}

	#if OLD
				//Debug.Log (boundingDist + " " + total
				if ( boundingDist < totalRadius ) {
					// Collision

					// We want the collision to end this frame, so ignore the inverseAgentTimeHorizon and
					// instead use the frame's delta time instead. The closest point will not always be 
					// on the circle, but the velocity will have to be really
					// high for it not to be, so just assume the velocity is low
					// (if not, the agents will jump through each other)

					voBoundingOrigin *= (1.0f/simulator.DeltaTime);
					boundingDist *= (1.0f/simulator.DeltaTime);

					Vector2 d = relativeVelocity - voBoundingOrigin;
					float dm = d.magnitude;

					// Normalize
					d /= dm;

					float resMagn = (totalRadius*(1.0f/simulator.DeltaTime) - dm);
					maxRelVelMagn = System.Math.Max ( resMagn, maxRelVelMagn );

					Vector2 closestOnCircleDelta = d*resMagn;//d*(boundingDist-dm);
					Vector2 closestOnCircleVector = d;

					halfPlanes.Add ( new HalfPlane ( closestOnCircleDelta*0.5f + optimalVelocity, closestOnCircleVector ) );
					if (debug) halfPlanes[halfPlanes.Count-1].Draw ( Color.blue );
				} else {
					voBoundingOrigin *= inverseAgentTimeHorizon;
					boundingDist *= inverseAgentTimeHorizon;

					// Common case, no collision

					VO vo = new VO( voBoundingOrigin + (optimalVelocity + otherOptimalVelocity)*0.5f, voBoundingOrigin.normalized, totalRadius*inverseAgentTimeHorizon);

					maxRelVelMagn = System.Math.Max ( maxRelVelMagn, relativeVelocity.magnitude );
					// Optimized: float factor = Vector3.Dot ( optimalVelocity, voBoundingOrigin ) < (boundingDist^2);

					if (debug) DrawVO ( voBoundingOrigin, boundingRadius, Vector2.zero);

					if ( debug ) {
						//Debug.DrawLine ( To3D(position), To3D ( relativeVelocity ), Color.blue );
						DrawCross ( relativeVelocity, Color.white, 0.5f);

					}

					//DrawCross ( voBoundingOrigin );

					float alpha = Mathf.Atan2 ( -voBoundingOrigin.y, -voBoundingOrigin.x );

					float delta = Mathf.Abs(Mathf.Acos (boundingRadius/(voBoundingOrigin).magnitude));

					float actualDot = Vector2.Dot ( -voBoundingOrigin.normalized, (relativeVelocity-voBoundingOrigin).normalized );
					float dot = Mathf.Abs ( Mathf.Acos ( Mathf.Clamp01(actualDot) ) );

					bool closestIsCircle = dot < delta;
					/*DrawCircle ( circleCenter, radius, alpha-delta, alpha+delta, Color.black );
					Vector2 p1 = circleCenter + new Vector2 ( Mathf.Cos (alpha-delta), Mathf.Sin (alpha-delta) ) * boundingRadius;
					Vector2 p2 = circleCenter + new Vector2 ( Mathf.Cos (alpha+delta), Mathf.Sin (alpha+delta) ) * boundingRadius;*/

					if ( debug ) Debug.Log (" <= 1 && " + dot + " < " + delta + " ( " +actualDot + ")");
					if ( closestIsCircle ) {
						// The cutoff circle is the closest 
						// part of the VO
						// Find the closest point on the VO

						Vector2 closestOnCircleVector = (relativeVelocity - voBoundingOrigin);
						float magn = closestOnCircleVector.magnitude;
						closestOnCircleVector = closestOnCircleVector/magn;//(closestOnCircleVector/magn)*boundingRadius*(1-(magn/boundingRadius));
						Vector2 closestOnCircle = closestOnCircleVector*boundingRadius + voBoundingOrigin;

						//if ( magn > boundingRadius ) closestOnCircleVector = -closestOnCircleVector;
						if ( debug ) DrawCross ( closestOnCircle, 1);
						halfPlanes.Add ( new HalfPlane ( (closestOnCircle - relativeVelocity)*0.5f + optimalVelocity, closestOnCircleVector.normalized ) );
						if (debug) halfPlanes[halfPlanes.Count-1].Draw ( closestIsCircle ? Color.blue : Color.red );

					} else {
						// One of the legs of the VO is the closest part
						// find the closest point on the VO

						bool left = Polygon.Left ( Vector2.zero, voBoundingOrigin, relativeVelocity );
						float gamma = alpha + (left ? delta : -delta);

						// Point on circle
						Vector2 line = new Vector2 ( Mathf.Cos (gamma), Mathf.Sin (gamma) ) * boundingRadius;
						// Vector tangent to circle which is the correct line tangent
						Vector2 dir = new Vector2(line.y,-line.x).normalized;

						// Point in space
						line += voBoundingOrigin;

						Vector2 onLine = line + dir * Vector2.Dot ( dir, relativeVelocity - line);
						if (debug) {
							DrawCross ( onLine, Color.red, 1 );
							Debug.DrawRay ( To3D(line), To3D(dir*10), Color.red );
							Debug.Log (line + " " + dir + " " + gamma + " " + relativeVelocity);
						}

						if ( !left ) dir = -dir;

						halfPlanes.Add ( new HalfPlane ( (onLine - relativeVelocity)*0.5f + optimalVelocity, (line-voBoundingOrigin).normalized ) );
						if (debug) halfPlanes[halfPlanes.Count-1].Draw ( Color.blue );
					}
				}
	#endif
			}

			if ( this.DebugDraw ) {
				/*for ( int x = 0; x < simulator.tex.width; x++ ) {
					for ( int y = 0; y < simulator.tex.height; y++ ) {
						Vector2 p = new Vector2 (x*simulator.textureSize / simulator.tex.width, y*simulator.textureSize / simulator.tex.height);

						Vector2 dir = Vector2.zero;
						float weight = 0;
						for ( int i = 0; i < voCount; i++ ) {
							float w = 0;
							dir += vos[i].Sample (p-position, out w);
							if ( w > weight ) weight = w;
						}
						Vector2 d2 = (desiredVelocity - (p-position));
						dir += d2*DesiredVelocityScale;

						if ( d2.magnitude * DesiredVelocityWeight > weight ) weight = d2.magnitude * DesiredVelocityWeight;

						if ( weight > 0 ) dir /= weight;

						Vector2 d3 = simulator.SampleDensity (p+position);
						Debug.DrawRay ( To3D(p), To3D(d3*1f), Color.blue );
						simulator.Plot (p, Rainbow(weight*simulator.colorScale));
					}
				}*/
			}

			//if ( debug ) {
			float best = float.PositiveInfinity;
			Vector2 result = Vector2.zero;

			float cutoff = new Vector2(velocity.x,velocity.z).magnitude*simulator.qualityCutoff;

			//for ( int i = 0; i < 10; i++ ) {
			{
				result = Trace ( vos, voCount, new Vector2(desiredVelocity.x, desiredVelocity.z), cutoff, out best );
			}

			// Can be uncommented for higher quality local avoidance
			/*for ( int i = 0; i < 3; i++ ) {
				Vector2 p = desiredVelocity + new Vector2(Mathf.Cos(Mathf.PI*2*(i/3.0f)), Mathf.Sin(Mathf.PI*2*(i/3.0f)));
				float score;
				Vector2 res = Trace ( vos, voCount, p, velocity.magnitude*simulator.qualityCutoff, out score );
				
				if ( score < best ) {
					//if ( score < best*0.9f ) Debug.Log ("Better " + score + " < " + best);
					result = res;
					best = score;
				}
			}*/

			{
				Vector2 p = Vector2.zero;
				float score;
				Vector2 res = Trace ( vos, voCount, p, cutoff, out score );
				
				if ( score < best ) {
					//if ( score < best*0.9f ) Debug.Log ("Better " + score + " < " + best);
					result = res;
					best = score;
				}
			}
			//}
				

			if (DebugDraw) DrawCross (result+position2D);


			newVelocity = To3D(Vector2.ClampMagnitude (result, maxSpeed));
		}

		public static float DesiredVelocityWeight = 0.1f;
		public static float DesiredVelocityScale = 0.1f;
		public static float GlobalIncompressibility = 30;

		static Color Rainbow ( float v ) {
			Color c = new Color(v,0,0);
			if ( c.r > 1 ) { c.g = c.r - 1; c.r = 1; }
			if ( c.g > 1 ) { c.b = c.g - 1; c.g = 1; }
			return c;
		
		}

		Vector2 Trace ( VO[] vos, int voCount, Vector2 p, float cutoff, out float score ) {

			score = 0;
			float stepScale = simulator.stepScale;

			//while ( true ) {
			for ( int s = 0; s < 50; s++ ) {


				float step = 1.0f - (s/50.0f);
				step *= stepScale;

				Vector2 dir = Vector2.zero;
				float mx = 0;
				for ( int i = 0; i < voCount; i++ ) {
					float w;
					Vector2 d = vos[i].Sample (p, out w);
					dir += d;

					if ( w > mx ) mx = w;
					//mx = System.Math.Max (mx, d.sqrMagnitude);
				}

				Vector2 d2 = (new Vector2(desiredVelocity.x,desiredVelocity.z) - p);
				float weight = d2.magnitude*DesiredVelocityWeight;
				dir += d2*DesiredVelocityScale;
				mx = System.Math.Max (mx, weight);

				/*if ( simulator.densityScale > 0 ) {
					d2 = simulator.SampleDensity (p+position);
					dir += d2;
					mx = System.Math.Max (mx, d2.magnitude);
				}*/

				score = mx;//dir.sqrMagnitude;

				float sq = dir.sqrMagnitude;
				if ( sq > 0 ) dir *= mx/Mathf.Sqrt(sq); // +1 to avoid division by zero
					//Vector2.ClampMagnitude (dir, Mathf.Sqrt(mx));
				//dir /= vos.Count+1;

				dir *= step;

				//Vector2 prev = p;
				p += dir;

				if ( score < cutoff ) break;
				//if (debug) Debug.DrawLine ( To3D(prev+position), To3D(p+position), Color.green);
			}

			return p;
		}

		/** Returns the intersection factors for line 1 and line 2. The intersection factors is a distance along the line \a start - \a end where the other line intersects it.\n
		 * \code intersectionPoint = start1 + factor1 * (end1-start1) \endcode
		 * \code intersectionPoint2 = start2 + factor2 * (end2-start2) \endcode
		 * Lines are treated as infinite.\n
		 * false is returned if the lines are parallel and true if they are not.
		 */
		public static bool IntersectionFactor (Vector2 start1, Vector2 dir1, Vector2 start2, Vector2 dir2, out float factor ) {

			float den = dir2.y*dir1.x - dir2.x * dir1.y;

			// Parallel
			if (den == 0) {
				factor = 0;
				return false;
			}

			float nom = dir2.x*(start1.y-start2.y)- dir2.y*(start1.x-start2.x);
			
			factor = nom/den;

			return true;
		}
	}
}
