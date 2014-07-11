using UnityEngine;
using System.Collections;
using System;
using System.Collections.Generic;
using Pathfinding;

namespace Pathfinding {
	/** Local Avoidance (beta) [obsolete].
	 * This component can be used as a CharacterController with the call SimpleMove.\n
	 * Using this on multiple units will improve how they behave in groups and avoid collisions.\n
	 * Known limitations and issues
	 * 	- Many units all trying to go through the same point will take quite some time to sort out
	 * 	- Many units trying to go to the same point will sometimes result in overlapping.
	 * 	- Only works in 2D (i.e on in XZ space), multiple floors are not handled properly yet.
	 * 	- Only works with agents which were present from the beginning of the game (otherwise you need to update the LocalAvoidance.agents array, see LocalAvoidance.Start)
	 * 	- Units do not avoid walls at the moment, support for avoiding at least navmesh edges will be implemented in later versions
	 *
	 * \n
	 * See demo of the script here: http://www.youtube.com/watch?v=WUOymX-NpN4
	 * \n
	 * Keep in mind that this is a beta, some features might not work properly and the script is really messy.\n
	 * \n
	 * Based on ClearPath, RVO, VO and a number of other techniques.
	 * This is roughly the same technique which Unity's local avoidance uses.
	 * 
	 * \deprecated This class has been replaced by the much more mature RVO local avoidance.
	 * It may be removed in future versions.
	 */
	[RequireComponent(typeof(CharacterController))]
	public class LocalAvoidance : MonoBehaviour {
		
		public float speed = 2F;
		/** How many seconds to plan for. 1.0 usually works best */
		public float delta = 1.0F;
		
		/** Blending between taking this vecocity or other units velocity into account.
		 * 0.5 usually works best */
		public float responability = 0.5F;
		
		/** Initialization. Caches CharacterController component and finds other agents */
		void Start () {
			controller = GetComponent<CharacterController>();
			agents = FindObjectsOfType (typeof(LocalAvoidance)) as LocalAvoidance[];
			//targetPoint = transform.position+transform.forward*targetPointDistance;
		}
		
		public void Update () {
			SimpleMove (transform.forward*speed);
		}
		
		/** Type of solver for local avoidance.
		 * The Geometric one is the one most developed and works best usually.\n
		 * The sampled one also works quite good, but not in all cases */
		public ResolutionType resType = ResolutionType.Geometric;
		
		/** Velocity during last frame */
		private Vector3 velocity;
		
		public float radius = 0.5F;
		
		/** Based on the desired velocity, how much larger magnitude should the output be allowed to have */
		public float maxSpeedScale = 1.5F;
		
		public Vector3[] samples;
		public float sampleScale = 1F;
		public float circleScale = 0.5F;
		public float circlePoint = 0.5F;
		
		public bool drawGizmos = false;
		
		protected CharacterController controller;
		protected LocalAvoidance[] agents;
		
		private Vector3 preVelocity;
		
		public Vector3 GetVelocity () {
			return preVelocity;
		}
		
		public enum ResolutionType {
			Sampled,
			Geometric
		}
		
		// Update is called once per frame
		/*void FixedUpdate () {
			
			/*if (delta < 0.25F) {
				return;
			}
			
			Vector3 desiredMovement = Vector3.ClampMagnitude (targetPoint-transform.position ,speed);//transform.forward*speed;
			//desiredMovement += UnityEngine.Random.onUnitSphere*0.01F;
			
			Vector3 targetMovement = ClampMovement (desiredMovement);
			if (targetMovement != Vector3.zero) {
				targetMovement /= delta;
			}
			
			if (drawGizmos) {
				Debug.DrawRay (transform.position,desiredMovement,Color.magenta);
				Debug.DrawRay (transform.position,targetMovement,Color.yellow);
				Debug.DrawRay (transform.position+targetMovement,Vector3.up,Color.yellow);
			}
			
			//velocity = controller.velocity;
			velocity = Vector3.Lerp (velocity,targetMovement,0.9F);
			
			controller.Move (velocity*Time.fixedDeltaTime);
			
			velocity = controller.velocity;
			//
			
			Debug.DrawRay (transform.position,velocity,Color.grey);*
		}*/
		
		public void LateUpdate () {
			preVelocity = velocity;
		}
		
		/** Simple move. Similar to CharacterController.SimpleMove, but this will try to avoid other units */
		public void SimpleMove (Vector3 desiredMovement) {
			
			Vector3 rnd = UnityEngine.Random.insideUnitSphere*0.1F;
			rnd.y = 0;
			
			Vector3 targetMovement = ClampMovement (desiredMovement + rnd);
			
			if (targetMovement != Vector3.zero) {
				targetMovement /= delta;
			}
			
			if (drawGizmos) {
				Debug.DrawRay (transform.position,desiredMovement,Color.magenta);
				Debug.DrawRay (transform.position,targetMovement,Color.yellow);
				Debug.DrawRay (transform.position+targetMovement,Vector3.up,Color.yellow);
			}
			
			controller.SimpleMove (targetMovement);
			velocity = controller.velocity;
			Debug.DrawRay (transform.position,velocity,Color.blue);
			
			
		}
		
		public const float Rad2Deg = (float)(360.0/(Math.PI*2));
		const int maxVOCounter = 50;
		
		List<VO> vos = new List<VO> ();
		
		public Vector3 ClampMovement (Vector3 direction) {
			
			Vector3 targetDir = direction*delta;
			Vector3 targetPoint = transform.position+direction;
			Vector3 bestPoint = targetPoint;
			float 	bestPointDist = 0;
			
			//float lowestVO = float.PositiveInfinity;
			int lowestVOCounter = 0;
			
			
			//List<VO> vos = new List<VO> ();
			vos.Clear ();
			
			float velocityMagn = velocity.magnitude;
			
			foreach (LocalAvoidance agent in agents) {
				if (agent == this || agent == null) {
					continue;
				}
				
				
				Vector3 rCenter = agent.transform.position-transform.position;
				float dist = rCenter.magnitude;
				float doubleRad = (radius+agent.radius);
				
				if (dist > targetDir.magnitude*delta+doubleRad + velocityMagn + agent.GetVelocity ().magnitude) {
					continue;
				}
				
				if (lowestVOCounter > maxVOCounter) {
					continue;
				} else {
					lowestVOCounter++;
				}
				
				VO vo = new VO ();
				
				vo.origin = transform.position + Vector3.Lerp (velocity*delta,agent.GetVelocity ()*delta,responability);
				vo.direction = rCenter.normalized;
				
				
				
				if (doubleRad > rCenter.magnitude) {
					vo.angle = Mathf.PI*0.5F;
					/*vo.limit = dist-doubleRad;
					if (vo.limit < 0) {
						vo.limit = 0.001F;
					}
					
					vos.Add (vo);
					continue;*/
					//doubleRad = rCenter.magnitude*0.99F;
					//continue;
				} else {
					vo.angle = (float)Math.Asin (doubleRad/dist);
				}
				
				//float n = (float)Math.Tan (Math.Asin ((doubleRad)/rCenter.magnitude))*(rCenter.magnitude-doubleRad);
				//Vector3 m = (rCenter.magnitude - doubleRad)*rCenter.normalized;//+(agent.velocity+velocity)/2F;
				/*if (drawGizmos) {
					Debug.DrawLine (transform.position,m,Color.red);
					Debug.Log (n);
				}*/
				vo.limit = dist - doubleRad;//m.magnitude;
				
				if (vo.limit < 0) {
					vo.origin += vo.direction * (vo.limit);
					vo.limit = 0.000F;
				}
				
				float ax = Mathf.Atan2 (vo.direction.z,vo.direction.x);
				vo.pRight = new Vector3 (Mathf.Cos (ax+vo.angle),0,Mathf.Sin (ax+vo.angle));
				vo.pLeft = new Vector3 (Mathf.Cos (ax-vo.angle),0,Mathf.Sin (ax-vo.angle));
				
				vo.nLeft = new Vector3 (Mathf.Cos (ax+vo.angle-Mathf.PI*0.5F),0,Mathf.Sin (ax+vo.angle-Mathf.PI*0.5F));
				vo.nRight = new Vector3 (Mathf.Cos (ax-vo.angle+Mathf.PI*0.5F),0,Mathf.Sin (ax-vo.angle+Mathf.PI*0.5F));
				
				//Debug.DrawRay (vo.origin,vo.nLeft,Color.red);
				//Debug.DrawRay (vo.origin,vo.nRight,Color.magenta);
				
				//vo.limit = Mathf.Min (rCenter.magnitude - doubleRad, rCenter.magnitude/2F);
				//doubleRad /= delta;
				//doubleRad /= 2;
				
				//vo.limit = doubleRad/Mathf.Tan (vo.angle);
				
				vos.Add (vo);
			}
			
			
			if (resType == ResolutionType.Geometric) {
				
				//Is there any need to run more calculations
				for (int i=0;i<vos.Count;i++) {
					if (vos[i].Contains (bestPoint)) {
						bestPointDist = float.PositiveInfinity;
						if (drawGizmos) Debug.DrawRay (bestPoint,Vector3.down,Color.red);
						bestPoint = transform.position;
						break;
					}
				}
				
				if (drawGizmos) {
					for (int i=0;i<vos.Count;i++) {
						vos[i].Draw (Color.black);
						
						/*if (vos[i].Contains	(testPoint)) {
							Debug.DrawLine (vos[i].origin,testPoint,Color.green);
						} else {
							Debug.DrawLine (vos[i].origin,testPoint,Color.red);
						}*/
						
					}
				}
				
				if (bestPointDist == 0) {
					return targetDir;
				}
				
				List<VOLine> lines = new List<VOLine> ();
				//List<VOIntersection> ints = new List<VOIntersection>();
				
				for (int i=0;i<vos.Count;i++) {
					VO vo = vos[i];
					
					
					float a = (float)Math.Atan2 (vo.direction.z,vo.direction.x);
				
					//Vector3 cross = Vector3.Cross (vo.direction,Vector3.up);
					//Vector3 p1 = cross*(float)Math.Tan (vo.angle)*vo.limit;
				
					Vector3 pLeft = vo.origin+new Vector3 ((float)Math.Cos (a+vo.angle),0,(float)Math.Sin (a+vo.angle))*vo.limit;
					Vector3 pRight = vo.origin+new Vector3 ((float)Math.Cos (a-vo.angle),0,(float)Math.Sin (a-vo.angle))*vo.limit;
					
					//Vector3 pLeft = vo.origin+vo.direction*vo.limit+p1;
					//Vector3 pRight = vo.origin+vo.direction*vo.limit-p1;
				
					Vector3 pLeft2 = pLeft+new Vector3 ((float)Math.Cos (a+vo.angle),0,(float)Math.Sin (a+vo.angle))*100;
					Vector3 pRight2 = pRight+new Vector3 ((float)Math.Cos (a-vo.angle),0,(float)Math.Sin (a-vo.angle))*100;
					
					int IgnoreDirection = Polygon.Left (vo.origin,vo.origin+vo.direction,transform.position+velocity) ? 1 : 2;//Vector3.Dot (transform.position+velocity-vo.origin,cross) > 0 ? 2 : 1;//(pRight-(transform.position+velocity)).sqrMagnitude < (pLeft-(transform.position+velocity)).sqrMagnitude ? 2 : 1;//Vector3.Dot (pRight-pLeft,transform.position+velocity-pLeft) > 0 ? 1 : 2;
					
					lines.Add (new VOLine (vo,pLeft,pLeft2,true,1, IgnoreDirection == 1));
					lines.Add (new VOLine (vo,pRight,pRight2,true,2, IgnoreDirection == 2));
					lines.Add (new VOLine (vo,pLeft,pRight,false,3, false));
					
					bool pLeftInside = false;// || IgnoreDirection == 1;
					bool pRightInside = false;// || IgnoreDirection == 2;
					
					if (!pLeftInside) {
						for (int q = 0;q<vos.Count;q++) {
							if (q != i) {
								if (vos[q].Contains (pLeft)) {
									pLeftInside = true;
									break;
								}
							}
						}
					}
					if (!pRightInside) {
						for (int q = 0;q<vos.Count;q++) {
							if (q != i) {
								if (vos[q].Contains (pRight)) {
									pRightInside = true;
									break;
								}
							}
						}
					}
					
					vo.AddInt (0,pLeftInside,1);//pLeftInside	|| IgnoreDirection == 1,1);
					vo.AddInt (0,pRightInside,2);//pRightInside 	|| IgnoreDirection == 2,2);
					vo.AddInt (0,pLeftInside,3);//pLeftInside	,3);//@ Ignore shouldn't really be there, but it works better with it of some reason
					vo.AddInt (1,pRightInside,3);//pRightInside	,3);//@ Ignore shouldn't really be there, but it works better with it of some reason
				}
				
				
				for (int i=0;i<lines.Count;i++) {
					for (int j=i+1;j<lines.Count;j++) {
						//if (i==j) continue;
						
						VOLine line = lines[i];
						VOLine line2 = lines[j];
						
						if (line.vo == line2.vo) continue;
							
						float factor1;
						float factor2;
						if (Polygon.IntersectionFactor (line.start,line.end,line2.start,line2.end,out factor1, out factor2)) {
							
							if (factor1 < 0 || factor2 < 0 || (!line.inf && factor1 > 1) || (!line2.inf && factor2 > 1)) {
								continue;
							}
							
							Vector3 p = line.start+(line.end-line.start)*factor1;
							bool inside = line.wrongSide || line2.wrongSide;
							if (!inside) {
								for (int q = 0;q<vos.Count;q++) {
									if (vos[q] != line.vo && vos[q] != line2.vo) {
										if (vos[q].Contains (p)) {
											inside = true;
											break;
										}
									}
								}
							}
						
							line.vo.AddInt (factor1,inside,line.id);
							line2.vo.AddInt (factor2,inside,line2.id);
							
							//ints.Add (new VOIntersection (line.vo,line2.vo, factor1, factor2,inside));
							if (drawGizmos) Debug.DrawRay (line.start+(line.end-line.start)*factor1,Vector3.up,inside ? Color.magenta : Color.green);
						}
					}
				}
				
				for (int i=0;i<vos.Count;i++) {
					Vector3 segmClosest;
					if (vos[i].FinalInts (targetPoint, transform.position+velocity, drawGizmos, out segmClosest)) {
						float dist = (segmClosest-targetPoint).sqrMagnitude;
						if (dist < bestPointDist) {
							bestPoint = segmClosest;
							bestPointDist = dist;
							if (drawGizmos) Debug.DrawLine (targetPoint+Vector3.up,bestPoint+Vector3.up,Color.red);
						}
					}
				}
			//}
			
				
				if (drawGizmos) Debug.DrawLine (targetPoint+Vector3.up,bestPoint+Vector3.up,Color.red);
				
				return Vector3.ClampMagnitude(bestPoint - transform.position,targetDir.magnitude*maxSpeedScale);
				
			} else if (resType == ResolutionType.Sampled) {
				/*Sampling Solution
				 * ================= */
				Vector3 forward = targetDir;
				Vector3 normForward = forward.normalized;
				Vector3 right = Vector3.Cross (normForward,Vector3.up);
				
				int circleSamples = 10;
				for (int i=0;i<10;i++, circleSamples+=2) {
					
					float radPerSamples = (float)(Math.PI*circlePoint/circleSamples);
					float offset = (float)(Math.PI-circlePoint*Math.PI)*0.5F;
					
					for (int j=0;j<circleSamples;j++) {
						float a = radPerSamples * j;
						Vector3 sample = transform.position + targetDir - (forward*(float)Math.Sin (a+offset)*i*circleScale + right * (float)Math.Cos (a+offset)*i*circleScale);
						
						if (CheckSample (sample,vos)) {
							return sample-transform.position;
						}
					}
				}
				
				for (int i=0;i<samples.Length;i++) {
					Vector3 sample = transform.position+samples[i].x*right + samples[i].z*normForward + samples[i].y*forward;
					
					if (CheckSample (sample,vos)) {
						return sample-transform.position;
					}
				}
				
				return Vector3.zero;
				 /* End of Sampling Solution
				 * ========================
				 */
			}
			
			
			return Vector3.zero;
			
			//if (drawGizmos) {
				
				
			/*HalfPlane[] hps = new HalfPlane[vos.Count];
			for (int i=0;i<vos.Count;i++) {
				HalfPlane hp = (HalfPlane)vos[i];
				hps[i] = hp;
			}
			
			/*HalfPlane hp_top = new HalfPlane ();
			hp_top.point = transform.position+Vector3.forward*speed;
			hp_top.normal = -new Vector3 (0.001F,0,1F).normalized;
			hps[vos.Count+0] = hp_top;
			
			HalfPlane hp_bot = new HalfPlane ();
			hp_bot.point = transform.position-Vector3.forward*speed;
			hp_bot.normal = new Vector3 (0.001F,0,1F).normalized;
			hps[vos.Count+1] = hp_bot;
			
			HalfPlane hp_left = new HalfPlane ();
			hp_left.point = transform.position-Vector3.right*speed;
			hp_left.normal = new Vector3 (1F,0,0.001F).normalized;
			hps[vos.Count+2] = hp_left;
			
			HalfPlane hp_right = new HalfPlane ();
			hp_right.point = transform.position+Vector3.right*speed;
			hp_right.normal = -new Vector3 (1F,0,0.001F).normalized;
			hps[vos.Count+3] = hp_right;*
			
			for (int i=0;i<hps.Length;i++) {
				if (drawGizmos) { hps[i].Draw (); }
			}
			
			for (int i=0;i<hps.Length;i++) {
				HalfPlane hp = hps[i];
				if (hp.Contains	(bestPoint)) {
					if (drawGizmos) {
						Debug.DrawLine (bestPoint,hp.point,Color.green);
					}
					continue;
				}
				
				
				Vector3 tangent = Vector3.Cross (hp.normal,Vector3.up);
				
				float leftBound = -100;
				float rightBound = 100;
				
				if (tangent.x < 0) {
					tangent = -tangent;
				}
				
				for (int j=0;j<i;j++) {
					HalfPlane hp2 = hps[j];
					Vector3 intersection = hp.Intersection (hp2);
					
					float dot = Vector3.Dot (tangent,hp2.normal);
					if (dot > 0) {
						leftBound = Mathf.Max (leftBound,intersection.x);
					} else if (dot < 0) {
						rightBound = Mathf.Min (rightBound,intersection.x);
					} else {
						//Parallel
						if (Vector3.Dot (hp.normal,hp2.normal) < 0) {
							if (drawGizmos) {
								Debug.DrawLine (bestPoint,hp.point,Color.red);
							}
							return Vector3.zero;
						}
					}
					/*if (hp.normal.x > hp2.normal.x) {
						if (hp2.normal.x < 0) {
							rightBound = Mathf.Min (intersection.x, rightBound);
						} else {
							leftBound = Mathf.Max (leftBound,intersection.x);
						}
					} else if (hp.normal.x < hp2.normal.x) {
						if (hp2.normal.x < 0) {
							leftBound = Mathf.Max (leftBound,intersection.x);
						} else {
							rightBound = Mathf.Min (rightBound,intersection.x);
						}
					}*
				}
				
				if (drawGizmos) {
					hp.DrawBounds (leftBound,rightBound);
				}
				if (leftBound > rightBound) {
					if (drawGizmos) {
						Debug.DrawRay (bestPoint,Vector3.up,Color.red);
						Debug.DrawLine (bestPoint,hp.point,Color.red);
					}
					return Vector3.zero;
				}
				
				Vector3 closest = hp.ClosestPoint (targetPoint,leftBound,rightBound);
				if (drawGizmos) {
					Debug.DrawLine (bestPoint,closest,Color.red);
				}
				bestPoint = closest;
			}
			
			return bestPoint-transform.position;*/
		}
		
		public struct VOLine {
			public VO vo;
			public Vector3 start, end;
			public bool inf;
			public int id;
			public bool wrongSide;
			
			public VOLine (VO vo, Vector3 start, Vector3 end, bool inf, int id, bool wrongSide) {
				this.vo = vo;
				this.start = start;
				this.end = end;
				this.inf = inf;
				this.id = id;
				this.wrongSide = wrongSide;
			}
		}
		
		public struct VOIntersection {
			public VO vo1,vo2;
			public float factor1,factor2;
			public bool inside;
			public VOIntersection (VO vo1, VO vo2, float factor1, float factor2, bool inside = false) {
				this.vo1 = vo1;
				this.vo2 = vo2;
				this.factor1 = factor1;
				this.factor2 = factor2;
				this.inside = inside;
			}
		}
		
		public bool CheckSample (Vector3 sample, List<VO> vos) {
			bool anyFailed = false;
			for (int q=0;q<vos.Count;q++) {
				if (vos[q].Contains	(sample)) {
					if (drawGizmos) {
						Debug.DrawRay (sample,Vector3.up,Color.red);
					}
					anyFailed = true;
					break;
				}
			}
			if (drawGizmos && !anyFailed) {
				Debug.DrawRay (sample,Vector3.up,Color.yellow);
			}
			return !anyFailed;
		}
		
		public class HalfPlane {
			public Vector3 point;
			public Vector3 normal;
			
			public bool Contains (Vector3 p) {
				p -= point;
				return Vector3.Dot (normal,p) >= 0;
			}
			
			public Vector3 ClosestPoint (Vector3 p) {
				p -= point;
				Vector3 tangent = Vector3.Cross (normal,Vector3.up);
				float dot = Vector3.Dot (tangent,p);
				return point + tangent*dot;
			}
			
			public Vector3 ClosestPoint (Vector3 p, float minX, float maxX) {
				p -= point;
				Vector3 tangent = Vector3.Cross (normal,Vector3.up);
				if (tangent.x < 0) {
					tangent = -tangent;
				}
				float dot = Vector3.Dot (tangent,p);
				float min = (minX - point.x) / tangent.x;
				float max = (maxX - point.x) / tangent.x;
				dot = Mathf.Clamp (dot,min,max);
				return point + tangent*dot;
			}
			
			public Vector3 Intersection (HalfPlane hp) {
				Vector3 tangent = Vector3.Cross (normal,Vector3.up);
				Vector3 tangent2 = Vector3.Cross (hp.normal,Vector3.up);
				
				return Polygon.IntersectionPointOptimized (point,tangent,hp.point,tangent2);
			}
			
			public void DrawBounds (float left, float right) {
				Vector3 tangent = Vector3.Cross (normal,Vector3.up);
				if (tangent.x < 0) {
					tangent = -tangent;
				}
				float factor1 = (left - point.x) / tangent.x;
				float factor2 = (right - point.x) / tangent.x;
				Debug.DrawLine (point + tangent*factor1+Vector3.up*0.1F, point + tangent*factor2+Vector3.up*0.1F,Color.yellow);
			}
			
			public void Draw () {
				Vector3 tangent = Vector3.Cross (normal,Vector3.up);
				Debug.DrawLine (point-tangent*10,point+tangent*10,Color.blue);
				Debug.DrawRay (point,normal,new Color (0.8F,0.1F,0.2F));
			}
		}
		
		public enum IntersectionState {
			Inside,
			Outside,
			Enter,
			Exit
		}
		
		public struct IntersectionPair : IComparable<IntersectionPair> {
			public float factor;
			public IntersectionState state;
			
			public IntersectionPair (float factor, bool inside) {
				this.factor = factor;
				state = inside ? IntersectionState.Inside : IntersectionState.Outside;
			}
			
			public void SetState (IntersectionState s) {
				state = s;
			}
			
			public int CompareTo (IntersectionPair o) {
				//if (o == null || !(o is IntersectionPair)) {
				//	return 0;
				//}
				
				//IntersectionPair ob = (IntersectionPair)o;
				if (o.factor < factor) {
					return 1;
				} else if (o.factor > factor) {
					return -1;
				}
				return 0;
			}
		}
		
		public class VO {
			public Vector3 origin; /**< Origin of the VO cone */
			public Vector3 direction; /**< Direction of the VO, should be normalized */
			public float angle; /**< Angle as Cos (a) */
			public float limit; /**< Distance from origin to start of cone */
			
			public Vector3 pLeft;
			public Vector3 pRight;
			
			public Vector3 nLeft, nRight; //Normals determining the inside of the cone
			
			public List<IntersectionPair> ints1 = new List<IntersectionPair>();
			public List<IntersectionPair> ints2 = new List<IntersectionPair>();
			public List<IntersectionPair> ints3 = new List<IntersectionPair>();
			
			public void AddInt (float factor, bool inside, int id) {
				switch (id) {
				case 1:
					ints1.Add (new IntersectionPair(factor,inside));
					break;
				case 2:
					ints2.Add (new IntersectionPair(factor,inside));
					break;
				case 3:
					ints3.Add (new IntersectionPair(factor,inside));
					break;
				}
			}
			
			public bool FinalInts (Vector3 target, Vector3 closeEdgeConstraint, bool drawGizmos, out Vector3 closest) {
				ints1.Sort ();
				ints2.Sort ();
				ints3.Sort ();
				
				float a = (float)Math.Atan2 (direction.z,direction.x);
				
				Vector3 cross = Vector3.Cross ( direction,Vector3.up);
				Vector3 p1 = cross*(float)Math.Tan ( angle)* limit;
			
				Vector3 pLeft =  origin+ direction* limit+p1;
				Vector3 pRight =  origin+ direction* limit-p1;
			
				Vector3 pLeft2 = pLeft+new Vector3 ((float)Math.Cos (a+ angle),0,(float)Math.Sin (a+ angle))*100;
				Vector3 pRight2 = pRight+new Vector3 ((float)Math.Cos (a- angle),0,(float)Math.Sin (a- angle))*100;
				
				bool anyCloseFound = false;
				closest = Vector3.zero;
				
				int IgnoreDirection =  Vector3.Dot (closeEdgeConstraint-origin,cross) > 0 ? 2 : 1;//(pRight-(transform.position+velocity)).sqrMagnitude < (pLeft-(transform.position+velocity)).sqrMagnitude ? 2 : 1;//Vector3.Dot (pRight-pLeft,target-pLeft) > 0 ? 1 : 2;
				
				for (int j=1;j<=3;j++) {
					
					if (j == IgnoreDirection) {
						continue;
					}
					
					List<IntersectionPair> ints = j == 1 ? ints1 : (j == 2 ? ints2 : ints3);
					
					/*IntersectionState prevInside = ints[0].state;
					if (ints[0].state == IntersectionState.Outside) {
						ints[0].SetState (IntersectionState.Exit);
					}*/
					
					/*for (int i=1;i<ints.Count;i++) {
						Debug.Log ("Intersection at "+j+", "+i+" : "+ints[i].factor);
						prevInside = ints[i].state;
						if (prevInside == IntersectionState.Outside) {
							ints[i].SetState (IntersectionState.Enter);
						} else if (prevInside == IntersectionState.Inside) {
							ints[i].SetState (ints[i].state == IntersectionState.Inside ? IntersectionState.Inside : IntersectionState.Exit);
						}
					}
					if (ints[ints.Count-1].state == IntersectionState.Exit) {
						ints.Add (new IntersectionPair (1,false));
					}*/
					
					Vector3 start = (j == 1 || j == 3 ? pLeft : pRight);
					Vector3 end = (j == 1 ? pLeft2 : (j == 2 ? pRight2 : pRight));
					
					float closestFactor = AstarMath.NearestPointFactor (start,end, target);
					
					float closeMin = float.PositiveInfinity;
					float closeMax = float.NegativeInfinity;
					
					bool anySegmClose = false;
					for (int i=0;i<ints.Count - (j == 3 ? 1 : 0);i++) {
						
						if (drawGizmos) Debug.DrawRay (start+(end-start)*ints[i].factor,Vector3.down,ints[i].state == IntersectionState.Outside ? Color.green : Color.red);
						if (ints[i].state == IntersectionState.Outside && ((i == ints.Count-1 && (i == 0 || ints[i-1].state != IntersectionState.Outside)) || (i < ints.Count-1 && ints[i+1].state == IntersectionState.Outside))) {
							
							anySegmClose = true;
							float startFactor = ints[i].factor;
							float endFactor = i == ints.Count-1 ? (j == 3 ? 1 : float.PositiveInfinity) : ints[i+1].factor;
							
							
							if (drawGizmos) Debug.DrawLine (start+(end-start)*startFactor + Vector3.up, start+(end-start)*Mathf.Clamp01(endFactor) + Vector3.up, Color.green);
							
							if (startFactor <= closestFactor && endFactor >= closestFactor) {
								closeMin = closestFactor;
								closeMax = closestFactor;
								break;
							} else if (endFactor < closestFactor && endFactor > closeMax) {
								closeMax = endFactor;
								
							} else if (startFactor > closestFactor && startFactor < closeMin) {
								closeMin = startFactor;
							}
						}
					}
					
					if (anySegmClose) {
						
						//The closest factor
						float closeV = closeMin == float.NegativeInfinity ? closeMax : (closeMax == float.PositiveInfinity ? closeMin : (Mathf.Abs(closestFactor-closeMin) < Mathf.Abs(closestFactor-closeMax) ? closeMin : closeMax));
						
						/*if (Mathf.Abs(closestFactor-closeMin) < Mathf.Abs(closestFactor-closeMax)) {
							segmClose = start+ (end-start)*closeMin;
						} else {
							segmClose = start+ (end-start)*closeMax;
						}*/
						Vector3 segmClose = start+ (end-start)*closeV;
						
						if (!anyCloseFound || (segmClose-target).sqrMagnitude < (closest-target).sqrMagnitude) {
							closest = segmClose;
						}
						if (drawGizmos) Debug.DrawLine (target,closest,Color.yellow);
						
						anyCloseFound = true;
					}
				}
				
				return anyCloseFound;
			}
			
			
			/** Returns if this VO contains the point \a p */
			public bool Contains (Vector3 p) {
				
				//nRight, nLeft
				
				return Vector3.Dot (nLeft,p-origin) > 0.0F  && Vector3.Dot (nRight,p-origin) > 0.0F&& Vector3.Dot (direction,p-origin) > limit;
				
				/*float acos = (float)Math.Cos (angle);
				p -= origin;
				p.y = 0;
				float dot = Vector3.Dot (direction,p.normalized);
				if (dot < acos) {
					return false;
				}
				dot *= p.magnitude;*/
				//return dot > limit;
			}
			
			/** Returns if this VO contains the point \a p */
			public float ScoreContains (Vector3 p) {
				/*float acos = (float)Math.Cos (angle);
				p -= origin;
				p.y = 0;
				float dot = Vector3.Dot (direction,p.normalized);
				if (dot < acos) {
					return 0;
				}
				float dot1 = dot;
				dot *= p.magnitude;
				if (dot > limit) {
					return Mathf.Min ((dot - limit)*50,(dot-acos)*100);
				} else {
					return 0F;
				}*/
				return 0F;
				//return dot > limit;
			}
			
			public void Draw (Color c) {
				
				float a = (float)Math.Atan2 (direction.z,direction.x);
				
				Vector3 p1 = Vector3.Cross (direction,Vector3.up)*(float)Math.Tan (angle)*limit;
				
				Debug.DrawLine (origin+direction*limit+p1,origin+direction*limit-p1,c);
				
				Debug.DrawRay (origin+direction*limit+p1,new Vector3 ((float)Math.Cos (a+angle),0,(float)Math.Sin (a+angle))*10,c);
				Debug.DrawRay (origin+direction*limit-p1,new Vector3 ((float)Math.Cos (a-angle),0,(float)Math.Sin (a-angle))*10,c);
				
			}
				
			public static explicit operator HalfPlane (VO vo) {
				HalfPlane hp = new HalfPlane ();
				hp.point = vo.origin + vo.direction*vo.limit;
				hp.normal = -vo.direction;
				//Vector3 p1 = Vector3.Cross (vo.direction,Vector3.up)*(float)Math.Tan (angle)*limit;
				return hp;
			}
		}
	}
}