using UnityEngine;
using System.Collections;
using Pathfinding;

public class BezierMover : MonoBehaviour {

	public Transform[] points;

	public float tangentLengths = 5;
	public float speed = 1;

	float time = 0;

	void Update (  ) {
		Move ( true );
	}

	Vector3 Plot (float t) {
		Vector3 inTang, outTang;
		
		
		int c = points.Length;
		int pt = Mathf.FloorToInt(t);
		inTang = (  (points[(pt+1)%c].position - points[(pt+0)%c].position).normalized - (points[(pt-1+c)%c].position - points[(pt+0)%c].position).normalized ).normalized;
		
		outTang = (  (points[(pt+2)%c].position - points[(pt+1)%c].position).normalized - (points[(pt-0+c)%c].position - points[(pt+1)%c].position).normalized ).normalized;
		
		Debug.DrawLine ( points[pt%c].position, points[pt%c].position + inTang*tangentLengths, Color.red);
		Debug.DrawLine ( points[(pt+1)%c].position - outTang*tangentLengths, points[(pt+1)%c].position, Color.green);

		return AstarMath.CubicBezier ( points[pt%c].position, points[pt%c].position + inTang*tangentLengths, points[(pt+1)%c].position - outTang*tangentLengths, points[(pt+1)%c].position, t - pt);
	}

	// Update is called once per frame
	void Move ( bool progress ) {

		/*if ( time > pt+1 ) {
			Move ( false );
			return;
		}*/

		float mn = time;
		float mx = time+1;
		while ( mx - mn > 0.0001f ) {
			float mid = (mn+mx)/2;

			Vector3 p = Plot ( mid );
			if ( (p-transform.position).sqrMagnitude > (speed*Time.deltaTime)*(speed*Time.deltaTime) ) {
				mx = mid;
			} else {
				mn = mid;
			}
		}

		time = (mn+mx)/2;


		/*Vector3 p1 = AstarMath.CubicBezier ( points[pt%c].position, points[pt%c].position + inTang*tangentLengths, points[(pt+1)%c].position - outTang*tangentLengths, points[(pt+1)%c].position, time - pt);
		Vector3 p2 = AstarMath.CubicBezier ( points[pt%c].position, points[pt%c].position + inTang*tangentLengths, points[(pt+1)%c].position - outTang*tangentLengths, points[(pt+1)%c].position, time - pt + 0.001f);*/
		Vector3 p1 = Plot(time);
		Vector3 p2 = Plot(time+0.001f);
		transform.position = p1;
		transform.rotation = Quaternion.LookRotation ( p2 - p1 );

	}

	public void OnDrawGizmos () {
		if ( points.Length >= 3 ) {

			for ( int i = 0; i < points.Length; i++ ) if ( points[i] == null ) return;

			for ( int pt = 0; pt < points.Length; pt++ ) {

				int c = points.Length;
				Vector3 inTang = (  (points[(pt+1)%c].position - points[pt+0].position).normalized - (points[(pt-1+c)%c].position - points[pt+0].position).normalized ).normalized;
				
				Vector3 outTang = (  (points[(pt+2)%c].position - points[(pt+1)%c].position).normalized - (points[(pt-0+c)%c].position - points[(pt+1)%c].position).normalized ).normalized;

				Vector3 pp = points[pt].position;
				
				for ( int i=1;i<=100;i++) {
					Vector3 p = AstarMath.CubicBezier ( points[pt].position, points[pt].position + inTang*tangentLengths, points[(pt+1)%c].position - outTang*tangentLengths, points[(pt+1)%c].position, i / 100.0f );
					Gizmos.DrawLine ( pp, p );
					pp = p;
				}
			}

		}
	}

}
