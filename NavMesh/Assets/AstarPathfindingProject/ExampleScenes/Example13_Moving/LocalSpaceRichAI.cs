using UnityEngine;
using System.Collections;
using Pathfinding;

public class LocalSpaceRichAI : RichAI {

	public LocalSpaceGraph graph;

	public override void UpdatePath ()
	{
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

		Matrix4x4 m = graph.GetMatrix ();

		seeker.StartPath (m.MultiplyPoint3x4 (tr.position), m.MultiplyPoint3x4 (target.position));

	}

	protected override Vector3 UpdateTarget (RichFunnel fn)
	{

		Matrix4x4 m = graph.GetMatrix ();
		Matrix4x4 mi = m.inverse;


		Debug.DrawRay (m.MultiplyPoint3x4 (tr.position), Vector3.up*2, Color.red);
		Debug.DrawRay (mi.MultiplyPoint3x4 (tr.position), Vector3.up*2, Color.green);

		buffer.Clear ();
		/* Current position. We read and write to tr.position as few times as possible since doing so
				 * is much slower than to read and write from/to a local variable
				 */
		Vector3 position = tr.position;
		bool requiresRepath;

		// Update, but first convert our position to graph space, then convert the result back to world space
		position = mi.MultiplyPoint3x4 ( fn.Update ( m.MultiplyPoint3x4(position), buffer, 2, out lastCorner, out requiresRepath) );

		Debug.DrawRay ( position, Vector3.up*3, Color.black );

		// convert the result to world space from graph space
		for ( int i=0; i < buffer.Count; i++ ) {
			buffer[i] = mi.MultiplyPoint3x4 ( buffer[i] );
			Debug.DrawRay ( buffer[i], Vector3.up*3, Color.yellow );
		}

		if (requiresRepath && !waitingForPathCalc) {
			UpdatePath ();
		}
		
		return position;
	}
}
