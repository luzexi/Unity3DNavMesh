using System;
using UnityEngine;
using Pathfinding;

namespace Pathfinding {
	/** Returns a path heading away from a specified point to avoid.
	 * The search will terminate when G \>= \a length (passed to the constructor) + FleePath.spread.\n
	 * \ingroup paths
	 * Can be used to make an AI to flee from an enemy (cannot guarantee that it will not be forced into corners though :D )\n
	 * \code

//Call a FleePath call like this, assumes that a Seeker is attached to the GameObject
Vector3 thePointToFleeFrom = Vector3.zero;

//The path will be returned when the path is over a specified length (or more accurately has "costed" more than a specific value)
//This is usally roughly the distance from the start to the end multiplied by 100
int theGScoreToStopAt = 1000;

//Create a path object
FleePath path = FleePath.Construct (transform.position, thePointToFleeFrom, theGScoreToStopAt);

//Get the Seeker component which must be attached to this GameObject
Seeker seeker = GetComponent<Seeker>();

//Start the path and return the result to MyCompleteFunction (which is a function you have to define, the name can of course be changed)
seeker.StartPath (path,MyCompleteFunction);

	 * \endcode
	 * \astarpro */
	public class FleePath : RandomPath {
		
		[System.Obsolete("Please use the Construct method instead")]
		public FleePath (Vector3 start, Vector3 avoid, int length, OnPathDelegate callbackDelegate = null) : base (start,length,callbackDelegate) {
			throw new System.Exception ("Please use the Construct method instead");
		}
		
		public FleePath () {}
		
		/** Constructs a new FleePath.
		 * The FleePath will be taken from a pool.
		 */
		public static FleePath Construct (Vector3 start, Vector3 avoid, int searchLength, OnPathDelegate callback = null) {
			FleePath p = PathPool<FleePath>.GetPath ();
			p.Setup (start,avoid, searchLength, callback);
			return p;
		}
		
		protected void Setup (Vector3 start, Vector3 avoid, int searchLength, OnPathDelegate callback) {
			Setup (start, searchLength, callback);
			aim = avoid-start;
			aim *= 10;
			aim = start - aim;
		}
		
		protected override void Recycle () {
			PathPool<FleePath>.Recycle (this);
		}
	}
}