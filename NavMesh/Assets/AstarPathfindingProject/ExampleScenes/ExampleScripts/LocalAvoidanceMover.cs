using UnityEngine;
using System.Collections;
using Pathfinding;

namespace Pathfinding {
	/** Small example script of using the LocalAvoidance class.
	  * \deprecated This class should not be used anymore, use the RVO system instead.
	  */
	[RequireComponent(typeof(LocalAvoidance))]
	[System.Obsolete("Use the RVO system instead")]
	public class LocalAvoidanceMover : MonoBehaviour {
		
		public float targetPointDist = 10;
		public float speed = 2;
		
		Vector3 targetPoint;
		LocalAvoidance controller;
			
		// Use this for initialization
		void Start () {
			targetPoint = transform.forward*targetPointDist + transform.position;
			controller = GetComponent<LocalAvoidance>();
			
		}
		
		// Update is called once per frame
		void Update () {
			if (controller != null) {
				//The LocalAvoidance controller can be called just like a CharacterController
				controller.SimpleMove ((targetPoint-transform.position).normalized*speed);
			}
			/* else {
				GetComponent<CharacterController>().SimpleMove ((targetPoint-transform.position).normalized*speed);
			}*/
		}
	}
}