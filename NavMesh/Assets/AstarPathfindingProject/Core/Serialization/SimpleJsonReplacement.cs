//#define ASTAR_NO_JSON

#if ASTAR_NO_JSON
using UnityEngine;
using System.Collections;

namespace Pathfinding.Serialization.JsonFx {

	public class JsonMemberAttribute : System.Attribute {}
	public class JsonOptInAttribute : System.Attribute {}
	public class JsonNameAttribute : System.Attribute { public JsonNameAttribute (string s) {} }
}
#endif