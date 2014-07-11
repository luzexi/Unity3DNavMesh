using UnityEngine;
using UnityEditor;
using System.Collections;
using Pathfinding;

namespace Pathfinding {
	[CustomGraphEditor (typeof(LayerGridGraph),"Layered Grid Graph")]
	public class LayerGridGraphEditor : GridGraphEditor {
	
		public override void OnInspectorGUI (NavGraph target) {
			
			LayerGridGraph graph = target as LayerGridGraph;
			
			graph.mergeSpanRange = EditorGUILayout.FloatField ("Merge Span Range",graph.mergeSpanRange);
			graph.characterHeight = EditorGUILayout.FloatField ("Character Height",graph.characterHeight);
			graph.maxClimb = Mathf.Clamp (EditorGUILayout.FloatField ("Max climb",graph.maxClimb),0,graph.characterHeight);
			
			graph.neighbours = NumNeighbours.Four;
			textureVisible = false;
			base.OnInspectorGUI (target);
			
			if (graph.neighbours != NumNeighbours.Four) {
				Debug.Log ("Note: Only 4 neighbours per grid node is allowed in this graph type");
			}
			
			if (graph.collision.thickRaycast)
				HelpBox ("Note: Thick raycast cannot be used with this graph type");
		}
	}
}