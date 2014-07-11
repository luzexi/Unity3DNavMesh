using UnityEngine;
using System.Collections;
using UnityEditor;
using Pathfinding.RVO;

[CustomEditor(typeof(Pathfinding.RVO.RVOSimulator))]
public class RVOSimulatorEditor : UnityEditor.Editor {

	public override void OnInspectorGUI () {
		DrawDefaultInspector();
		/*EditorGUILayout.HelpBox ("RVO Local Avoidance is temporarily disabled in the A* Pathfinding Project due to licensing issues.\n" +
			"I am working to get it back as soon as possible. All agents will fall back to not avoiding other agents.\n" +
			"Sorry for the inconvenience.", MessageType.Warning ); */
	}
}
