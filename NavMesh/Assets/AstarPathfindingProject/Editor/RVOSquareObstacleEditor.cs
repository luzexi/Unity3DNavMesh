using UnityEngine;
using System.Collections;
using UnityEditor;
using Pathfinding.RVO;

[CustomEditor(typeof(RVOSquareObstacle))]
public class RVOSquareObstacleEditor : Editor
{
	public override void OnInspectorGUI ()
	{
		DrawDefaultInspector ();
		EditorGUILayout.HelpBox ("Due to recent changes to the local avoidance system. RVO obstacles are currently not functional." +
			" They will be enabled in a comming update. In the meantime you can use agents with the 'locked' field set to true as simple obstacles.", MessageType.Warning );
	}
}