using UnityEngine;
using System.Collections;

public class LocalSpaceGraph : MonoBehaviour {

	protected Matrix4x4 originalMatrix;

	// Use this for initialization
	void Start () {
		originalMatrix = transform.localToWorldMatrix;
	}
	
	// Update is called once per frame
	public Matrix4x4 GetMatrix ( ) {
		return transform.worldToLocalMatrix * originalMatrix;
	}
}
