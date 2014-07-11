using UnityEngine;
using System.Collections;

/** Places ROV agents in circles.
  * Used in a example scene
  */
public class RVOAgentPlacer : MonoBehaviour {
	
	public int agents = 100;
	
	public float ringSize = 100;
	public LayerMask mask;
	
	public GameObject prefab;
	
	public Vector3 goalOffset;
	
	public float repathRate = 1;
	
	// Use this for initialization
	IEnumerator Start () {
		yield return null;
		
		for (int i=0;i<agents;i++) {
			float angle = ((float)i / agents)*(float)System.Math.PI*2;
			
			Vector3 pos = new Vector3 ((float)System.Math.Cos(angle),0,(float)System.Math.Sin(angle))*ringSize;
			Vector3 antipodal = -pos + goalOffset;
			
			GameObject go = GameObject.Instantiate (prefab,Vector3.zero,Quaternion.Euler(0,angle+180,0)) as GameObject;
			RVOExampleAgent ag = go.GetComponent<RVOExampleAgent>();
			
			if (ag == null) {
				Debug.LogError ("Prefab does not have an RVOExampleAgent component attached");
				yield break;
			}
			
			//ag.radius = radius;
			go.transform.parent = transform;
			go.transform.position = pos;
			
			ag.repathRate = repathRate;
			ag.SetTarget (antipodal);
			
			ag.SetColor (GetColor (angle));
		}
		
	}
	
	const float rad2Deg = 360.0f/ ((float)System.Math.PI*2);
	
	public Color GetColor (float angle) {
		return HSVToRGB (angle * rad2Deg, 0.8f, 0.6f);
	}
	
	/**
	 * Converts an HSV color to an RGB color, according to the algorithm described at http://en.wikipedia.org/wiki/HSL_and_HSV
	 *
	 * @return the RGB representation of the color.
	 */
	static Color HSVToRGB(float h, float s, float v)
	{
		float Min;
		float Chroma;
		float Hdash;
		float X;
		float r = 0,g = 0,b = 0;
	 
		Chroma = s * v;
		Hdash = h / 60.0f;
		X = Chroma * (1.0f - System.Math.Abs((Hdash % 2.0f) - 1.0f));
	 
		if(Hdash < 1.0f)
		{
			r = Chroma;
			g = X;
		}
		else if(Hdash < 2.0f)
		{
			r = X;
			g = Chroma;
		}
		else if(Hdash < 3.0f)
		{
			g = Chroma;
			b = X;
		}
		else if(Hdash < 4.0f)
		{
			g= X;
			b = Chroma;
		}
		else if(Hdash < 5.0f)
		{
			r = X;
			b = Chroma;
		}
		else if(Hdash < 6.0f)
		{
			r = Chroma;
			b = X;
		}
	 
		Min = v - Chroma;
	 
		r += Min;
		g += Min;
		b += Min;
	 
		return new Color (r,g,b);
	}
}
