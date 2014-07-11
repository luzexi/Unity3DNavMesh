
//#define ASTARDEBUG

using UnityEngine;
using System.Collections;

namespace Pathfinding.Voxels {
	/** \astarpro */
	public class Utility {
		
		public static Color[] colors = new Color[7] {Color.green,Color.blue,Color.red,Color.yellow,Color.cyan, Color.white, Color.black};
		
		public static Color GetColor (int i) {
			while (i >= colors.Length) {
				i -= colors.Length;
			}
			
			while (i < 0) {
				i += colors.Length;
			}
			
			return colors[i];
		}
		
		public static int Bit (int a, int b) {
			return (a & (1 << b)) >> b;
		}
		
		public static Color IntToColor (int i, float a) {
			int	r = Bit(i, 1) + Bit(i, 3) * 2 + 1;
			int	g = Bit(i, 2) + Bit(i, 4) * 2 + 1;
			int	b = Bit(i, 0) + Bit(i, 5) * 2 + 1;
			return new Color (r*0.25F,g*0.25F,b*0.25F,a);
		}
	
		//For the real value, divide by 2
		public static float TriangleArea2 (Vector3 a, Vector3 b, Vector3 c) {
			return Mathf.Abs (a.x*b.z+b.x*c.z+c.x*a.z-a.x*c.z-c.x*b.z-b.x*a.z);
		}
		
		//public static float TriangleArea (Vector2 a, Vector2 b, Vector2 c) {
		//	return Mathf.Abs (a.x*b.y+b.x*c.y+c.x*a.y-a.x*c.y-c.x*b.y-b.x*a.y);
		//}
		
		public static float TriangleArea (Vector3 a, Vector3 b, Vector3 c) {
			return (b.x-a.x)*(c.z-a.z)-(c.x-a.x)*(b.z-a.z);
		}
		
		//Derived from the above function
		//public static float TriangleArea (Vector2 p, Vector3 a, Vector3 b, Vector3 c) {
			
		//}
		
		public static float Min (float a, float b, float c) {
			a = a < b ? a : b;
			return a < c ? a : c;
		}
		
		public static float Max (float a, float b, float c) {
			a = a > b ? a : b;
			return a > c ? a : c;
		}
		
		public static int Max (int a, int b, int c, int d) {
			a = a > b ? a : b;
			a = a > c ? a : c;
			return a > d ? a : d;
		}
		
		public static int Min (int a, int b, int c, int d) {
			a = a < b ? a : b;
			a = a < c ? a : c;
			return a < d ? a : d;
		}
		
		public static float Max (float a, float b, float c, float d) {
			a = a > b ? a : b;
			a = a > c ? a : c;
			return a > d ? a : d;
		}
		
		public static float Min (float a, float b, float c, float d) {
			a = a < b ? a : b;
			a = a < c ? a : c;
			return a < d ? a : d;
		}
		
		public static string ToMillis (float v) {
			return (v*1000).ToString ("0");
		}
		
		public static float lastStartTime;
		public static void StartTimer () {
			lastStartTime = Time.realtimeSinceStartup;
		}
		
		public static void EndTimer (string label) {
			Debug.Log (label+", process took "+ToMillis(Time.realtimeSinceStartup-lastStartTime)+"ms to complete");
		}	
		
		public static float lastAdditiveTimerStart;
		public static float additiveTimer;
		public static void StartTimerAdditive (bool reset) {
			if (reset) {
				additiveTimer = 0;
			}
			
			lastAdditiveTimerStart = Time.realtimeSinceStartup;
		}
		
		public static void EndTimerAdditive (string label, bool log) {
			additiveTimer += Time.realtimeSinceStartup-lastAdditiveTimerStart;
			
			if (log) {
				Debug.Log (label+", process took "+ToMillis(additiveTimer)+"ms to complete");
			}
				
			lastAdditiveTimerStart = Time.realtimeSinceStartup;
		}
		
		public static void CopyVector (float[] a, int i, Vector3 v) {
			a[i] = v.x;
			a[i+1] = v.y;
			a[i+2] = v.z;
		}
		
		private static float[] clipPolygonCache = new float[7*3];
		private static int[] clipPolygonIntCache = new int[7*3];
		
		public static int ClipPoly(float[] vIn, int n, float[] vOut, float pnx, float pnz, float pd)
		{
		        float[] d = clipPolygonCache;
		        
		        for (int i = 0; i < n; ++i)
		                d[i] = pnx*vIn[i*3+0] + pnz*vIn[i*3+2] + pd;
		        
		        int m = 0;
		        for (int i = 0, j = n-1; i < n; j=i, ++i)
		        {
		                bool ina = d[j] >= 0;
		                bool inb = d[i] >= 0;
		                if (ina != inb)
		                {
		                        float s = d[j] / (d[j] - d[i]);
		                        vOut[m*3+0] = vIn[j*3+0] + (vIn[i*3+0] - vIn[j*3+0])*s;
		                        vOut[m*3+1] = vIn[j*3+1] + (vIn[i*3+1] - vIn[j*3+1])*s;
		                        vOut[m*3+2] = vIn[j*3+2] + (vIn[i*3+2] - vIn[j*3+2])*s;
		                        m++;
		                }
		                if (inb)
		                {
		                        vOut[m*3+0] = vIn[i*3+0];
		                        vOut[m*3+1] = vIn[i*3+1];
		                        vOut[m*3+2] = vIn[i*3+2];
		                        m++;
		                }
		        }
		        return m;
		}
		
		public static int ClipPolygon (float[] vIn, int n, float[] vOut, float multi, float offset, int axis) {
			
			float[] d = clipPolygonCache;
			
			for (int i=0;i<n;i++) {
				d[i] = multi*vIn[i*3+axis]+offset;
			}
			
			//Number of resulting vertices
			int m = 0;
			
			for (int i=0, j = n-1; i < n; j=i, i++) {
				
				bool prev = d[j] >= 0;
				bool curr = d[i] >= 0;
				
				if (prev != curr) {
					int m3 = m*3;
					int i3 = i*3;
					int j3 = j*3;
					
					float s = d[j] / (d[j] - d[i]);
					
					vOut[m3+0] = vIn[j3+0] + (vIn[i3+0]-vIn[j3+0])*s;
					vOut[m3+1] = vIn[j3+1] + (vIn[i3+1]-vIn[j3+1])*s;
					vOut[m3+2] = vIn[j3+2] + (vIn[i3+2]-vIn[j3+2])*s;
					
					//vOut[m*3+0] = vIn[j*3+0] + (vIn[i*3+0]-vIn[j*3+0])*s;
					//vOut[m*3+1] = vIn[j*3+1] + (vIn[i*3+1]-vIn[j*3+1])*s;
					//vOut[m*3+2] = vIn[j*3+2] + (vIn[i*3+2]-vIn[j*3+2])*s;
					
					m++;
				}
				
				if (curr) {
					int m3 = m*3;
					int i3 = i*3;
					
					vOut[m3+0] = vIn[i3+0];
					vOut[m3+1] = vIn[i3+1];
					vOut[m3+2] = vIn[i3+2];
					
					m++;
				}
			}
			
			return m;
		}
		
		public static int ClipPolygonY (float[] vIn, int n, float[] vOut, float multi, float offset, int axis) {
			
			float[] d = clipPolygonCache;
			
			for (int i=0;i<n;i++) {
				d[i] = multi*vIn[i*3+axis]+offset;
			}
			
			//Number of resulting vertices
			int m = 0;
			
			for (int i=0, j = n-1; i < n; j=i, i++) {
				
				bool prev = d[j] >= 0;
				bool curr = d[i] >= 0;
				
				if (prev != curr) {
					
					vOut[m*3+1] = vIn[j*3+1] + (vIn[i*3+1]-vIn[j*3+1]) * (d[j] / (d[j] - d[i]));
					
					m++;
				}
				
				if (curr) {
					
					vOut[m*3+1] = vIn[i*3+1];
					
					m++;
				}
			}
			
			return m;
		}
		
		public static int ClipPolygon (Vector3[] vIn, int n, Vector3[] vOut, float multi, float offset, int axis) {
			float[] d = clipPolygonCache;
			
			for (int i=0;i<n;i++) {
				d[i] = multi*vIn[i][axis]+offset;
			}
			
			//Number of resulting vertices
			int m = 0;
			
			for (int i=0, j = n-1; i < n; j=i, i++) {
				
				bool prev = d[j] >= 0;
				bool curr = d[i] >= 0;
				
				if (prev != curr) {
					float s = d[j] / (d[j] - d[i]);
					
					vOut[m] = vIn[j] + (vIn[i]-vIn[j])*s;
					m++;
				}
				
				if (curr) {
					vOut[m] = vIn[i];
					m++;
				}
			}
			
			return m;
		}
		
		public static int ClipPolygon (Int3[] vIn, int n, Int3[] vOut, int multi, int offset, int axis) {
			int[] d = clipPolygonIntCache;
			
			for (int i=0;i<n;i++) {
				d[i] = multi*vIn[i][axis]+offset;
			}
			
			//Number of resulting vertices
			int m = 0;
			
			for (int i=0, j = n-1; i < n; j=i, i++) {
				
				bool prev = d[j] >= 0;
				bool curr = d[i] >= 0;
				
				if (prev != curr) {
					double s = (double)d[j] / (d[j] - d[i]);
					
					vOut[m] = vIn[j] + (vIn[i]-vIn[j])*s;
					m++;
				}
				
				if (curr) {
					vOut[m] = vIn[i];
					m++;
				}
			}
			
			return m;
		}
		
		public static bool IntersectXAxis (out Vector3 intersection,Vector3 start1,Vector3 dir1,float x) {
				
			float den = dir1.x;
			
			if (den == 0) {
				intersection = Vector3.zero;
				return false;
			}
			
			float nom = x-start1.x;
			
			float u = nom/den;
			
			u = Mathf.Clamp01 (u);
			
			intersection = start1 + dir1*u;
			return true;
		}
		
		public static bool IntersectZAxis (out Vector3 intersection,Vector3 start1,Vector3 dir1,float z) {
				
			float den = -dir1.z;
			
			if (den == 0) {
				intersection = Vector3.zero;
				return false;
			}
			
			float nom = start1.z-z;
			
			float u = nom/den;
			
			u = Mathf.Clamp01 (u);
			
			intersection = start1 + dir1*u;
			return true;
		}
	}
}
