using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.Util;

namespace Pathfinding {
	/** Helper for navmesh cut objects.
	 * Adding an instance of this component into the scene makes
	 * sure that NavmeshCut components update the recast graph correctly when they move around.
	 * 
	 * \astarpro
	 */
	public class TileHandlerHelper : MonoBehaviour {
		
		TileHandler handler;
		
		/** How often to check if an update needs to be done (real seconds between checks).
		 * For very large worlds with lots of NavmeshCut objects, it might be a performance penalty to do this check every frame.
		 * If you think this is a performance penalty, increase this number to check less often.
		 * 
		 * For almost all games, this can be kept at 0.
		 * 
		 * If negative, no updates will be done. They must be manually triggered using #ForceUpdate
		 */
		public float updateInterval = 0;
		
		float lastUpdateTime = -999;
	
		List<Bounds> forcedReloadBounds = new List<Bounds>();
	
		/** Use the specified handler, will create one at start if not called */
		public void UseSpecifiedHandler (TileHandler handler) {
			this.handler = handler;
		}
	
		void OnEnable () {
			NavmeshCut.OnDestroyCallback += HandleOnDestroyCallback;
		}
	
		void OnDisable () {
			NavmeshCut.OnDestroyCallback -= HandleOnDestroyCallback;
		}
		
		public void DiscardPending () {
			List<NavmeshCut> cuts = NavmeshCut.GetAll();
			for (int i=0;i<cuts.Count;i++) {
				if (cuts[i].RequiresUpdate()) {
					cuts[i].NotifyUpdated ();
				}
			}
		}

		void Start () {
			if (FindObjectsOfType(typeof(TileHandlerHelper)).Length > 1) {
				Debug.LogError ("There should only be one TileHandlerHelper per scene. Destroying.");
				Destroy (this);
				return;
			}
			
			if (handler == null) {
				if (AstarPath.active == null || AstarPath.active.astarData.recastGraph == null) {
					Debug.LogWarning ("No AstarPath object in the scene or no RecastGraph on that AstarPath object");
				}
				
				handler = new TileHandler(AstarPath.active.astarData.recastGraph);
				handler.CreateTileTypesFromGraph();
			}
	
		}
	
		/** Called when a NavmeshCut is destroyed */
		void HandleOnDestroyCallback ( NavmeshCut obj ) {
			forcedReloadBounds.Add (obj.LastBounds);
			lastUpdateTime = -999;
		}
	
		/** Update is called once per frame */
		void Update () {
			
			if (updateInterval == -1 || Time.realtimeSinceStartup - lastUpdateTime < updateInterval || handler == null) {
				return;
			}
			
			ForceUpdate ();
		}
		
		/** Checks all NavmeshCut instances and updates graphs if needed */
		public void ForceUpdate () {
			
			if (handler == null) {
				throw new System.Exception ("Cannot update graphs. No TileHandler. Do not call this method in Awake.");
			}
			
			lastUpdateTime = Time.realtimeSinceStartup;
			
			List<NavmeshCut> cuts = NavmeshCut.GetAll();
	
			if ( forcedReloadBounds.Count == 0 ) {
				int any = 0;
				
				for (int i=0;i<cuts.Count;i++) {
					if (cuts[i].RequiresUpdate()) {
						any++;
						break;
					}
				}
				
				// Nothing needs to be done for now
				if (any == 0) return;
			}
	
			bool end = handler.StartBatchLoad ();
			
			//Debug.Log ("Updating...");
	
			for ( int i=0; i < forcedReloadBounds.Count; i++ ) {
				handler.ReloadInBounds (forcedReloadBounds[i]);
			}
			forcedReloadBounds.Clear();
	
			for (int i=0;i<cuts.Count;i++) {
				if (cuts[i].enabled) {
					if (cuts[i].RequiresUpdate()) {
						handler.ReloadInBounds (cuts[i].LastBounds);
						handler.ReloadInBounds (cuts[i].GetBounds());
					}
				} else if (cuts[i].RequiresUpdate()) {
					handler.ReloadInBounds (cuts[i].LastBounds);
				}
			}
			
			for (int i=0;i<cuts.Count;i++) {
				if (cuts[i].RequiresUpdate()) {
					cuts[i].NotifyUpdated ();
				}
			}
			
			if (end) handler.EndBatchLoad ();
		}
	}
}