
//#define NoVirtualUpdateH //Should UpdateH be virtual or not
//#define NoVirtualUpdateG //Should UpdateG be virtual or not
//#define NoVirtualOpen //Should Open be virtual or not
//#define NoTagPenalty		//Enables or disables tag penalties. Can give small performance boost
//#define ASTAR_SET_LEVELGRIDNODE_HEIGHT //Enables a field height for each node if you want to use that information. Not used by any included scripts. Warning: Will break node serialization if changed.
//#define ASTAR_LEVELGRIDNODE_FEW_LAYERS //Reduces the max number of layers for LayeredGridGraphs from 255 to 16. Saves memory.

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.Serialization.JsonFx;
using Pathfinding.Serialization;

namespace Pathfinding {
	
	/** Grid Graph, supports layered worlds.
	 * The GridGraph is great in many ways, reliable, easily configured and updatable during runtime.
	 * But it lacks support for worlds which have multiple layers, such as a building with multiple floors.\n
	 * That's where this graph type comes in. It supports basically the same stuff as the grid graph, but also multiple layers.
	 * It uses a more memory, and is probably a bit slower.
	 * \note Does not support 8 connections per node, only 4.

\ingroup graphs
\shadowimage{layergridgraph_graph.png}
\shadowimage{layergridgraph_inspector.png}

\astarpro
	 */
	public class LayerGridGraph : GridGraph, IRaycastableGraph, IUpdatableGraph {
		
		//This function will be called when this graph is destroyed
		public override void OnDestroy () {
			base.OnDestroy ();
			//Clean up a reference in a static variable which otherwise should point to this graph forever and stop the GC from collecting it
			RemoveGridGraphFromStatic ();
			
			//LevelGridNode.RemoveGridGraph (this);
		}
		
		public new void RemoveGridGraphFromStatic () {
			LevelGridNode.SetGridGraph ( active.astarData.GetGraphIndex (this), null);
		}
		
		public int[] nodeCellIndices;
		
		/** Number of layers.
		 * \warning Do not modify this variable
		 */
		[JsonMember]
		public int layerCount = 0;
		
		/** If two layered nodes are too close, they will be merged */
		[JsonMember]
		public float mergeSpanRange = 0.5F;
		
		/** Nodes with a short distance to the node above it will be set unwalkable */
		[JsonMember]
		public float characterHeight = 0.4F;
		
		public new LevelGridNode[] nodes;
		
		public override bool uniformWidthDepthGrid {
			get {
				return false;
			}
		}
		
		public override void GetNodes (GraphNodeDelegateCancelable del) {
			if (nodes == null) return;
			
			for (int i=0;i < nodes.Length;i++) {
				if (nodes[i] != null && !del(nodes[i])) break;
			}
		}
		
		public new void UpdateArea (GraphUpdateObject o) {
			
			if (nodes == null || nodes.Length != width*depth*layerCount) {
				Debug.LogWarning ("The Grid Graph is not scanned, cannot update area ");
				//Not scanned
				return;
			}
			
			//Copy the bounds
			Bounds b = o.bounds;
			
			//Matrix inverse 
			//node.position = matrix.MultiplyPoint3x4 (new Vector3 (x+0.5F,0,z+0.5F));
			
			Vector3 min, max;
			GetBoundsMinMax (b,inverseMatrix,out min, out max);
			
			int minX = Mathf.RoundToInt (min.x-0.5F);
			int maxX = Mathf.RoundToInt (max.x-0.5F);
			
			int minZ = Mathf.RoundToInt (min.z-0.5F);
			int maxZ = Mathf.RoundToInt (max.z-0.5F);
			//We now have coordinates in local space (i.e 1 unit = 1 node)
			
			IntRect originalRect = new IntRect(minX,minZ,maxX,maxZ);
			IntRect affectRect = originalRect;
			
			IntRect gridRect = new IntRect(0,0,width-1,depth-1);
			
			IntRect physicsRect = originalRect;
			
#if ASTARDEBUG
			Matrix4x4 debugMatrix = matrix;
			debugMatrix *= Matrix4x4.TRS (new Vector3(0.5f,0,0.5f),Quaternion.identity,Vector3.one);
			
			originalRect.DebugDraw (debugMatrix,Color.red);
#endif
			
			bool willChangeWalkability = o.updatePhysics || o.modifyWalkability;
			
			bool willChangeNodeInstances = (o is LayerGridGraphUpdate ? ((LayerGridGraphUpdate)o).recalculateNodes : false);
			bool preserveExistingNodes = (o is LayerGridGraphUpdate ? ((LayerGridGraphUpdate)o).preserveExistingNodes : true);
			
			int erosion = o.updateErosion ? erodeIterations : 0;
			
			if (o.trackChangedNodes	&& willChangeNodeInstances) {
				Debug.LogError ("Cannot track changed nodes when creating or deleting nodes.\nWill not update LayerGridGraph");
				return;
			}
			
			//Calculate the largest bounding box which might be affected
			
			if (o.updatePhysics && !o.modifyWalkability) {
				//Add the collision.diameter margin for physics calls
				if (collision.collisionCheck) {
					Vector3 margin = new Vector3 (collision.diameter,0,collision.diameter)*0.5F;
					
					min -= margin*1.02F;//0.02 safety margin, physics is rarely very accurate
					max += margin*1.02F;
					
					physicsRect = new IntRect(
					                            Mathf.RoundToInt (min.x-0.5F),
					                            Mathf.RoundToInt (min.z-0.5F),
					                            Mathf.RoundToInt (max.x-0.5F),
					                            Mathf.RoundToInt (max.z-0.5F)
					                            );
					
					affectRect = IntRect.Union (physicsRect, affectRect);
				}
			}
			
			if (willChangeWalkability || erosion > 0) {
				//Add affect radius for erosion. +1 for updating connectivity info at the border
				affectRect = affectRect.Expand (erosion + 1);
			}
			
			IntRect clampedRect = IntRect.Intersection (affectRect,gridRect);
			
			//Mark nodes that might be changed
			if (!willChangeNodeInstances) {
				for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
					for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
						for (int y=0;y<layerCount;y++) {
							o.WillUpdateNode (nodes[y*width*depth + z*width+x]);
						}
					}
				}
			}
			
			//Update Physics
			if (o.updatePhysics && !o.modifyWalkability) {
				
				collision.Initialize (matrix,nodeSize);
				
				clampedRect = IntRect.Intersection (physicsRect,gridRect);
				
				bool addedNodes = false;
				
				for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
					for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
						/** \todo FIX */
						addedNodes |= RecalculateCell (x,z,preserveExistingNodes);
					}
				}
				
				for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
					for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
						for (int y=0;y<layerCount;y++) {
							int index = y*width*depth + z*width+x;
							
							LevelGridNode node = nodes[index] as LevelGridNode;
							
							if (node == null) continue;
							
							CalculateConnections (nodes,node,x,z,y);
						}
					}
				}
			}
			
			//Apply GUO
			
			clampedRect = IntRect.Intersection (originalRect, gridRect);
			for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
				for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
					for (int y=0;y<layerCount;y++) {
						int index = y*width*depth + z*width+x;
						
						LevelGridNode node = nodes[index] as LevelGridNode;
						
						if (node == null) continue;
						
						if (willChangeWalkability) {
							node.Walkable = node.WalkableErosion;
							if (o.bounds.Contains ((Vector3)node.position)) o.Apply (node);
							node.WalkableErosion = node.Walkable;
						} else {
							if (o.bounds.Contains ((Vector3)node.position)) o.Apply (node);
						}
					}
				}
			}
			
#if ASTARDEBUG
			physicsRect.DebugDraw (debugMatrix,Color.blue);
			affectRect.DebugDraw (debugMatrix,Color.black);
#endif
			
			//Recalculate connections
			if (willChangeWalkability && erosion == 0) {
				
				clampedRect = IntRect.Intersection (affectRect, gridRect);
				for (int x = clampedRect.xmin; x <= clampedRect.xmax;x++) {
					for (int z = clampedRect.ymin;z <= clampedRect.ymax;z++) {
						for (int y=0;y<layerCount;y++) {
							int index = y*width*depth + z*width+x;
						
							LevelGridNode node = nodes[index] as LevelGridNode;
							
							if (node == null) continue;
							
							CalculateConnections (nodes,node,x,z,y);
						}
					}
				}
			} else if (willChangeWalkability && erosion > 0) {
				
				
				clampedRect = IntRect.Union (originalRect, physicsRect);
				
				IntRect erosionRect1 = clampedRect.Expand (erosion);
				IntRect erosionRect2 = erosionRect1.Expand (erosion);
				
				erosionRect1 = IntRect.Intersection (erosionRect1,gridRect);
				erosionRect2 = IntRect.Intersection (erosionRect2,gridRect);
				
#if ASTARDEBUG
				erosionRect1.DebugDraw (debugMatrix,Color.magenta);
				erosionRect2.DebugDraw (debugMatrix,Color.cyan);
#endif
				
				/*
				all nodes inside clampedRect might have had their walkability changed
				all nodes inside erosionRect1 might get affected by erosion from clampedRect and erosionRect2
				all nodes inside erosionRect2 (but outside erosionRect1) will be reset to previous walkability
				after calculation since their erosion might not be correctly calculated (nodes outside erosionRect2 would maybe have effect)
				*/
				
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						for (int y=0;y<layerCount;y++) {
							int index = y*width*depth + z*width+x;
						
							LevelGridNode node = nodes[index] as LevelGridNode;
							
							if (node == null) continue;
							
							bool tmp = node.Walkable;
							node.Walkable = node.WalkableErosion;
							
							if (!erosionRect1.Contains (x,z)) {
								//Save the border's walkabilty data in bit 16 (will be reset later)
								node.TmpWalkable = tmp;
							}
						}
					}
				}
				
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						for (int y=0;y<layerCount;y++) {
							int index = y*width*depth + z*width+x;
						
							LevelGridNode node = nodes[index] as LevelGridNode;
							
							if (node == null) continue;
							
#if ASTARDEBUG
							if (!node.walkable)
								Debug.DrawRay ((Vector3)node.position, Vector3.up*2,Color.red);
#endif				
							CalculateConnections (nodes,node,x,z,y);
						}
					}
				}
				
				//Erode the walkable area
				ErodeWalkableArea (erosionRect2.xmin,erosionRect2.ymin,erosionRect2.xmax+1,erosionRect2.ymax+1);
				
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						if (erosionRect1.Contains (x,z)) continue;
						
						for (int y=0;y<layerCount;y++) {
							int index = y*width*depth + z*width+x;
							
							LevelGridNode node = nodes[index] as LevelGridNode;
							
							if (node == null) continue;
							
							//Restore temporarily stored data
							node.Walkable = node.TmpWalkable;
						}
					}
				}
				
				//Recalculate connections of all affected nodes
				for (int x = erosionRect2.xmin; x <= erosionRect2.xmax;x++) {
					for (int z = erosionRect2.ymin;z <= erosionRect2.ymax;z++) {
						for (int y=0;y<layerCount;y++) {
							int index = y*width*depth + z*width+x;
							
							LevelGridNode node = nodes[index] as LevelGridNode;
							
							if (node == null) continue;
							
							CalculateConnections (nodes,node,x,z,y);
						}
					}
				}
			}
			//Debug.LogError ("No support for Graph Updates to Layered Grid Graphs");
		}
		
		public override void ScanInternal (OnScanStatus status) {
			
			scans++;
			
			if (nodeSize <= 0) {
				return;
			}
			
			GenerateMatrix ();
			
			if (width > 1024 || depth > 1024) {
				Debug.LogError ("One of the grid's sides is longer than 1024 nodes");
				return;
			}
			
			//GenerateBounds ();
			
			/*neighbourOffsets = new int[8] {
				-width-1,-width,-width+1,
				-1,1,
				width-1,width,width+1
			}*/
			
			SetUpOffsetsAndCosts ();
			
			//GridNode.RemoveGridGraph (this);
			
			LevelGridNode.SetGridGraph (active.astarData.GetGraphIndex(this), this);
			
			//graphNodes = new LevelGridNode[width*depth];
			
			//nodes = CreateNodes (width*depth);
			//graphNodes = nodes as LevelGridNode[];
			
			maxClimb = Mathf.Clamp (maxClimb,0,characterHeight);
			
			LinkedLevelCell[] linkedCells = new LinkedLevelCell[width*depth];
			
			if (collision == null) {
				collision = new GraphCollision ();
			}
			collision.Initialize (matrix,nodeSize);
			
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					
					linkedCells[z*width+x] = new LinkedLevelCell ();
					
					LinkedLevelCell llc = linkedCells[z*width+x];
					//GridNode node = graphNodes[z*width+x];//new LevelGridNode ();
					
					//node.SetIndex (z*width+x);
					
					Vector3 pos = matrix.MultiplyPoint3x4 (new Vector3 (x+0.5F,0,z+0.5F));
					
					
					RaycastHit[] hits = collision.CheckHeightAll (pos);
					
					//Sort the hits based on distance with bubble sort (fast enough)
					//Furthest away first (i.e lowest nodes in the graph)
					/*bool changed = true;
					while (changed) {
						changed = false;
						for (int i=0;i<hits.Length-1;i++) {
							if (hits[i].distance < hits[i+1].distance) {
								RaycastHit tmp = hits[i];
								hits[i] = hits[i+1];
								hits[i+1] = tmp;
								changed = true;
							}
						}
					}*/
					
					for (int i=0;i<hits.Length/2;i++) {
						RaycastHit tmp = hits[i];
						
						hits[i] = hits[hits.Length-1-i];
						hits[hits.Length-1-i] = tmp;
					}
					
					if (hits.Length > 0) {
						
						//lln.position = hits[0].point;
						//lln.walkable = collision.Check (lln.position);
						
						
						/*LinkedLevelNode lln = new LinkedLevelNode ();
						lln.position = hits[0].point;
						lln.walkable = collision.Check (lln.position);
						llc.first = lln;*/
						
						LinkedLevelNode lln = null;
						
						for (int i=0;i<hits.Length;i++) {
							
							LinkedLevelNode tmp = new LinkedLevelNode ();
							tmp.position = hits[i].point;
							
							if (lln != null) {
								/** \todo Use hit.distance instead */
								if (tmp.position.y - lln.position.y <= mergeSpanRange) {
									//if (tmp.position.y > lln.position.y) {
										lln.position = tmp.position;
										lln.hit = hits[i];
										lln.walkable = collision.Check (tmp.position);
									//}
									continue;
								}
							}
							
							tmp.walkable = collision.Check (tmp.position);
							tmp.hit = hits[i];
							tmp.height = float.PositiveInfinity;
							
							if (llc.first == null) {
								llc.first = tmp;
								lln = tmp;
							} else {
								lln.next = tmp;
								
								lln.height = tmp.position.y - lln.position.y;
								lln = lln.next;
							}
						
						}
					} else {
						LinkedLevelNode lln = new LinkedLevelNode ();
						lln.position = pos;
						lln.height = float.PositiveInfinity;
						lln.walkable = !collision.unwalkableWhenNoGround;
						llc.first = lln;
					}
					
					//node.penalty = 0;//Mathf.RoundToInt (Random.value*100);
					
					//node.walkable = collision.Check (node.position);
					
					//node.SetGridIndex (gridIndex);
				}
			}
			
			
			int spanCount = 0;
			layerCount = 0;
			//Count the total number of nodes in the graph
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					
					LinkedLevelCell llc = linkedCells[z*width+x];
					
					LinkedLevelNode lln = llc.first;
					int cellCount = 0;
					//Loop through all nodes in this cell
					do {
						cellCount++;
						spanCount++;
						lln = lln.next;
					} while (lln != null);
					
					layerCount = cellCount > layerCount ? cellCount : layerCount;
				}
			}
			
			if (layerCount > LevelGridNode.MaxLayerCount) {
				Debug.LogError ("Too many layers, a maximum of LevelGridNode.MaxLayerCount are allowed (found "+layerCount+")");
				return;
			}
			
			//Create all nodes
			nodes = new LevelGridNode[width*depth*layerCount];
			for (int i=0;i<nodes.Length;i++) {
				nodes[i] = new LevelGridNode (active);
				nodes[i].Penalty = initialPenalty;
			}
			
			int nodeIndex = 0;
			
			//Max slope in cosinus
			float cosAngle = Mathf.Cos (maxSlope*Mathf.Deg2Rad);
			
			for (int z = 0; z < depth; z++) {
				for (int x = 0; x < width; x++) {
					
					LinkedLevelCell llc = linkedCells[z*width+x];
					LinkedLevelNode lln = llc.first;
					
					llc.index = nodeIndex;
					
					int count = 0;
					int layerIndex = 0;
					do {
						LevelGridNode node = nodes[z*width+x + width*depth*layerIndex] as LevelGridNode;
#if ASTAR_SET_LEVELGRIDNODE_HEIGHT
						node.height = lln.height;
#endif
						node.SetPosition ((Int3)lln.position);
						node.Walkable = lln.walkable;
						
						//Adjust penalty based on the surface slope
						if (lln.hit.normal != Vector3.zero && (penaltyAngle || cosAngle < 1.0f)) {
							//Take the dot product to find out the cosinus of the angle it has (faster than Vector3.Angle)
							float angle = Vector3.Dot (lln.hit.normal.normalized,collision.up);
							
							//Add penalty based on normal
							if (penaltyAngle) {
								node.Penalty += (uint)Mathf.RoundToInt ((1F-angle)*penaltyAngleFactor);
							}
							
							//Check if the slope is flat enough to stand on
							if (angle < cosAngle) {
								node.Walkable = false;
							}
						}
						
						node.NodeInGridIndex = z*width+x;
						//node.nodeOffset = count;
						if (lln.height < characterHeight) {
							node.Walkable = false;
						}
						
						node.WalkableErosion = node.Walkable;
						
						nodeIndex++;
						count++;
						lln = lln.next;
						layerIndex++;
					} while (lln != null);
					
					for (;layerIndex<layerCount;layerIndex++) {
						nodes[z*width+x + width*depth*layerIndex] = null;
					}
					
					llc.count = count;
				}
			}
			
			nodeIndex = 0;
			
			nodeCellIndices = new int[linkedCells.Length];
			
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					
					/*LinkedLevelCell llc = linkedCells[z*width+x];
					LinkedLevelNode lln = llc.first;
					
					nodeCellIndices[z*width+x] = llc.index;
					
					do {
						LevelGridNode node = (LevelGridNode)nodes[nodeIndex];
						
						CalculateConnections (nodes,linkedCells,node,x,z,n);
						nodeIndex++;
						lln = lln.next;
					} while (lln != null);*/
					
					for (int i=0;i<layerCount;i++) {
						GraphNode node = nodes[z*width+x + width*depth*i];
						CalculateConnections (nodes,node,x,z,i);
					}
				}
			}
			
			uint graphIndex = (uint)active.astarData.GetGraphIndex(this);
			
			for (int i=0;i<nodes.Length;i++) {
				LevelGridNode lgn = nodes[i] as LevelGridNode;
				if (lgn == null) continue;
				
				UpdatePenalty (lgn);
				
				lgn.GraphIndex = graphIndex;
				
				//Set the node to be unwalkable if it hasn't got any connections
				if (!lgn.HasAnyGridConnections ()) {
					lgn.Walkable = false;
					lgn.WalkableErosion = lgn.Walkable;
				}
			}
					/*GridNode node = graphNodes[z*width+x];
				
					CalculateConnections (graphNodes,x,z,node);
					
					if (z == 5 && x == 5) {
						int index = z*width+x;
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.red);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.green);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.blue);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.yellow);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.cyan);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.magenta);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.black);
						Debug.DrawRay (node.position,(nodes[index+neighbourOffsets[0]].position-node.position)*0.5F,Color.white);
					}*/
				//}
			//}
			
			ErodeWalkableArea (0,0,width,depth);
		}
		
		/** Recalculates single cell.
		 * 
		 * \param x X coordinate of the cell
		 * \param z Z coordinate of the cell
		 * \param preserveExistingNodes If true, nodes will be reused, this can be used to preserve e.g penalty when recalculating
		 * 
		 * \returns If new layers or nodes were added. If so, you need to call
		 * AstarPath.active.DataUpdate() after this function to make sure pathfinding works correctly for them
		 * (when doing a scan, that function does not need to be called however).
		 * 
		 * \note Connections are not recalculated for the nodes.
		 */
		public bool RecalculateCell (int x, int z, bool preserveExistingNodes) {
			
			LinkedLevelCell llc = new LinkedLevelCell ();
			//GridNode node = graphNodes[z*width+x];//new LevelGridNode ();
			
			//node.SetIndex (z*width+x);
			
			Vector3 pos = matrix.MultiplyPoint3x4 (new Vector3 (x+0.5F,0,z+0.5F));
			
			
			RaycastHit[] hits = collision.CheckHeightAll (pos);
			
			//Sort the hits based on distance with bubble sort (fast enough)
			//Furthest away first (i.e lowest nodes in the graph)
			/*bool changed = true;
			while (changed) {
				changed = false;
				for (int i=0;i<hits.Length-1;i++) {
					if (hits[i].distance < hits[i+1].distance) {
						RaycastHit tmp = hits[i];
						hits[i] = hits[i+1];
						hits[i+1] = tmp;
						changed = true;
					}
				}
			}*/
			
			for (int i=0;i<hits.Length/2;i++) {
				RaycastHit tmp = hits[i];
				
				hits[i] = hits[hits.Length-1-i];
				hits[hits.Length-1-i] = tmp;
			}
			
			bool addedNodes = false;
			
			if (hits.Length > 0) {
				
				//lln.position = hits[0].point;
				//lln.walkable = collision.Check (lln.position);
				
				
				/*LinkedLevelNode lln = new LinkedLevelNode ();
				lln.position = hits[0].point;
				lln.walkable = collision.Check (lln.position);
				llc.first = lln;*/
				
				LinkedLevelNode lln = null;
				
				for (int i=0;i<hits.Length;i++) {
					
					LinkedLevelNode tmp = new LinkedLevelNode ();
					tmp.position = hits[i].point;
					
					if (lln != null) {
						/** \todo Use hit.distance instead */
						if (tmp.position.y - lln.position.y <= mergeSpanRange) {
							//if (tmp.position.y > lln.position.y) {
								lln.position = tmp.position;
								lln.hit = hits[i];
								lln.walkable = collision.Check (tmp.position);
							//}
							continue;
						}
					}
					
					tmp.walkable = collision.Check (tmp.position);
					tmp.hit = hits[i];
					tmp.height = float.PositiveInfinity;
					
					if (llc.first == null) {
						llc.first = tmp;
						lln = tmp;
					} else {
						lln.next = tmp;
						
						lln.height = tmp.position.y - lln.position.y;
						lln = lln.next;
					}
				
				}
			} else {
				LinkedLevelNode lln = new LinkedLevelNode ();
				lln.position = pos;
				lln.height = float.PositiveInfinity;
				lln.walkable = !collision.unwalkableWhenNoGround;
				llc.first = lln;
			}
			
			
			//=========
			
			uint graphIndex = (uint)active.astarData.GetGraphIndex(this);
			
			{
				//llc
				LinkedLevelNode lln = llc.first;
				
				int count = 0;
				int layerIndex = 0;
				do {
					
					if (layerIndex >= layerCount) {
						if (layerIndex+1 > LevelGridNode.MaxLayerCount) {
							Debug.LogError ("Too many layers, a maximum of LevelGridNode.MaxLayerCount are allowed (required "+(layerIndex+1)+")");
							return addedNodes;
						}
						
						AddLayers (1);
						addedNodes = true;
					}
					
					LevelGridNode node = nodes[z*width+x + width*depth*layerIndex] as LevelGridNode;
					
					if (node == null || !preserveExistingNodes) {
						//Create a new node
						nodes[z*width+x + width*depth*layerIndex] = new LevelGridNode(active);
						node = nodes[z*width+x + width*depth*layerIndex] as LevelGridNode;
						node.Penalty = initialPenalty;
						node.GraphIndex = graphIndex;
						addedNodes = true;
					}
					
					//node.connections = null;
#if ASTAR_SET_LEVELGRIDNODE_HEIGHT
					node.height = lln.height;
#endif
					node.SetPosition ((Int3)lln.position);
					node.Walkable = lln.walkable;
					node.WalkableErosion = node.Walkable;
					
					//Adjust penalty based on the surface slope
					if (lln.hit.normal != Vector3.zero) {
						//Take the dot product to find out the cosinus of the angle it has (faster than Vector3.Angle)
						float angle = Vector3.Dot (lln.hit.normal.normalized,collision.up);
						
						//Add penalty based on normal
						if (penaltyAngle) {
							node.Penalty += (uint)Mathf.RoundToInt ((1F-angle)*penaltyAngleFactor);
						}
						
						//Max slope in cosinus
						float cosAngle = Mathf.Cos (maxSlope*Mathf.Deg2Rad);
						
						//Check if the slope is flat enough to stand on
						if (angle < cosAngle) {
							node.Walkable = false;
						}
					}
					
					node.NodeInGridIndex = z*width+x;
					//node.nodeOffset = count;
					if (lln.height < characterHeight) {
						node.Walkable = false;
					}
					count++;
					lln = lln.next;
					layerIndex++;
				} while (lln != null);
				
				for (;layerIndex<layerCount;layerIndex++) {
					nodes[z*width+x + width*depth*layerIndex] = null;
				}
				
				llc.count = count;
			}
			
			return addedNodes;
		}
		
		/** Increases the capacity of the nodes array to hold more layers.
		  * After this function has been called and new nodes have been set up, the AstarPath.DataUpdate function must be called.
		  */
		public void AddLayers (int count) {
			int newLayerCount = layerCount + count;
			
			if (newLayerCount > LevelGridNode.MaxLayerCount) {
				Debug.LogError ("Too many layers, a maximum of LevelGridNode.MaxLayerCount are allowed (required "+newLayerCount+")");
				return;
			}
			
			LevelGridNode[] tmp = nodes;
			nodes = new LevelGridNode[width*depth*newLayerCount];
			for (int i=0;i<tmp.Length;i++) nodes[i] = tmp[i];
			layerCount = newLayerCount;
		}
		
		/** Updates penalty for the node.
		 * This function sets penalty to zero (0) and then adjusts it if #penaltyPosition is set to true.
		 */
		public virtual void UpdatePenalty (LevelGridNode node) {
			
			node.Penalty = 0;//Mathf.RoundToInt (Random.value*100);
			node.Penalty = initialPenalty;
			
			if (penaltyPosition) {
				node.Penalty += (uint)Mathf.RoundToInt ((node.position.y-penaltyPositionOffset)*penaltyPositionFactor);
			}
		}
		
		/** Erodes the walkable area. \see #erodeIterations */
		public override void ErodeWalkableArea (int xmin, int zmin, int xmax, int zmax) {
			//Clamp values to grid
			xmin = xmin < 0 ? 0 : (xmin > width ? width : xmin);
			xmax = xmax < 0 ? 0 : (xmax > width ? width : xmax);
			zmin = zmin < 0 ? 0 : (zmin > depth ? depth : zmin);
			zmax = zmax < 0 ? 0 : (zmax > depth ? depth : zmax);
			
			if (erosionUseTags) {
				Debug.LogError ("Erosion Uses Tags is not supported for LayerGridGraphs yet");
			}
			
			for (int it=0;it < erodeIterations;it++) {
				for (int l=0;l<layerCount;l++) {
					for (int z = zmin; z < zmax; z ++) {
						for (int x = xmin; x < xmax; x++) {
							LevelGridNode node = nodes[z*width+x + width*depth*l] as LevelGridNode;
							if (node == null) continue;
							
							if (!node.Walkable) {
								
								/*int index = node.GetIndex ();
								
								for (int i=0;i<4;i++) {
									int conn = node.GetConnectionValue (i);
									/** \todo Add constant for 0xF - InvalidConnection *
									if (conn != 0xF) {
										nodes[index+neighbourOffsets[i] + width*depth*conn].walkable = false;
									}
								}*/
							} else {
								bool anyFalseConnections = false;
							
								for (int i=0;i<4;i++) {
									if (!node.GetConnection (i)) {
										anyFalseConnections = true;
										break;
									}
								}
								
								if (anyFalseConnections) {
									node.Walkable = false;
								}
							}
						}
					}
				}
				
				//Recalculate connections
				for (int l=0;l<layerCount;l++) {
					for (int z = zmin; z < zmax; z ++) {
						for (int x = xmin; x < xmax; x++) {
							LevelGridNode node = nodes[z*width+x + width*depth*l];
							if (node == null) continue;
							CalculateConnections (nodes,node,x,z,l);
						}
					}
				}
			}
		}
		
		//public void CalculateConnections (Node[] nodes, LinkedLevelCell[] linkedCells, Node node, int x, int z, int nodeIndex) {
		public void CalculateConnections (GraphNode[] nodes, GraphNode node, int x, int z, int layerIndex) {
			
			if (node == null) return;
			
			LevelGridNode lgn = (LevelGridNode)node;
			lgn.ResetAllGridConnections ();
			
			if (!node.Walkable) {
				return;
			}
			
			float height = 0;
			if (layerIndex == layerCount-1 || nodes[lgn.NodeInGridIndex + width*depth*(layerIndex+1)] == null) {
				height = float.PositiveInfinity;
			} else {
				height = System.Math.Abs (lgn.position.y - nodes[lgn.NodeInGridIndex+width*depth*(layerIndex+1)].position.y)*Int3.PrecisionFactor;
			}
			
			for (int dir=0;dir<4;dir++) {
				
				int nx = x + neighbourXOffsets[dir];
				int nz = z + neighbourZOffsets[dir];
				
				//Check for out-of-bounds
				if (nx < 0 || nz < 0 || nx >= width || nz >= depth) {
					continue;
				}
				
				//Calculate new index
				int nIndex = nz*width+nx;
				int conn = LevelGridNode.NoConnection;
				
				for (int i=0;i<layerCount;i++) {
					GraphNode other = nodes[nIndex + width*depth*i];
					if (other != null && other.Walkable) {
						
						float otherHeight = 0;
						
						//Is there a node above this one
						if (i == layerCount-1 || nodes[nIndex+width*depth*(i+1)] == null) {
							otherHeight = float.PositiveInfinity;
						} else {
							otherHeight = System.Math.Abs (other.position.y - nodes[nIndex+width*depth*(i+1)].position.y)*Int3.PrecisionFactor;
						}
						
						float bottom = Mathf.Max (other.position.y*Int3.PrecisionFactor,lgn.position.y*Int3.PrecisionFactor);
						float top = Mathf.Min (other.position.y*Int3.PrecisionFactor+otherHeight,lgn.position.y*Int3.PrecisionFactor+height);
						
						float dist = top-bottom;
						
						if (dist >= characterHeight && Mathf.Abs (other.position.y-lgn.position.y)*Int3.PrecisionFactor <= maxClimb) {
							
						
							//Debug.DrawLine (lgn.position,other.position,new Color (0,1,0,0.5F));
							conn = i;
						}
					}
				}
				
				lgn.SetConnectionValue (dir,conn);
				
				/*LinkedLevelCell llc = linkedCells[nIndex];
				//LinkedLevelNode lln = llc.first;
				
				int conn = 0xF;
				//float minDist = Mathf.Infinity;
				
				for (int i=0;i<llc.count;i++) {
					LevelGridNode other = (LevelGridNode)nodes[llc.index+i];
					
					if (!other.walkable) {
						continue;
					}
					
					float bottom = Mathf.Max (other.position.y*Int3.PrecisionFactor,lgn.position.y*Int3.PrecisionFactor);
					float top = Mathf.Min (other.position.y*Int3.PrecisionFactor+other.height,lgn.position.y*Int3.PrecisionFactor+lgn.height);
					
					float dist = top-bottom;
					
					//if (z == 3) {
					//	Debug.DrawRay (lgn.position,Vector3.up*(dist >= characterHeight ? 2 : 0)*0.9F,Color.yellow);
					//}
					
					//Debug.DrawLine ((Vector3)lgn.position+Vector3.up,(Vector3)other.position+Vector3.up,new Color (1,0,0,0.5F));
					
					//if (Mathf.Abs (other.position.y-lgn.position.y)*Int3.PrecisionFactor > maxClimb) {
					//	Debug.DrawLine (lgn.position,other.position,new Color (1,0,0,0.5F));
					//}
					
					
					if (dist >= characterHeight && Mathf.Abs (other.position.y-lgn.position.y)*Int3.PrecisionFactor <= maxClimb) {
						
						if (i >= 0xF) {
							Debug.LogError ("Too many layers");
							continue;
						}
						
						//Debug.DrawLine (lgn.position,other.position,new Color (0,1,0,0.5F));
						conn = i;
					}
				}
				
				lgn.SetConnection (dir,conn);
				*/
				//Debug.Log ("Yey");
				//Debug.DrawLine (node.position,minNode.position,Color.yellow);
			}
			
		}
		
		
		public override NNInfo GetNearest (Vector3 position, NNConstraint constraint, GraphNode hint = null) {
			
			if (nodes == null || depth*width*layerCount != nodes.Length) {
				//Debug.LogError ("NavGraph hasn't been generated yet");
				return new NNInfo ();
			}
			
			position = inverseMatrix.MultiplyPoint3x4 (position);
			
			int x = Mathf.Clamp (Mathf.RoundToInt (position.x-0.5F)  , 0, width-1);
			int z = Mathf.Clamp (Mathf.RoundToInt (position.z-0.5F)  , 0, depth-1);
			
			int index = width*z+x;
			float minDist = float.PositiveInfinity;
			LevelGridNode minNode = null;
			for (int i=0;i<layerCount;i++) {
				LevelGridNode node = nodes[index + width*depth*i];
				if (node != null) {
					float dist =  ((Vector3)node.position - position).sqrMagnitude;
					if (dist < minDist) {
						minDist = dist;
						minNode = node;
					}
				}
			}
			
			return new NNInfo(minNode);
			
		}
		
		private LevelGridNode GetNearestNode (Vector3 position, int x, int z, NNConstraint constraint) {
			
			
			int index = width*z+x;
			float minDist = float.PositiveInfinity;
			LevelGridNode minNode = null;
			for (int i=0;i<layerCount;i++) {
				LevelGridNode node = nodes[index + width*depth*i];
				if (node != null) {
					float dist =  ((Vector3)node.position - position).sqrMagnitude;
					if (dist < minDist && constraint.Suitable (node)) {
						minDist = dist;
						minNode = node;
					}
				}
			}
			return minNode;
		}
		
		public override NNInfo GetNearestForce (Vector3 position, NNConstraint constraint) {
			
			if (nodes == null || depth*width*layerCount != nodes.Length || layerCount == 0) {
				return new NNInfo ();
			}
			
			Vector3 globalPosition = position;
			
			position = inverseMatrix.MultiplyPoint3x4 (position);
			
			int x = Mathf.Clamp (Mathf.RoundToInt (position.x-0.5F)  , 0, width-1);
			int z = Mathf.Clamp (Mathf.RoundToInt (position.z-0.5F)  , 0, depth-1);
			
			LevelGridNode minNode = null;
			float minDist = float.PositiveInfinity;
			int overlap = getNearestForceOverlap;
			
			minNode = GetNearestNode (globalPosition,x,z,constraint);
			if (minNode != null) {
				minDist = ((Vector3)minNode.position-globalPosition).sqrMagnitude;
			}
			/*if (constraint.Suitable (firstBestNode)) {
				minNode = firstBestNode;
				minDist = ((Vector3)minNode.position-globalPosition).sqrMagnitude;
			}*/
			
			if (minNode != null) {
				if (overlap == 0) return new NNInfo(minNode);
				else overlap--;
			}
			
			
			//int counter = 0;
			
			float maxDist = constraint.constrainDistance ? AstarPath.active.maxNearestNodeDistance : float.PositiveInfinity;
			float maxDistSqr = maxDist*maxDist;
			
			//for (int w = 1; w < getNearestForceLimit;w++) {
			for (int w = 1;;w++) {
				int nx = x;
				int nz = z+w;
				
				//int nz2 = nz*width;
				
				//Check if the nodes are within distance limit
				if (nodeSize*w > maxDist) {
					return new NNInfo(minNode);
				}
				
				for (nx = x-w;nx <= x+w;nx++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					LevelGridNode node = GetNearestNode (globalPosition, nx,nz, constraint);
					if (node != null) {
						float dist = ((Vector3)node.position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz*width].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) { minDist = dist; minNode = node; }
					}
				}
				
				nz = z-w;
				//nz2 = nz*width;
				
				for (nx = x-w;nx <= x+w;nx++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					LevelGridNode node = GetNearestNode (globalPosition, nx,nz, constraint);
					if (node != null) {
						float dist = ((Vector3)node.position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz*width].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) { minDist = dist; minNode = node; }
					}
				}
				
				nx = x-w;
				nz = z-w+1;
				
				for (nz = z-w+1;nz <= z+w-1; nz++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					LevelGridNode node = GetNearestNode (globalPosition, nx,nz, constraint);
					if (node != null) {
						float dist = ((Vector3)node.position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz*width].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) { minDist = dist; minNode = node; }
					}
				}
				
				nx = x+w;
				
				for (nz = z-w+1;nz <= z+w-1; nz++) {
					if (nx < 0 || nz < 0 || nx >= width || nz >= depth) continue;
					LevelGridNode node = GetNearestNode (globalPosition, nx,nz, constraint);
					if (node != null) {
						float dist = ((Vector3)node.position-globalPosition).sqrMagnitude;
						//Debug.DrawRay (nodes[nx+nz*width].position,Vector3.up*dist,Color.cyan);counter++;
						if (dist < minDist && dist < maxDistSqr) { minDist = dist; minNode = node; }
					}
				}
				
				if (minNode != null) {
					if (overlap == 0) return new NNInfo(minNode);
					else overlap--;
				}
			}
			//return new NNInfo ();
		}
		
		
		//FUNNEL ALGORITHM
		
#if FALSE
		public new void BuildFunnelCorridor (List<GraphNode> path, int sIndex, int eIndex, List<Vector3> left, List<Vector3> right) {
			
			for (int n=sIndex;n<eIndex;n++) {
				
				LevelGridNode n1 = path[n] as LevelGridNode;
				LevelGridNode n2 = path[n+1] as LevelGridNode;
				
				AddPortal (n1,n2,left,right);
			}
		}
		
		public new void AddPortal (GraphNode n1, GraphNode n2, List<Vector3> left, List<Vector3> right) {
			//Not implemented
		}
		
		public void AddPortal (LevelGridNode n1, LevelGridNode n2, List<Vector3> left, List<Vector3> right) {
			
			if (n1 == n2) {
				return;
			}
			
			int i1 = n1.GetIndex ();
			int i2 = n2.GetIndex ();
			int x1 = i1 % width;
			int x2 = i2 % width;
			int z1 = i1 / width;
			int z2 = i2 / width;
			
			Vector3 n1p = (Vector3)n1.position;
			Vector3 n2p = (Vector3)n2.position;
			
			int diffx = Mathf.Abs (x1-x2);
			int diffz = Mathf.Abs (z1-z2);
			
			if (diffx > 1 || diffz > 1) {
				//If the nodes are not adjacent to each other
				
				left.Add (n1p);
				right.Add (n1p);
				left.Add (n2p);
				right.Add (n2p);
			} else if ((diffx+diffz) <= 1){
				//If it is not a diagonal move
				
				Vector3 dir = n2p - n1p;
				//dir = dir.normalized * nodeSize * 0.5F;
				dir *= 0.5F;
				Vector3 tangent = Vector3.Cross (dir.normalized, Vector3.up);
				tangent *= /*tangent.normalized * */nodeSize * 0.5F;
				
				left.Add (n1p + dir - tangent);
				right.Add (n1p + dir + tangent);
			} else {
				//Diagonal move
				
				Vector3 dir = n2p - n1p;
				Vector3 avg = (n1p + n2p) * 0.5F;
				Vector3 tangent = Vector3.Cross (dir.normalized, Vector3.up);
				
				tangent *= nodeSize * 0.5F;
				
				left.Add (avg - tangent);
				right.Add (avg + tangent);
				
				/*Node t1 = nodes[z1 * width + x2];
				Node t2 = nodes[z2 * width + x1];
				Node target = null;
				
				if (t1.walkable) {
					target = t1;
				} else if (t2.walkable) {
					target = t2;
				}
				
				if (target == null) {
					Vector3 avg = (n1p + n2p) * 0.5F;
					
					left.Add (avg);
					right.Add (avg);
				} else {
					AddPortal (n1,(LevelGridNode)target,left,right);
					AddPortal ((LevelGridNode)target,n2,left,right);
				}*/
			}
		}
#endif
		
		/** Returns if \a _b is visible from \a _a on the graph.
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public new bool Linecast (Vector3 _a, Vector3 _b) {
			GraphHitInfo hit;
			return Linecast (_a,_b,null, out hit);
		}
		
		/** Returns if \a _b is visible from \a _a on the graph.
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [in] hint If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public new bool Linecast (Vector3 _a, Vector3 _b, GraphNode hint) {
			GraphHitInfo hit;
			return Linecast (_a,_b,hint, out hit);
		}
		
		/** Returns if \a _b is visible from \a _a on the graph.
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [out] hit Contains info on what was hit, see GraphHitInfo
		 * \param [in] hint If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public new bool Linecast (Vector3 _a, Vector3 _b, GraphNode hint, out GraphHitInfo hit) {
			return SnappedLinecast (_a,_b,hint,out hit);
		}
		
		/** Returns if \a _b is visible from \a _a on the graph.
		 * This function is different from the other Linecast functions since it 1) snaps the start and end positions directly to the graph
		 * and it uses Bresenham's line drawing algorithm as opposed to the others which use sampling at fixed intervals.
		 * If you only care about if one \b node can see another \b node, then this function is great, but if you need more precision than one node,
		 * use the normal linecast functions
		 * \param [in] _a Point to linecast from
		 * \param [in] _b Point to linecast to
		 * \param [out] hit Contains info on what was hit, see GraphHitInfo
		 * \param [in] hint (deprecated) If you have some idea of what the start node might be (the one close to \a _a), pass it to hint since it can enable faster lookups.
		 * 
		 * This is not the same as Physics.Linecast, this function traverses the graph and looks for collisions.
		 * \astarpro */
		public new bool SnappedLinecast (Vector3 _a, Vector3 _b, GraphNode hint, out GraphHitInfo hit) {
			hit = new GraphHitInfo ();
			
			//System.DateTime startTime = System.DateTime.UtcNow;
			
			LevelGridNode n1 = GetNearest (_a,NNConstraint.None).node as LevelGridNode;
			LevelGridNode n2 = GetNearest (_b,NNConstraint.None).node as LevelGridNode;
			
			if (n1 == null || n2 == null) {
				hit.node = null;
				hit.point = _a;
				return true;
			}
			
			_a = inverseMatrix.MultiplyPoint3x4 ((Vector3)n1.position);
			_a.x -= 0.5F;
			_a.z -= 0.5F;
			
			_b = inverseMatrix.MultiplyPoint3x4 ((Vector3)n2.position);
			_b.x -= 0.5F;
			_b.z -= 0.5F;
			
			Int3 a = new Int3 (Mathf.RoundToInt (_a.x),Mathf.RoundToInt (_a.y),Mathf.RoundToInt (_a.z));
			Int3 b = new Int3 (Mathf.RoundToInt (_b.x),Mathf.RoundToInt (_b.y),Mathf.RoundToInt (_b.z));
			
			hit.origin = (Vector3)a;
			
			//Debug.DrawLine (matrix.MultiplyPoint3x4 (a*100),matrix.MultiplyPoint3x4 (b*100),Color.yellow);
			
			if (!n1.Walkable) {//nodes[a.z*width+a.x].walkable) {
				hit.node = n1;//nodes[a.z*width+a.x];
				hit.point = matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
				hit.point.y = ((Vector3)hit.node.position).y;
				return true;
			}
			
			int dx = Mathf.Abs (a.x-b.x);
			int dz = Mathf.Abs (a.z-b.z);
			
			LevelGridNode currentNode = n1;
			
			while (true) {
				
				if (currentNode == n2) { //a.x == b.x && a.z == b.z) {
					
					//System.DateTime endTime2 = System.DateTime.UtcNow;
					//float theTime2 = (endTime2-startTime).Ticks*0.0001F;
			
					//Debug.Log ("Grid Linecast : Time "+theTime2.ToString ("0.00"));
			
					return false;
				}
				
				//The nodes are at the same position in the graph when seen from above
				if (currentNode.NodeInGridIndex == n2.NodeInGridIndex) {
					hit.node = currentNode;
					hit.point = (Vector3)currentNode.position;
					return true;
				}
				
				dx = System.Math.Abs(a.x-b.x);
				dz = System.Math.Abs(a.z-b.z);
				
				int dir = 0;
				
				if (dx >= dz) {
					dir = b.x>a.x ? 1 : 3;
				} else if (dz > dx) {
					dir = b.z>a.z  ? 2 : 0;
				}
				
				
				if (CheckConnection (currentNode,dir)) {
					LevelGridNode other = nodes[currentNode.NodeInGridIndex+neighbourOffsets[dir] + width*depth*currentNode.GetConnectionValue(dir)] as LevelGridNode;
					
					if (!other.Walkable) {
						hit.node = other;
						hit.point = (Vector3)other.position;
						return true;
					}
					
					//Debug.DrawLine (matrix.MultiplyPoint3x4 (a*100),matrix.MultiplyPoint3x4 (newPos*100));
					a = (Int3)inverseMatrix.MultiplyPoint3x4 ((Vector3)other.position);
					currentNode = other;
				} else {
					
					hit.node = currentNode;
					hit.point = (Vector3)currentNode.position;//matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
					return true;
				}
				
				
				/*int e2 = err*2;
				
				
				Int3 newPos = a;
				
				if (e2 > -dz) {
					err = err-dz;
					dir = sx;
					newPos.x += sx;
				}
				
				if (e2 < dx) {
					err = err+dx;
					dir += width*sz;
					newPos.z += sz;
				}
				
				if (dir == 0) {
					Debug.LogError ("Offset is zero, this should not happen");
					return false;
				}
				
				for (int i=0;i<neighbourOffsets.Length;i++) {
					if (neighbourOffsets[i] == dir) {
						if (CheckConnection (nodes[a.z*width+a.x] as LevelGridNode,i)) {
							if (!nodes[newPos.z*width+newPos.x].walkable) {
								hit.node = nodes[a.z*width+a.x];
								hit.point = matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
								hit.point.y = ((Vector3)hit.node.position).y;
								return true;
							}
							
							//Debug.DrawLine (matrix.MultiplyPoint3x4 (a*100),matrix.MultiplyPoint3x4 (newPos*100));
							a = newPos;
							break;
						} else {
						
							hit.node = nodes[a.z*width+a.x];
							hit.point = matrix.MultiplyPoint3x4 (new Vector3 (a.x+0.5F,0,a.z+0.5F));
							hit.point.y = ((Vector3)hit.node.position).y;
							return true;
						}
					}
				}*/
			}
			
			//Debug.DrawLine (_a,_b,Color.green);
			//hit.success = true;
			
		}
		
		/** Returns if \a node is connected to it's neighbour in the specified direction */
		public bool CheckConnection (LevelGridNode node, int dir) {
			return node.GetConnection (dir);
		}
		
		public override void OnDrawGizmos (bool drawNodes) {
			
			if (!drawNodes) {
				return;
			}
		
			base.OnDrawGizmos (false);
			
			if (nodes == null) {
				return;
			}
			
			PathHandler debugData = AstarPath.active.debugPathData;
			
			for (int n=0;n<nodes.Length;n++) {
				
				LevelGridNode node = nodes[n] as LevelGridNode;
				
				if (node == null || !node.Walkable) continue;
				
				//int index = node.GetIndex ();
				
				Gizmos.color = NodeColor (node,AstarPath.active.debugPathData);
				
				if (AstarPath.active.showSearchTree && AstarPath.active.debugPathData != null) {
					if (InSearchTree(node,AstarPath.active.debugPath)) {
						PathNode nodeR = debugData.GetPathNode (node);
						if (nodeR != null && nodeR.parent != null) {
							Gizmos.DrawLine ((Vector3)node.position, (Vector3)nodeR.parent.node.position);
						}
					}
				} else {
					for (int i=0;i<4;i++) {
						int conn = node.GetConnectionValue(i);//(node.gridConnections >> i*4) & 0xF;
						if (conn != LevelGridNode.NoConnection) {
							
							int nIndex = node.NodeInGridIndex + neighbourOffsets[i] + width*depth*conn;
								//nodeCellIndices[node.GetIndex ()+neighbourOffsets[i]]+conn;//index-node.nodeOffset+neighbourOffsets[i]+conn;
							
							if (nIndex < 0 || nIndex > nodes.Length) {
								continue;
							}
							
							GraphNode other = nodes[nIndex];
							
							if (other == null) continue;
							
							Gizmos.DrawLine ((Vector3)node.position,(Vector3)other.position);
						}
					}
				}
			}
		}
		
		public override void SerializeExtraInfo (GraphSerializationContext ctx)
		{
			if (nodes == null) {
				ctx.writer.Write(-1);
				return;
			}
			
			ctx.writer.Write (nodes.Length);
			
			for (int i=0;i<nodes.Length;i++) {
				if (nodes[i] == null) {
					ctx.writer.Write((int)-1);
				} else {
					ctx.writer.Write((int)0);
					nodes[i].SerializeNode(ctx);
				}
			}
		}
		
		public override void DeserializeExtraInfo (GraphSerializationContext ctx)
		{
			
			int count = ctx.reader.ReadInt32();
			if (count == -1) {
				nodes = null;
				return;
			}
			
			nodes = new LevelGridNode[count];
			for (int i=0;i<nodes.Length;i++) {
				if (ctx.reader.ReadInt32() != -1 ){
					nodes[i] = new LevelGridNode (active);
					nodes[i].DeserializeNode(ctx);
				} else {
					nodes[i] = null;
				}
			}
		}
		
		public override void PostDeserialization () {
			
#if ASTARDEBUG
			Debug.Log ("Grid Graph - Post Deserialize");
#endif
			
			GenerateMatrix ();
			
			SetUpOffsetsAndCosts ();
			
			if (nodes == null || nodes.Length == 0) return;
			
			//graphNodes = new GridNode[nodes.Length];
			
			LevelGridNode.SetGridGraph (AstarPath.active.astarData.GetGraphIndex(this),this);
			
			for (int z = 0; z < depth; z ++) {
				for (int x = 0; x < width; x++) {
					for (int i=0;i<layerCount;i++) {
						LevelGridNode node = nodes[z*width+x + width*depth*i];
						
						if (node == null) {
							continue;
						}
						
						node.NodeInGridIndex = z*width+x;
					}
				}
			}
		}
		
	}
	
	public class LinkedLevelCell {
		public int count;
		public int index;
		
		public LinkedLevelNode first;
		
	}
	
	public class LinkedLevelNode {
		public Vector3 position;
		public bool walkable;
		public RaycastHit hit;
		public float height;
		public LinkedLevelNode next;
	}
	
	/** Describes a single node for the LayeredGridGraph.
	  * Works almost the same as a grid node, except that it also stores to which layer the connections go to
	  */
	public class LevelGridNode : GraphNode {
		
		const int GridFlagsWalkableErosionOffset = 8;
		const int GridFlagsWalkableErosionMask = 1 << GridFlagsWalkableErosionOffset;
		
		const int GridFlagsWalkableTmpOffset = 9;
		const int GridFlagsWalkableTmpMask = 1 << GridFlagsWalkableTmpOffset;
		
		public LevelGridNode (AstarPath astar) : base (astar) {
		}
		
		private static LayerGridGraph[] _gridGraphs = new LayerGridGraph[0];
		public static LayerGridGraph GetGridGraph (uint graphIndex) { return _gridGraphs[(int)graphIndex]; }
		
		public static void SetGridGraph (int graphIndex, LayerGridGraph graph) {
			if (_gridGraphs.Length <= graphIndex) {
				LayerGridGraph[] gg = new LayerGridGraph[graphIndex+1];
				for (int i=0;i<_gridGraphs.Length;i++) gg[i] = _gridGraphs[i];
				_gridGraphs = gg;
			}
			
			_gridGraphs[graphIndex] = graph;
		}
		
		protected ushort gridFlags;
		protected int nodeInGridIndex;
		
#if ASTAR_LEVELGRIDNODE_FEW_LAYERS
		protected ushort gridConnections;	
#else
		protected uint gridConnections;
#endif

#if ASTAR_SET_LEVELGRIDNODE_HEIGHT
		public float height;
#endif
		
		protected static LayerGridGraph[] gridGraphs;
		
#if ASTAR_LEVELGRIDNODE_FEW_LAYERS
		public const int NoConnection = 0xF;
		private const int ConnectionMask = 0xF;
		private const int ConnectionStride = 4;
#else
		public const int NoConnection = 0xFF;
		public const int ConnectionMask = 0xFF;
		private const int ConnectionStride = 8;
#endif
		public const int MaxLayerCount = ConnectionMask;
			
		/** Removes all grid connections from this node */
		public void ResetAllGridConnections () {
			unchecked {
#if ASTAR_LEVELGRIDNODE_FEW_LAYERS
				gridConnections = (ushort)-1;
#else
				gridConnections = (uint)-1;
#endif
			}
		}
		
		/*public override Int3 Position {
			get {
				return position;
			}
		}*/
		
		public bool WalkableErosion {
			get {
				return (gridFlags & GridFlagsWalkableErosionMask) != 0;
			}
			set {
				unchecked { gridFlags = (ushort)(gridFlags & ~GridFlagsWalkableErosionMask | (value ? (ushort)GridFlagsWalkableErosionMask : (ushort)0)); }
			}
		}
		
		public bool TmpWalkable {
			get {
				return (gridFlags & GridFlagsWalkableTmpMask) != 0;
			}
			set {
				unchecked { gridFlags = (ushort)(gridFlags & ~GridFlagsWalkableTmpMask | (value ? (ushort)GridFlagsWalkableTmpMask : (ushort)0)); }
			}
		}
		
		public int NodeInGridIndex { get { return nodeInGridIndex;} set { nodeInGridIndex = value; }}
		
		/** Does this node have any grid connections */
		public bool HasAnyGridConnections () {
			unchecked {
#if ASTAR_LEVELGRIDNODE_FEW_LAYERS
				return gridConnections != (ushort)-1;
#else
				return gridConnections != (uint)-1;
#endif
			}
		}
		
		public void SetPosition (Int3 position) {
			this.position = position;
		}
		
		public override void ClearConnections (bool alsoReverse) {
			if (alsoReverse) {
				LayerGridGraph graph = GetGridGraph (GraphIndex);
				int[] neighbourOffsets = graph.neighbourOffsets;
				LevelGridNode[] nodes = graph.nodes;
				
				for (int i=0;i<4;i++) {
					int conn = GetConnectionValue(i);//(gridConnections >> i*4) & 0xF;
					if (conn != LevelGridNode.NoConnection) {
						LevelGridNode other = nodes[NodeInGridIndex+neighbourOffsets[i] + graph.width*graph.depth*conn];
						if (other != null) {
							//Remove reverse connection
							/** \todo Check if correct */
							other.SetConnectionValue(i < 4 ? ((i + 2) % 4) : (((5-2) % 4) + 4),NoConnection);
						}
					}
				}
				
			}
			
			ResetAllGridConnections ();
		}
		
		public override void GetConnections (GraphNodeDelegate del)
		{
			int index = NodeInGridIndex;
			
			LayerGridGraph graph = GetGridGraph (GraphIndex);
			int[] neighbourOffsets = graph.neighbourOffsets;
			LevelGridNode[] nodes = graph.nodes;
			
			for (int i=0;i<4;i++) {
				int conn = GetConnectionValue(i);//(gridConnections >> i*4) & 0xF;
				if (conn != LevelGridNode.NoConnection) {
					LevelGridNode other = nodes[index+neighbourOffsets[i] + graph.width*graph.depth*conn];
					if (other != null) del (other);
				}
			}
		}
		
		public override void FloodFill (Stack<GraphNode> stack, uint region) {
			int index = NodeInGridIndex;
			
			LayerGridGraph graph = GetGridGraph (GraphIndex);
			int[] neighbourOffsets = graph.neighbourOffsets;
			LevelGridNode[] nodes = graph.nodes;
			
			for (int i=0;i<4;i++) {
				int conn = GetConnectionValue(i);//(gridConnections >> i*4) & 0xF;
				if (conn != LevelGridNode.NoConnection) {
					LevelGridNode other = nodes[index+neighbourOffsets[i] + graph.width*graph.depth*conn];
					if (other != null && other.Area != region) {
						other.Area = region;
						stack.Push (other);
					}
				}
			}
		}

		public override void AddConnection (GraphNode node, uint cost) {
			throw new System.NotImplementedException("Layered Grid Nodes do not have support for adding manual connections");
		}
		
		public override void RemoveConnection (GraphNode node) {
			throw new System.NotImplementedException("Layered Grid Nodes do not have support for adding manual connections");
		}
		
		/** Is there a grid connection in that direction */	
		public bool GetConnection (int i) {
			return ((gridConnections >> i*ConnectionStride) & ConnectionMask) != NoConnection;
		}
		
		/** Set which layer a grid connection goes to.
		 * \param dir Direction for the connection.
		 * \param value The layer of the connected node or #NoConnection if there should be no connection in that direction.
		 */
		public void SetConnectionValue (int dir, int value) {
#if ASTAR_LEVELGRIDNODE_FEW_LAYERS
			gridConnections = (ushort)(gridConnections & ~((ushort)(NoConnection << dir*ConnectionStride)) | (ushort)(value << dir*ConnectionStride));
#else
			gridConnections = gridConnections & ~((uint)(NoConnection << dir*ConnectionStride)) | (uint)(value << dir*ConnectionStride);
#endif
		}
		
		/** Which layer a grid connection goes to.
		 * \param dir Direction for the connection.
		 * \returns The layer of the connected node or #NoConnection if there is no connection in that direction.
		 */
		public int GetConnectionValue (int dir) {
			return (int)((gridConnections >> dir*ConnectionStride) & ConnectionMask);
		}
		
		public override bool GetPortal (GraphNode other, List<Vector3> left, List<Vector3> right, bool backwards)
		{
			if (backwards) return true;
			
			LayerGridGraph graph = GetGridGraph (GraphIndex);
			int[] neighbourOffsets = graph.neighbourOffsets;
			LevelGridNode[] nodes = graph.nodes;
			int index = NodeInGridIndex;
			
			for (int i=0;i<4;i++) {
				int conn = GetConnectionValue(i);//(gridConnections >> i*4) & 0xF;
				if (conn != LevelGridNode.NoConnection) {
					if (other == nodes[index+neighbourOffsets[i] + graph.width*graph.depth*conn]) {
						Vector3 middle = ((Vector3)(position + other.position))*0.5f;
						Vector3 cross = Vector3.Cross (graph.collision.up, (Vector3)(other.position-position));
						cross.Normalize();
						cross *= graph.nodeSize*0.5f;
						left.Add (middle - cross);
						right.Add (middle + cross);
						return true;
					}
				}
			}
			
			return false;
		}
		
		public override void UpdateRecursiveG (Path path, PathNode pathNode, PathHandler handler) {
			//BaseUpdateAllG (nodeR, nodeRunData);
			
			handler.PushNode (pathNode);
			UpdateG (path, pathNode);
			
			LayerGridGraph graph = GetGridGraph (GraphIndex);
			int[] neighbourOffsets = graph.neighbourOffsets;
			LevelGridNode[] nodes = graph.nodes;
			int index = NodeInGridIndex;
			
			for (int i=0;i<4;i++) {
				int conn = GetConnectionValue(i);//(gridConnections >> i*4) & 0xF;
				if (conn != LevelGridNode.NoConnection) {
					
					LevelGridNode other = nodes[index+neighbourOffsets[i] + graph.width*graph.depth*conn];
					PathNode otherPN = handler.GetPathNode (other);
					
					if (otherPN != null && otherPN.parent == pathNode && otherPN.pathID == handler.PathID) {
						other.UpdateRecursiveG (path, otherPN,handler);
					}
				}
			}
		}
		
		public override void Open (Path path, PathNode pathNode, PathHandler handler) {
			
			//BaseOpen (nodeRunData, nodeR, targetPosition, path);
			
			LayerGridGraph graph = GetGridGraph(GraphIndex);
			int[] neighbourOffsets = graph.neighbourOffsets;
			uint[] neighbourCosts = graph.neighbourCosts;
			LevelGridNode[] nodes = graph.nodes;
			
			int index = NodeInGridIndex;//indices & 0xFFFFFF;
			
			for (int i=0;i<4;i++) {
				int conn = GetConnectionValue(i);//(gridConnections >> i*4) & 0xF;
				if (conn != LevelGridNode.NoConnection) {
					
					GraphNode other = nodes[index+neighbourOffsets[i] + graph.width*graph.depth*conn];
					
					if (!path.CanTraverse (other)) {
						continue;
					}
					
					PathNode otherPN = handler.GetPathNode (other);
					
					if (otherPN.pathID != handler.PathID) {
						otherPN.parent = pathNode;
						otherPN.pathID = handler.PathID;
						
						otherPN.cost = neighbourCosts[i];
						
						otherPN.H = path.CalculateHScore (other);
						other.UpdateG (path, otherPN);
						
						handler.PushNode (otherPN);
					
					} else {
						//If not we can test if the path from the current node to this one is a better one then the one already used
						uint tmpCost = neighbourCosts[i];
						
						if (pathNode.G+tmpCost+path.GetTraversalCost(other) < otherPN.G) {
							//Debug.Log ("Path better from " + NodeIndex + " to " + otherPN.node.NodeIndex + " " + (pathNode.G+tmpCost+path.GetTraversalCost(other)) + " < " + otherPN.G);
							otherPN.cost = tmpCost;
							
							otherPN.parent = pathNode;
							
							other.UpdateRecursiveG (path,otherPN, handler);
							
						//Or if the path from this node ("other") to the current ("current") is better
						} else if (otherPN.G+tmpCost+path.GetTraversalCost (this) < pathNode.G) {
							//Debug.Log ("Path better from " + otherPN.node.NodeIndex + " to " + NodeIndex + " " + (otherPN.G+tmpCost+path.GetTraversalCost (this)) + " < " + pathNode.G);
							pathNode.parent = otherPN;
							pathNode.cost = tmpCost;
							
							UpdateRecursiveG(path, pathNode, handler);
						}
					}
				}
			}
		}
		
		public override void SerializeNode (GraphSerializationContext ctx) {
			base.SerializeNode (ctx);
			ctx.writer.Write (position.x);
			ctx.writer.Write (position.y);
			ctx.writer.Write (position.z);
			ctx.writer.Write (gridFlags);
			ctx.writer.Write (gridConnections);
		}
		
		public override void DeserializeNode (GraphSerializationContext ctx)
		{
			base.DeserializeNode (ctx);
			position = new Int3(ctx.reader.ReadInt32(), ctx.reader.ReadInt32(), ctx.reader.ReadInt32());
			gridFlags = ctx.reader.ReadUInt16();
#if ASTAR_LEVELGRIDNODE_FEW_LAYERS
			gridConnections = ctx.reader.ReadUInt16();
#else
			gridConnections = ctx.reader.ReadUInt32();
#endif
		}
	}
	
	/** GraphUpdateObject with more settings for the LayerGridGraph. 
	 * \see Pathfinding.GraphUpdateObject
	 * \see Pathfinding.LayerGridGraph
	 */
	public class LayerGridGraphUpdate : GraphUpdateObject {
		/** Recalculate nodes in the graph. Nodes might be created, moved or destroyed depending on how the world has changed. */
		public bool recalculateNodes = false;
		
		/** If true, nodes will be reused. This can be used to preserve e.g penalty values when recalculating */
		public bool preserveExistingNodes = true;
		
	}
}