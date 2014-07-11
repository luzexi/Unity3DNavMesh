using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using Pathfinding;

namespace Pathfinding {
	/** \astarpro */
	public class OptimizationHandler {
		
		public static string[] folders = new string[1] {"AstarPathfindingProject"};
		
		public static void FindDefines (Dictionary<string,DefineObject> defines) {
			for (int i=0;i<folders.Length;i++) {
				FindDefines (Application.dataPath+"/"+folders[i],defines);
			}
		}
		
		public static void FindDefines (string directory, Dictionary<string,DefineObject> defines) {
			
			if (!Directory.Exists (directory)) {
				Debug.LogError ("Directory does not exist ("+directory+")");
				return;
			}
			
			
			
			DirectoryInfo dir = new DirectoryInfo(directory);
			FileInfo[] files = dir.GetFiles("*.cs");
			foreach(FileInfo filePath in files) {
				//Debug.Log ("Copying "+filePath.Name+" to "+destination);
				
					
				//string result = PreProcessScript (directory+"/"+filePath.Name, directives);
				
				string path = directory + "/" + filePath.Name;
				
				if (!File.Exists (path)) {
					Debug.LogError ("File does not exist : "+path);
					continue;
				}
		
			
				string text = File.ReadAllText (path);
				
				//Regex matching (optional //)#define [name] (optional->)//[description]
				Regex defineRegex = new Regex (@"^(//)?#define\s+?(\w+)(?:[^\n]*//(.+))?",RegexOptions.Multiline);
				
				//Regex matching "somestring" (with the quotation marks)
				Regex nameRegex = new Regex ("\"(.*?)\"",RegexOptions.Multiline);
				
				//Regex matching [something]
				Regex enumRegex = new Regex (@"\[(.*?)\]",RegexOptions.Multiline);
				
				//Loop through all matches in this file
				foreach (Match match in defineRegex.Matches(text)) {
					
					
					string name = match.Groups[2].Value;
					string key = name;
					
					bool firstAdd = false;
					
					//Create the define object if it has not already been found in another file
					if (!defines.ContainsKey (name)) {
						firstAdd = true;
						defines.Add (name, new DefineObject ());
					}
					
					DefineObject defOb = defines[name];
					
					//Get the description from group 3
					string desc = match.Groups[3].Value;
					
					defOb.name = name;
					
					//Find the optional name value in the description (in the form "MyName") and replace the name got from the key
					Match nameMatch = nameRegex.Match (desc);
					
					if (nameMatch.Success) {
						defOb.name = nameMatch.Groups[1].Value;
						desc = desc.Replace (nameMatch.Value,"");
					}
					
					//It is enabled if we couldn't find the "//" in the beginning of the string
					bool enabled = match.Groups[1].Value == "";
					
					//Check if some defines with this name are enabled and some are not
					if (!firstAdd && defOb.enabled != enabled) {
						defOb.inconsistent = true;
					}
					
					defOb.enabled = enabled;
					
					//Find the optional enum values (in the form [Value1,Value2,Value3])
					Match enumMatch = enumRegex.Match (desc);
					
					if (enumMatch.Success) {
						string enumstring = enumMatch.Groups[1].Value;
						
						//Split the enum values by ","
						string[] enums = enumstring.Split (","[0]);
						
						//Add a special value to the start of the list
						List<string> enumList = new List<string> (enums.Length+1);
						enumList.Add ("Define Disabled");
						enumList.AddRange (enums);
						
						enums = enumList.ToArray ();
						
						defOb.enumValues = enums;
						
						//Remove the enums from the description
						desc = desc.Replace (enumMatch.Value,"");
						
						if (defOb.enabled) {
							//Figure out which one is selected right now
							for (int j=0;j<defOb.enumValues.Length;j++) {
								if (key == defOb.enumValues[j]) {
									defOb.selectedEnum = j;
									break;
								}
							}
						}
					}
					
					//Only add to the brief if it was empty before
					if (defOb.brief == "") {
						defOb.brief = desc;
					}
					
					//Add to the files list
					defOb.files += (defOb.files == "" ? "" : ", ") + filePath.Name;
				}
			
			}
			
			//Search sub-folders
			DirectoryInfo[] children = dir.GetDirectories();
			foreach(DirectoryInfo dirPath in children) {
				FindDefines (directory+"/"+dirPath.Name, defines);
			}
			
		}
		
		public static void ApplyDefines (List<DefineObject> defines) {
			for (int i=0;i<folders.Length;i++) {
				ApplyDefines (Application.dataPath+"/"+folders[i],defines);
			}
		}
		
		public static void ApplyDefines (string directory, List<DefineObject> defines) {
			
			if (!Directory.Exists (directory)) {
				Debug.LogError ("Directory does not exist ("+directory+")");
				return;
			}
			
			
			
			DirectoryInfo dir = new DirectoryInfo(directory);
			FileInfo[] files = dir.GetFiles("*.cs");
			foreach(FileInfo filePath in files) {
				
				string path = directory + "/" + filePath.Name;
				
				if (!File.Exists (path)) {
					Debug.LogError ("File does not exist : "+path);
					continue;
				}
		
			
				string text = File.ReadAllText (path);
				
				StringBuilder newScript = new StringBuilder ();
				
				Regex defineRegex = new Regex (@"^(//)?#define\s+?(\w+)(?:[^\n]*//(.+))?",RegexOptions.Multiline);
				
				//Match[] matches = 
				
				int prevIndex = 0;
				
				bool changed = false;
				
				foreach (Match match in defineRegex.Matches(text)) {
					
					newScript.Append (text.Substring (prevIndex,match.Index-prevIndex));
					prevIndex = match.Index;
					
					string name = match.Groups[2].Value;
					
					//if (!defines.ContainsKey (name)) {
					DefineObject defOb = null;
					for (int i=0;i<defines.Count;i++) if (defines[i].name == name) {
						defOb = defines[i];
						break;
					}
					
					if ( defOb == null ) {
						newScript.Append (match.Value);
						continue;
					}
					
					//DefineObject defOb = defines[name];
					
					bool enabled = match.Groups[1].Value == "";
					
					if (!enabled && defOb.enabled) {
						prevIndex += 2;
						changed = true;
					} else if (enabled && !defOb.enabled) {
						newScript.Append ("//");
						changed = true;
					}
					
					if (defOb.selectedEnum != 0 && defOb.enumValues != null) {
						prevIndex = match.Groups[2].Index + match.Groups[2].Length;
						newScript.Append ("#define ");
						newScript.Append (defOb.enumValues[defOb.selectedEnum]);
						changed = true;
					}
				}
				
				if (changed) {
					newScript.Append (text.Substring (prevIndex));
					
					using (StreamWriter outfile = 
						new StreamWriter(path))
					{
						outfile.Write (newScript.ToString ());
					}
				}
			
			}
			
			DirectoryInfo[] children = dir.GetDirectories();
			foreach(DirectoryInfo dirPath in children) {
				ApplyDefines (directory+"/"+dirPath.Name, defines);
			}
			
		}
	}
	
	public class DefineObject {
		public string name;
		public bool enabled;
		public string files = "";
		public string brief = "";
		public string description;
		public bool inconsistent = false;
		public string[] enumValues;
		public int selectedEnum = 0;
	}
}