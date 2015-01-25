using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;

namespace OptitrackManagement {
	
	// marker
	public class Marker
	{
		public int ID = -1;
		public Vector3 pos;      
	}
	
	// bone
	public class RigidBody
	{
		public string name = "";
		public int ID = -1;
		public int parentID = -1;
		public float length = 1.0f;
		public Vector3 pos;
		public Quaternion ori;
		public Quaternion initOri;
		public Transform trans;
	}
	
	public class SkeletonData 
	{
		
		
	}
	
}