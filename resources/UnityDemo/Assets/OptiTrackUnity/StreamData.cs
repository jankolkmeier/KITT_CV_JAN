using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace OptitrackManagement {
	
	public class StreamData {
		
		public RigidBody[] _rigidBody = new RigidBody[200];
		public int _nRigidBodies = 0;
		//public List<RigidBody> _listRigidBody = new List<RigidBody>();
		
		public StreamData ()
		{
			
			//Debug.Log("StreemData: Construct");
			InitializeRigidBody();
		}
		
		public bool InitializeRigidBody()
		{
			_nRigidBodies = 0;
			for (int i = 0; i < 200; i++)
			{
				_rigidBody[i] = new RigidBody();
			}   
			return true;
		}
		
		public bool InitializeSkeleton()
		{   
			return true;
		}
		
		public bool InitializeMarkerSet()
		{      
			return true;
		}   
	}
}