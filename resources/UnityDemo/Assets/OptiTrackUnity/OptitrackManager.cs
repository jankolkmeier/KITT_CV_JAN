using UnityEngine;
using System;
using System.Collections;
//using NatNetML;
using OptitrackManagement;

[System.Serializable]
public class RigidBodyObject {
	public GameObject gameObject;
	public bool applyRotation;
	public bool applyTranslation;
}

public class OptitrackManager : MonoBehaviour {

	public float worldScale = 10.0f;
	public string clientName;

	private Vector3 _moveVector;
	private Quaternion _rotation;

	private bool _deinitValue = false;

	public string localAdapter = "";

	public RigidBodyObject[] rigidBodyObjects;
	
	~OptitrackManager() {      
		Debug.Log("OptitrackManager: Destruct");
		OptitrackManagement.DirectMulticastSocketClient.Close();
	}
	
	void Start () {
		Debug.Log("Starting OptiTrack client as "+ clientName);
		
		OptitrackManagement.DirectMulticastSocketClient.localAdapter = localAdapter;
		OptitrackManagement.DirectMulticastSocketClient.Start();
	}
	
	// Update is called once per frame
	void Update () {
		OptitrackManagement.DirectMulticastSocketClient.Update();
		
		if(OptitrackManagement.DirectMulticastSocketClient.IsInit()) {
			StreamData networkData = OptitrackManagement.DirectMulticastSocketClient.GetStreemData();

			for (int i = 0; i < networkData._nRigidBodies; i++) {
				_moveVector = networkData._rigidBody[i].pos*worldScale;
				_moveVector.z = _moveVector.z * -1;
				_rotation = networkData._rigidBody[i].ori;
				_rotation.x = _rotation.x*-1;
				_rotation.y = _rotation.y*-1;
				if (i < rigidBodyObjects.Length && rigidBodyObjects[i].gameObject != null) {
					if (rigidBodyObjects[i].applyRotation) {
						rigidBodyObjects[i].gameObject.transform.rotation = _rotation;
					}

					if (rigidBodyObjects[i].applyTranslation) {
						rigidBodyObjects[i].gameObject.transform.position = _moveVector;
					}
				}
			}
		}
		

		if(_deinitValue) {
			_deinitValue = false;
			OptitrackManagement.DirectMulticastSocketClient.Close();
		}
	}
	
}