using UnityEngine;
using System.Collections;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Text;
using System.Threading;
using System.Collections.Generic;
using System;

public class TrackingReceiver : MonoBehaviour {
	public int rcvPort = 9989;

	DatagramListener listener;

	public bool useRotation = false;
	public bool usePosition = true;

	public Transform target;

//	Vector3 newPos;
	Vector3 lastPos;
	Quaternion targetRotation;
	Vector3 targetPosition;

	Queue<Vector3> lastPositions;
	Queue<Quaternion> lastRotations;

	void Awake() {
		UnityEngine.Object.DontDestroyOnLoad (transform.gameObject);
	}

	// Use this for initialization
	void Start () {
		lastPos = new Vector3 (0, 0, 0);
		targetPosition = new Vector3 (0, 0, 0);
		targetRotation = Quaternion.identity;
		lastPositions = new Queue<Vector3> ();
		listener = new DatagramListener (rcvPort);
	}

	void UpdatePosition(string raw) {
		string[] trans = raw.Split ('\n');
		string[] pos = trans[0].Split(' ');
		string[] rot = trans[1].Split(' ');
		Vector3 newPos = new Vector3(float.Parse(pos[1]), float.Parse(pos[2]), -float.Parse(pos[3]));
		Quaternion newRot = Quaternion.Euler(-(float.Parse(rot[2])-180.0f), -float.Parse(rot[1]), float.Parse(rot[3]));
		targetRotation = newRot;
		targetPosition = newPos;
	}

	void Update() {
		if (listener.QueuedMessages () > 0) {
			string raw = listener.Read();
			UpdatePosition(raw);
		}
		
		if (Input.GetKeyDown(KeyCode.R)) {
			useRotation = !useRotation;
		}
		
		if (Input.GetKeyDown(KeyCode.P)) {
			usePosition = !usePosition;
		}
		
		if (usePosition) target.transform.position = Vector3.Lerp (target.transform.position, targetPosition, 0.1f);
		if (useRotation) target.transform.rotation = Quaternion.Slerp(target.transform.rotation, targetRotation, 0.1f);
	}

	
	void OnApplicationQuit() {
		listener.Close ();
	}

}
