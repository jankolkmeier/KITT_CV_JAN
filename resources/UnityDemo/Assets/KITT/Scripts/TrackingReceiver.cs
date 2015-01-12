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
	public int sendPort = 9988;
	public string sendIP = "127.0.0.1";
	UdpClient udpClient;

	Queue<string> receiveQueue;
	Queue<string> sendQueue;
	Thread receiveThread;
	Thread sendThread;
	IPEndPoint endPoint;

	DatagramListener listener;

	public bool useRotation = false;
	public bool usePosition = true;
	
	bool communicationRunning = false;

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
//		newPos = new Vector3 (0, 0, 0);
		lastPos = new Vector3 (0, 0, 0);
		targetPosition = new Vector3 (0, 0, 0);
		targetRotation = Quaternion.identity;
		lastPositions = new Queue<Vector3> ();
		receiveQueue = new Queue<string> ();
		listener = new DatagramListener (rcvPort);
		Connect ();

	}
	public void Connect() {
		communicationRunning = true;
		/*
		udpClient = new UdpClient();
		endPoint = new IPEndPoint (IPAddress.Parse (sendIP), sendPort);
		ThreadStart receiveStart = new ThreadStart (ReceiveData);
		ThreadStart sendStart = new ThreadStart (SendData);
		receiveThread = new Thread(receiveStart);
		sendThread = new Thread(sendStart);
		receiveThread.Start();
		sendThread.Start();
		*/
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


		if (receiveQueue.Count > 0) {
			string raw = receiveQueue.Dequeue();
			if (raw[0] == 'a' && raw[1] != '/') {
				UpdatePosition(raw.Substring(1));
			} 
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

	
	
	void ReceiveData() {
		try {
			Thread.Sleep(1000);
			Debug.Log("Start Receive Thread");
			Debug.Log("Client created");
			//udpReceive.Client.ReceiveTimeout = 300;
			while (communicationRunning) {
				Thread.Sleep(25);
				try {
					Byte[] receiveBytes = udpClient.Receive(ref endPoint);
					string raw = Encoding.ASCII.GetString(receiveBytes);
					receiveQueue.Enqueue(raw);
					//Debug.Log("REC "+raw+" from: "+endPoint.Address.ToString()+":"+endPoint.Port.ToString());
					//if (endPoint.Port != sendPort) {
					//	receiveQueue.Enqueue(raw);
					//}
				} catch (Exception e) {
					Debug.LogWarning("Receiving Failed: "+e);
					//return;
				}
			}

		} catch (ThreadAbortException e) {
			Debug.LogWarning("Receive thread endend: "+e);
		} finally {
			Debug.Log("Receive thread closed");
			communicationRunning = false;
		}
		
		
	}
	
	void SendData() {
		try {
			while (communicationRunning) {
				Thread.Sleep(50);
				string raw = "GET translation";
				Byte[] sendBytes = Encoding.ASCII.GetBytes(raw);
				try{
					udpClient.Send(sendBytes, sendBytes.Length, endPoint);
					//Debug.Log("SND "+raw+" to: "+endPoint.Address.ToString()+":"+endPoint.Port.ToString());
					//udpClient.Client.LocalEndPoint.
				} catch ( Exception e ) {
					Debug.LogWarning("Sending Failed: "+e);
				}
			}
		} catch (Exception e) {
			Debug.LogWarning("Send thread endend: "+e);
		} finally {
			communicationRunning = false;
		}
	}
	
	void OnApplicationQuit() {
		communicationRunning = false;
		listener.Close ();
		if (udpClient != null)
			udpClient.Close();
		if (receiveThread != null)
			receiveThread.Join(500);
		if (sendThread != null)
			sendThread.Join(500);
	}

}
