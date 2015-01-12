using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Text;
using System.Threading;
using UnityEngine;
using System;

public class DatagramListener {

	Thread listenThread;
	Queue<string> messageQueue;
	UdpClient udpClient;
	private object _messageQueueLock = new object();
	bool listening;
	int port;

	public DatagramListener(int port) {
		messageQueue = new Queue<string> ();
		ThreadStart listenStart = new ThreadStart (Listen);
		listenThread = new Thread(listenStart);
		this.port = port;
		listenThread.Start();
	}

	public void Close() {
		listening = false;
		if (udpClient != null)
			udpClient.Close();
		if (listenThread != null)
			listenThread.Join (500);
	}

	public int QueuedMessages() {
		lock (_messageQueueLock) {
			return messageQueue.Count;
		}
	}

	public string Read() {
		lock (_messageQueueLock) {
			return messageQueue.Dequeue();
		}
	}

	void Listen() {
		try {
			udpClient = new UdpClient(port);
			listening = true;
			while (listening) {
				try {
					IPEndPoint receiveEndPoint = new IPEndPoint(IPAddress.Any, port);
					Byte[] receiveBytes = udpClient.Receive(ref receiveEndPoint);
					string raw = Encoding.ASCII.GetString(receiveBytes);
					lock (_messageQueueLock) {
						messageQueue.Enqueue(raw);
					}
				} catch (Exception e) {
					if (false) Debug.Log(e);
					// TODO
				}
			}
		} catch (Exception e) {
			// TODO
			if (false) Debug.Log(e);
		} finally {
			udpClient.Close();
			listening = false;
		}
	}


}
