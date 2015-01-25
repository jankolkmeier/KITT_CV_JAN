using UnityEngine;
using System.Collections;

public class SetupTestCamera : MonoBehaviour {

	public string session = "test";
	public Transform target;

	int delta = 4;
	int frame_x_0 = -150;
	int frame_x_max = 150;
	int frame_y_0 = -300;
	int frame_y_max = -80;
	int frame_z;
	int frame_x;
	int frame_y;

	float last = 0.0f;
	float wait = 0.1f;

	// Use this for initialization
	void Start () {
		Debug.Log (camera.projectionMatrix);
		Screen.SetResolution (640/2, 480/2, false);

		frame_x = frame_x_0;
		frame_y = frame_y_0;
		frame_z = 0;
	}
	
	// Update is called once per frame
	void Update () {
		if (Time.time > last + wait && frame_y <= frame_y_max) {
			last = Time.time;

			if (frame_x > frame_x_max) {
				frame_x = frame_x_0;
				frame_y += delta;
			} 
			
			transform.position = new Vector3 (frame_x, frame_z, frame_y);
			transform.LookAt (target.position);
			//Application.CaptureScreenshot ("./"+session+"/frame_"+frame_x+"_"+frame_y+"_"+frame_z+".png");
			frame_x += delta;
		}

		if (Input.GetKeyUp (KeyCode.S)) {
			Save ();
		}
	}

	void Save() {
//		Application.CaptureScreenshot (session+"_frame_"+frame+".png");
//		Debug.Log(frame+": "+gameObject.transform.position);
//		Debug.Log(frame+": "+gameObject.transform.rotation);
//		frame++;
	}
}
