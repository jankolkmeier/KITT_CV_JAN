using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;

public class OptiTrackTestRun : MonoBehaviour {
	
	public Transform target;
	public Transform tracked;
	public string fileName = "test";
	public float captureInterval = 2.0f;
	public float scale = 100.0f;
	
	private float lastCapture;
	private List<string[]> output;
	private int img;
	private WebCamTexture webCamTex;
	private string folderName;
	
	// Use this for initialization
	void Start () {
		folderName = System.DateTime.Now.Hour+"_"+System.DateTime.Now.Minute+"_"+System.DateTime.Now.Second;
		System.IO.Directory.CreateDirectory(folderName);
		webCamTex = new WebCamTexture();
		renderer.material.mainTexture = webCamTex;
		webCamTex.Play();
		output = new List<string[]>();
		lastCapture = captureInterval;
		img = 0;
	}
	
	// Update is called once per frame
	void Update () {
		if (Time.time > lastCapture+captureInterval) {
			lastCapture = Time.time;
			Capture();
		}
	}
	
	void Capture() {
		string imgName = fileName+"_"+img+".png";
		Vector3 relativePosition = target.position - tracked.position;
		string[] line = new string[] { 
			img.ToString(),
			(-scale * relativePosition.x).ToString(),
			(-scale * relativePosition.y).ToString(),
			(scale * relativePosition.z).ToString()
		};
		
		Debug.Log(string.Join(" ", line));
		
		Texture2D save = new Texture2D(webCamTex.width, webCamTex.height);
		save.SetPixels(webCamTex.GetPixels());
		save.Apply();
		var bytes = save.EncodeToPNG();
		Destroy(save);
		File.WriteAllBytes(folderName+"/"+imgName, bytes);
		
		output.Add(line);
		img++;
	}
	
	
	void OnApplicationQuit() {
		if (output.Count < 1) return;
		
		string delimiter = ",";
		StringBuilder sb = new StringBuilder();  
		for (int index = 0; index < output.Count; index++)  
			sb.AppendLine(string.Join(delimiter, output[index]));  
		
		File.WriteAllText(folderName+"/"+fileName+".csv", sb.ToString());
	}
}
