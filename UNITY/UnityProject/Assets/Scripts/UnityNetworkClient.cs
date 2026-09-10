using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class Vector3Data {
    public float x;
    public float y;
    public float z;
}

[System.Serializable]
public class UnityVehicleState {
    public string id = "ego";
    public Vector3Data position;
    public float heading;
    public float speed;
}

[System.Serializable]
public class UnityObjectVelocity {
    public float x;
    public float z;
}

[System.Serializable]
public class UnityObject {
    public string id;
    public string type;
    public Vector3Data position;
    public UnityObjectVelocity velocity;
}

[System.Serializable]
public class UnityScenario {
    public string id;
    public float time;
}

[System.Serializable]
public class UnityStateMessage {
    public float timestamp;
    public UnityScenario scenario;
    public UnityVehicleState vehicle;
    public List<UnityObject> objects;
}

[System.Serializable]
public class TrajectoryPoint {
    public float x;
    public float z;
    public float speed;
}

[System.Serializable]
public class PythonDecision {
    public string action;
    public string risk;
    public float target_speed;
}

[System.Serializable]
public class PythonCommandMessage {
    public float timestamp;
    public PythonDecision decision;
    public List<TrajectoryPoint> trajectory;
}

public class UnityNetworkClient : MonoBehaviour
{
    public bool DEMO_MODE = false;
    public string serverUrl = "ws://localhost:8765";
    
    // Unity Network library implementation (e.g. NativeWebSocket or WebSocketSharp)
    // For this SIH26037 Hackathon C# class, you should drop in NativeWebSocket.
    
    // private WebSocket ws;
    private VehicleController vehicleController;
    private ScenarioManager scenarioManager;
    
    void Start()
    {
        vehicleController = FindObjectOfType<VehicleController>();
        scenarioManager = FindObjectOfType<ScenarioManager>();
        
        if (DEMO_MODE)
        {
            Debug.LogWarning("[DEMO MODE] Python Backend is disconnected. Running local offline fallback.");
            return;
        }
        
        ConnectToServer();
    }
    
    void ConnectToServer()
    {
        // Example implementation with NativeWebSocket:
        // ws = new WebSocket(serverUrl);
        // ws.OnOpen += () => { Debug.Log("Connected to Python Brain!"); InvokeRepeating("SendState", 0.1f, 0.1f); };
        // ws.OnMessage += (bytes) => { ProcessResponse(System.Text.Encoding.UTF8.GetString(bytes)); };
        // ws.Connect();
    }
    
    void SendState()
    {
        UnityStateMessage msg = new UnityStateMessage();
        msg.timestamp = Time.time;
        
        msg.scenario = new UnityScenario { id = scenarioManager.currentScenario, time = Time.time };
        
        msg.vehicle = new UnityVehicleState {
            position = new Vector3Data { x = vehicleController.transform.position.x, y = vehicleController.transform.position.y, z = vehicleController.transform.position.z },
            heading = vehicleController.transform.eulerAngles.y * Mathf.Deg2Rad,
            speed = vehicleController.currentSpeed
        };
        
        msg.objects = new List<UnityObject>();
        foreach(var agent in FindObjectsOfType<DynamicAgent>())
        {
            msg.objects.Add(new UnityObject {
                id = agent.agentId,
                type = agent.agentType,
                position = new Vector3Data { x = agent.transform.position.x, y = agent.transform.position.y, z = agent.transform.position.z },
                velocity = new UnityObjectVelocity { x = agent.velocity.x, z = agent.velocity.z }
            });
        }
        
        // string json = JsonUtility.ToJson(msg);
        // ws.SendText(json);
    }
    
    void ProcessResponse(string json)
    {
        PythonCommandMessage cmd = JsonUtility.FromJson<PythonCommandMessage>(json);
        
        // Pass strictly to vehicle controller
        vehicleController.UpdateTrajectory(cmd.trajectory, cmd.decision.target_speed, cmd.decision.action);
    }
}
