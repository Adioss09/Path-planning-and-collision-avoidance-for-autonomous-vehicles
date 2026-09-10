using UnityEngine;

public class DynamicAgent : MonoBehaviour
{
    public string agentId;
    public string agentType = "car"; // pedestrian, cattle, auto, bike, car
    public Vector3 velocity;
    
    private Vector3 lastPosition;
    
    void Start()
    {
        if (string.IsNullOrEmpty(agentId))
        {
            agentId = agentType + "_" + System.Guid.NewGuid().ToString().Substring(0, 5);
        }
        lastPosition = transform.position;
    }
    
    void Update()
    {
        // In Unity, objects might be driven by physics, NavMeshAgents, or simple translation.
        // We calculate instantaneous velocity to send to Python.
        
        Vector3 displacement = transform.position - lastPosition;
        velocity = displacement / Time.deltaTime;
        
        lastPosition = transform.position;
    }
}
