using UnityEngine;
using System.Collections.Generic;

public class VehicleController : MonoBehaviour
{
    public float currentSpeed = 0f;
    public float maxSteerAngle = 30f;
    public float wheelBase = 2.5f;
    
    private List<TrajectoryPoint> currentTrajectory = new List<TrajectoryPoint>();
    private float targetSpeed = 0f;
    private string currentAction = "CRUISE";
    
    public void UpdateTrajectory(List<TrajectoryPoint> newTrajectory, float newTargetSpeed, string action)
    {
        currentTrajectory = newTrajectory;
        targetSpeed = newTargetSpeed;
        currentAction = action;
    }

    void Update()
    {
        // Simple PID Speed Control
        if (currentAction == "EMERGENCY_BRAKE" || currentAction == "STOP")
        {
            currentSpeed = Mathf.Lerp(currentSpeed, 0, Time.deltaTime * 5f);
        }
        else
        {
            currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, Time.deltaTime * 2f);
        }

        // Pure Pursuit Steering
        if (currentTrajectory != null && currentTrajectory.Count > 0)
        {
            Transform targetPoint = GetLookaheadPoint(4.0f);
            
            if (targetPoint != null)
            {
                Vector3 localTarget = transform.InverseTransformPoint(targetPoint.position);
                float steerAngle = Mathf.Atan2(2.0f * wheelBase * localTarget.x, localTarget.sqrMagnitude) * Mathf.Rad2Deg;
                steerAngle = Mathf.Clamp(steerAngle, -maxSteerAngle, maxSteerAngle);
                
                transform.Rotate(Vector3.up, steerAngle * Time.deltaTime);
            }
        }
        
        // Forward movement
        transform.Translate(Vector3.forward * currentSpeed * Time.deltaTime);
    }
    
    Transform GetLookaheadPoint(float lookaheadDistance)
    {
        // Mock lookahead implementation. In a real system, iterate through currentTrajectory
        // to find the first point where distance > lookaheadDistance.
        if (currentTrajectory.Count > 0)
        {
            GameObject go = new GameObject("Lookahead");
            go.transform.position = new Vector3(currentTrajectory[0].x, 0, currentTrajectory[0].z);
            Destroy(go, 0.1f);
            return go.transform;
        }
        return null;
    }
}
