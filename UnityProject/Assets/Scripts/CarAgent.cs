using System;
using System.Collections;
using System.Collections.Generic;
using MLAgents;
using UnityEngine;
using UnityEngine.UI;

[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

public class CarAgent : Agent
{
    public List<AxleInfo> axleInfos; 
    public float maxMotorTorque;
    public float maxSteeringAngle;
    public float maxBrakeTorque;
    public float decelerationForce;
    public Text speedometer;
    public Text stepCount;
    public Text stepReward;
    public Text cumulativeReward;
    public Transform startingPoint;
    public bool visualObservations;
    public bool uiStats;

    private int totalLaps;
    private bool started;
    private RayPerception3D rayPerception;
    private Rigidbody agentRb;
    private readonly float[] rayAngles = { 0f, 30f, 60f, 90f, 120f, 150f, 180f };
    private readonly string[] detectableObjects = { "wall", "goal"};
    public override void InitializeAgent()
    {
        base.InitializeAgent();
        agentRb = GetComponent<Rigidbody>();
        rayPerception = GetComponent<RayPerception3D>();
    }

    public override void CollectObservations()
    {
        if (!visualObservations)
        {
            const float rayDistance = 20f;

            AddVectorObs(rayPerception.Perceive(rayDistance, rayAngles, detectableObjects, 0.5f, 0f));
        }

        var localVelocity = transform.InverseTransformDirection(agentRb.velocity);
        var forwardSpeed = Vector3.Normalize(localVelocity).z;
        AddVectorObs(forwardSpeed);
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        //Debug.Log(vectorAction[0] + "  " + vectorAction[1]);
        float motor = maxMotorTorque * vectorAction[1];
        float steering = maxSteeringAngle * vectorAction[0];

        foreach (AxleInfo axleInfo in axleInfos) {
            
            if (axleInfo.steering) {
                Steer(axleInfo, steering);
            }
            
            if (vectorAction[1] < 0)
            {
                Brake(axleInfo);
            }
            else
                Accelerate(axleInfo, motor);

            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }

        if (vectorAction[1] >= 0)
        {
            var localVelocity = transform.InverseTransformDirection(Vector3.Normalize(agentRb.velocity));
            var forwardSpeed = Vector3.Normalize(localVelocity).z;
            AddReward(forwardSpeed/5);
        }
        else
            AddReward(vectorAction[1]/10);
    }

    public override void AgentReset()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            axleInfo.leftWheel.motorTorque = 0;
            axleInfo.rightWheel.motorTorque = 0;
            axleInfo.leftWheel.brakeTorque = 2000;
            axleInfo.rightWheel.brakeTorque = 2000;
        }
        agentRb.velocity = Vector3.zero;
        agentRb.angularVelocity = Vector3.zero;
        transform.position = startingPoint.position;
        transform.rotation = Quaternion.identity;
        //transform.rotation = Quaternion.Euler(0f, 180f, 0);
        totalLaps = 0;
        started = false;
    }

    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.CompareTag("wall"))
        {
            SetReward(-1f);
            Done();
        }
        else if(other.gameObject.CompareTag("goal"))
        {
            SetReward(2f);
            Done();
        }
        /*else if (other.gameObject.CompareTag("ground"))
        {
            SetReward(-2f);
            Done();
        }*/
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.CompareTag("finish"))
        {
            if (!started)
            {
                started = true;
            }
            else
            {
                Debug.Log("Lap " + totalLaps + " completed.");
                totalLaps++;
                //Done();
            }
        }
    }

    private void FixedUpdate()
    {
        var localVelocity = transform.InverseTransformDirection(agentRb.velocity);
        var forwardSpeed = localVelocity.z;
        var speed = forwardSpeed * 3.6;
        //var speed = agentRb.velocity.magnitude * 3.6;
        if (uiStats)
        {
            speedometer.text = speed.ToString("0.00") + "KMH";
            stepCount.text = "Step Count: " + GetStepCount();
            stepReward.text = "Step Reward: " + GetReward();
            cumulativeReward.text = "Current Reward: " + GetCumulativeReward();
        }

        /*if (Input.GetKeyDown("space"))
        {
            AgentReset();
        }*/
        if (transform.localPosition.y <= -0.5)
        {
            SetReward(-1f);
            Done();
        }
        
        if (Mathf.Abs(transform.localRotation.z) >= 0.4f)
        {
            SetReward(-2f);
            Done();
        }
    }

    public void WheelCollided()
    {
        SetReward(-1f);
        Done();
    }

    private void Steer(AxleInfo axleInfo, float steering)
    {
        axleInfo.leftWheel.steerAngle = steering;
        axleInfo.rightWheel.steerAngle = steering;
    }

    private void Accelerate(AxleInfo axleInfo, float motor)
    {
        if (motor != 0)
        {
            axleInfo.leftWheel.brakeTorque = 0;
            axleInfo.rightWheel.brakeTorque = 0;
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
        }
        else if(axleInfo.motor)
        {
            Decelerate(axleInfo);
        }
    }

    private void Decelerate(AxleInfo axleInfo)
    {
        axleInfo.leftWheel.brakeTorque = decelerationForce;
        axleInfo.rightWheel.brakeTorque = decelerationForce;
    }

    private void Brake(AxleInfo axleInfo)
    {
        axleInfo.leftWheel.brakeTorque = maxBrakeTorque;
        axleInfo.rightWheel.brakeTorque = maxBrakeTorque;
    }
    
    private void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0) {
            return;
        }
     
        Transform visualWheel = collider.transform.GetChild(0);
     
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
     
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }
}
