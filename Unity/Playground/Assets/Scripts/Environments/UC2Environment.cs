using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;

using AgentType = UC2Agent;
using StateRequest = RosMessageTypes.InterfacesPkg.UC2EnvironmentStepRequest;
using StateResponse = RosMessageTypes.InterfacesPkg.UC2EnvironmentStepResponse;
using ResetRequest = RosMessageTypes.InterfacesPkg.UC2EnvironmentResetRequest;
using ResetResponse = RosMessageTypes.InterfacesPkg.UC2EnvironmentResetResponse;


public class UC2Environment : Environment<
    StateRequest,
    StateResponse,
    ResetRequest,
    ResetResponse> {
    
    [Header("UC2 Environment Settings")]
    public AgentType agent;
    public GameObject target;
    public CameraSensor cameraSensor;
    public List<Transform> spawnPoints;


    protected override void InitializeEnvironment() {

        // Append the agent to the list
        _agents = new List<IAgent> {
            agent
        };

        // Initialize agents list
        foreach (IAgent agent in _agents) {
            agent.Initialize();
        }

        // Initialize the camera sensor
        cameraSensor.Initialize();
        
    }


    protected override void Action(StateRequest request) {

        // Send the action to the agent
        agent.Action(request.action);
        
    }


    protected override StateResponse State(TimeMsg requestReceivedTimestamp) {

        // Get the data from the camera sensor
        cameraSensor.GetSensorData();

        // Get the state from the agent
        StateResponse response = new() {
            state = agent.State(),
            request_received_timestamp = requestReceivedTimestamp,
            response_sent_timestamp = GetCurrentTimestamp(),
            compressed_image = cameraSensor.compressedImage
        };

        return response;
    }


    protected override ResetResponse ResetEnvironment(ResetRequest request, TimeMsg requestReceivedTimestamp) {

        // Choose (different) random spawn points for the agent and the target
        int agentSpawnPointIndex = Random.Range(0, spawnPoints.Count);
        int targetSpawnPointIndex = Random.Range(0, spawnPoints.Count);

        while (targetSpawnPointIndex == agentSpawnPointIndex) {
            targetSpawnPointIndex = Random.Range(0, spawnPoints.Count);
        }

        // Move the agent and the target to the spawn points
        agent.transform.SetPositionAndRotation(spawnPoints[agentSpawnPointIndex].position, spawnPoints[agentSpawnPointIndex].rotation);
        target.transform.SetPositionAndRotation(spawnPoints[targetSpawnPointIndex].position, spawnPoints[targetSpawnPointIndex].rotation);

        // Reset the camera sensor
        cameraSensor.ResetSensor();
        cameraSensor.GetSensorData();

        // Reset the agent
        ResetResponse response = new() {
            state = agent.ResetAgent(),
            request_received_timestamp = requestReceivedTimestamp,
            response_sent_timestamp = GetCurrentTimestamp(),
            compressed_image = cameraSensor.compressedImage
        };

        return response;
    }
}
