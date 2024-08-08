// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using RosMessageTypes.InterfacesPkg;


// using AgentType = LidarSensorTestAgent;
// using ActionRequest = RosMessageTypes.InterfacesPkg.LidarSensorTestEnvironmentActionRequest;
// using ActionResponse = RosMessageTypes.InterfacesPkg.LidarSensorTestEnvironmentActionResponse;
// using StateRequest = RosMessageTypes.InterfacesPkg.LidarSensorTestEnvironmentStateRequest;
// using StateResponse = RosMessageTypes.InterfacesPkg.LidarSensorTestEnvironmentStateResponse;
// using ResetRequest = RosMessageTypes.InterfacesPkg.LidarSensorTestEnvironmentResetRequest;
// using ResetResponse = RosMessageTypes.InterfacesPkg.LidarSensorTestEnvironmentResetResponse;


// public class LidarSensorTestEnvironment : SingleAgentEnvironment<
//     ActionRequest,
//     ActionResponse,
//     StateRequest,
//     StateResponse,
//     ResetRequest,
//     ResetResponse> {


//     [Header("Agent")]
//     [SerializeField]
//     private AgentType _agent;
    
//     protected override ActionResponse Action(ActionRequest request) {

//         // Send the action to the agent
//         _agent.Action(request.action);

//         ActionResponse response = new ActionResponse{
//             timestamp = GetCurrentTimestamp()
//         };

//         return response;
//     }

//     protected override StateResponse State(StateRequest request) {

//         // Get the state from the agent
//         StateResponse response = new StateResponse {
//             state = _agent.State(),
//             timestamp = GetCurrentTimestamp()
//         };

//         return response;
//     }

//     protected override ResetResponse EnvironmentReset(ResetRequest request) {
//         return new ResetResponse();
//     }
// }
