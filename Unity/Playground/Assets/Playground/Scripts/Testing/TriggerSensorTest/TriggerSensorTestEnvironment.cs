// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using RosMessageTypes.InterfacesPkg;


// using AgentType = TriggerSensorTestAgent;
// using ActionRequest = RosMessageTypes.InterfacesPkg.TriggerSensorTestEnvironmentActionRequest;
// using ActionResponse = RosMessageTypes.InterfacesPkg.TriggerSensorTestEnvironmentActionResponse;
// using StateRequest = RosMessageTypes.InterfacesPkg.TriggerSensorTestEnvironmentStateRequest;
// using StateResponse = RosMessageTypes.InterfacesPkg.TriggerSensorTestEnvironmentStateResponse;
// using ResetRequest = RosMessageTypes.InterfacesPkg.TriggerSensorTestEnvironmentResetRequest;
// using ResetResponse = RosMessageTypes.InterfacesPkg.TriggerSensorTestEnvironmentResetResponse;


// public class TriggerSensorTestEnvironment : SingleAgentEnvironment<
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
