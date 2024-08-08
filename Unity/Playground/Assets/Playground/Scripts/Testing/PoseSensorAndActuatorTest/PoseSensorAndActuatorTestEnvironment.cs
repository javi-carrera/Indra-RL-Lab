// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using RosMessageTypes.InterfacesPkg;
// using System.Threading.Tasks;


// using AgentType = PoseSensorAndActuatorTestAgent;
// using ActionRequest = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestEnvironmentActionRequest;
// using ActionResponse = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestEnvironmentActionResponse;
// using StateRequest = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestEnvironmentStateRequest;
// using StateResponse = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestEnvironmentStateResponse;
// using ResetRequest = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestEnvironmentResetRequest;
// using ResetResponse = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestEnvironmentResetResponse;


// public class PoseSensorAndActuatorTestEnvironment : SingleAgentEnvironment<
//     ActionRequest,
//     ActionResponse,
//     StateRequest,
//     StateResponse,
//     ResetRequest,
//     ResetResponse> {

//     [Header("Agent")]
//     public AgentType _agent;


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
