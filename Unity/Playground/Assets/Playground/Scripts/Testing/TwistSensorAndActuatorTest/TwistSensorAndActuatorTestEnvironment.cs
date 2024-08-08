// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using RosMessageTypes.InterfacesPkg;
// using System.Threading.Tasks;


// using AgentType = TwistSensorAndActuatorTestAgent;
// using ActionRequest = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestEnvironmentActionRequest;
// using ActionResponse = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestEnvironmentActionResponse;
// using StateRequest = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestEnvironmentStateRequest;
// using StateResponse = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestEnvironmentStateResponse;
// using ResetRequest = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestEnvironmentResetRequest;
// using ResetResponse = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestEnvironmentResetResponse;


// public class TwistSensorAndActuatorTestEnvironment : SingleAgentEnvironment<
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
