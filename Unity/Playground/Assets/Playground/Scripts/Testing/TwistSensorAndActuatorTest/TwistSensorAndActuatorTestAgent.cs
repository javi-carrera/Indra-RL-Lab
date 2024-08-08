// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using RosMessageTypes.InterfacesPkg;
// using Unity.Robotics.ROSTCPConnector.MessageGeneration;

// using ActionMsg = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestAgentActionMsg;
// using StateMsg = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestAgentStateMsg;
// using ResetMsg = RosMessageTypes.InterfacesPkg.TwistSensorAndActuatorTestAgentStateMsg;


// public class TwistSensorAndActuatorTestAgent : Agent<
//     ActionMsg,
//     StateMsg,
//     ResetMsg> {

//     [SerializeField]
//     private TwistSensor _twistSensor;

//     [SerializeField]
//     private TwistActuator _twistActuator;


//     void Start() {
        
//         // Initialize sensors list
//         _sensors = new List<Sensor> {
//             _twistSensor,
//         };
//     }

//     public override void Action(ActionMsg action) {
//         // Set actuator data
//         _twistActuator.SetData(action.target_twist);
//     }

//     public override StateMsg State() {

//         // Get sensor data
//         foreach (Sensor sensor in _sensors) {
//             sensor.GetData();
//         }

//         // Fill the response
//         StateMsg state = new StateMsg {
//             twist = _twistSensor.twist
//         };

//         return state;
//     }

//     public override StateMsg ResetAgent(ResetMsg resetAction) {
//         return State();
//     }
// }

