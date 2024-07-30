using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using UnityEditor;

public class EscoPilotExampleEnvironment : SingleAgentEnvironment<EscoPilotExampleEnvironmentStepRequest, EscoPilotExampleEnvironmentStepResponse> {

    [Header("EscoPilot Example Environment")]

    [SerializeField]
    private EscoPilotExampleAgent _agent;
    [SerializeField]
    private GameObject _target;
    [SerializeField]
    private GameObject _start_end_positions;

    private List<Vector3> _start_end_positions_list;
    
    override protected EscoPilotExampleEnvironmentStepResponse ServiceCallback(EscoPilotExampleEnvironmentStepRequest request) {


        // Agent logic here
        EscoPilotExampleEnvironmentStepResponse response = new EscoPilotExampleEnvironmentStepResponse();

        if (request.reset){
            ResetEnvironment();
        }

        else {
            // Send the action to the agent
            _agent.PerformAction(request.agent_action);
            // TODO: Wait 'sample_time'
        }

        response.agent_state = _agent.UpdateAgentState();

        return response;
    }

    override protected void Start() {
        // start from parent class
        base.Start();
        // Get all children of _start_end_positions and fill _start_end_positions_list with their positions
        _start_end_positions_list = new List<Vector3>();
        foreach (Transform child in _start_end_positions.transform) {
            _start_end_positions_list.Add(child.position);
        }
        ResetEnvironment();
    }

    public void ResetEnvironment() {
        // Relocate agent to one of the predefined positions and target to a different one
        int start_position_index = Random.Range(0, _start_end_positions_list.Count);
        int target_position_index = Random.Range(0, _start_end_positions_list.Count);
        while (target_position_index == start_position_index) {
            target_position_index = Random.Range(0, _start_end_positions_list.Count);
        }
        _agent.transform.position = _start_end_positions_list[start_position_index];
        _target.transform.position = _start_end_positions_list[target_position_index];
    }
}
