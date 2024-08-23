using System;
using System.Collections.Generic;
using System.Text;


public static class MessageTypeMaps {

    public static Dictionary<Type, string> SensorMessageTypeMap = new() {
        { typeof(PoseSensor), "geometry_msgs/Pose" },
        { typeof(TwistSensor), "geometry_msgs/Twist" },
        { typeof(LidarSensor), "sensor_msgs/LaserScan" },
        { typeof(TriggerSensor), "TriggerSensor" }
    };

    public static Dictionary<Type, string> ActuatorMessageTypeMap = new() {
        { typeof(TwistActuator), "geometry_msgs/Twist" },
        { typeof(PoseActuator), "geometry_msgs/Pose" }
    };

}


/// <summary>
/// JSON representation an environment.
/// This is used to serialize the environment to a JSON file.
/// </summary>
/// 
[Serializable]
public class EnvironmentJson {

    public string environmentName;
    public string rootServiceName;
    public string environmentType;
    public List<AgentJson> agents;

    public EnvironmentJson(IEnvironment environment) {

        environmentName = environment.EnvironmentName;
        rootServiceName = environment.RootServiceName;
        environmentType = environment.GetType().ToString();

        agents = new List<AgentJson>();
        foreach (IAgent agent in environment.Agents) {
            AgentJson agentJSON = new(agent);
            agents.Add(agentJSON);
        }
    }
}


/// <summary>
/// JSON representation of an agent.
/// This is used to serialize the agent to a JSON file.
/// </summary>
/// 
[Serializable]
public class AgentJson {

    public string agentName;
    public string agentType;
    public List<SensorJson> sensors;
    public List<ActuatorJson> stateActuators;
    public List<ActuatorJson> resetActuators;

    public AgentJson(IAgent agent) {

        agentName = agent.AgentName;
        agentType = agent.GetType().ToString();

        sensors = new List<SensorJson>();
        foreach (ISensor sensor in agent.Sensors) {
            SensorJson sensorJSON = new(sensor);
            sensors.Add(sensorJSON);
        }

        stateActuators = new List<ActuatorJson>();
        foreach (IActuator actuator in agent.StateActuators) {
            ActuatorJson actuatorJSON = new(actuator);
            stateActuators.Add(actuatorJSON);
        }

        resetActuators = new List<ActuatorJson>();
        foreach (IActuator actuator in agent.ResetActuators) {
            ActuatorJson actuatorJSON = new(actuator);
            resetActuators.Add(actuatorJSON);
        }
    }
}


/// <summary>
/// JSON representation of a sensor.
/// This is used to serialize the sensor to a JSON file.
/// </summary>
/// 
[Serializable]
public class SensorJson {

    public string sensorName;
    public string sensorType;
    public string messageType;

    public SensorJson(ISensor sensor) {
        sensorName = sensor.SensorName;
        sensorType = sensor.GetType().ToString();
        messageType = MessageTypeMaps.SensorMessageTypeMap[sensor.GetType()];
    }
}


/// <summary>
/// JSON representation of an actuator.
/// This is used to serialize the sensor to a JSON file.
/// </summary>
/// 
[Serializable]
public class ActuatorJson {

    public string actuatorName;
    public string actuatorType;
    public string messageType;

    public ActuatorJson(IActuator actuator) {
        actuatorName = actuator.ActuatorName;
        actuatorType = actuator.GetType().ToString();
        messageType = MessageTypeMaps.ActuatorMessageTypeMap[actuator.GetType()];
    }

}