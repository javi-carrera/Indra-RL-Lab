indra-rl-lab (Docker)
**********************

Project Structure
=================

.. code-block:: none

   indra-rl-lab/
    └── [Volume]/
    |   └── [rl_pipeline]/
    |   |    └── [blocks]/
    |   |    |   ├── resnet.py/
    |   |    └── [configs]/ 
    |   |    |   ├── config.yml/
    |   |    └── [feature_extractors]/
    |   |    |   ├── base_extractor/
    |   |    |   ├── resnet_extractor/
    |   |    └── algorithm_registry/
    |   |    └── callbacks/
    |   |    └── rl_pipeline/
    |   |    └── schedulers/
    |   └── [ros_ws/src]/
    |   |     └── [interfaces_pkg]/
    |   |         └── [rl_pkg/msg]/
    |   |         |    ├── HealthInfo.msg
    |   |         |    └── ...
    |   |         └── [use_cases]/
    |   |             ├── [agents]/
    |   |             |   ├── TankAction.msg/
    |   |             |   ├── ...
    |   |             └── [uc1]/
    |   |             |    ├── UC1AgentAction.msg/
    |   |             |    ├── UC1EnvironmentStep.msg/
    |   |             |    ├── ...
    |   |             ├── [uc2]/
    |   |             └── [uc3]/
    |   └── [use_cases]/
    |        ├── [uc1]/  
    |        |    ├── deploy/
    |        |    ├── environment/
    |        |    ├── train/
    |        |    ├── test/
    |        | 
    |        └── [uc2]/
    └── Dockerfile
    └── requirements.txt


.. autoclass:: environment_node.EnvironmentNode
    :members:  # Includes all methods of MyClass
    :undoc-members:  # Includes methods without docstrings
    :show-inheritance:  # Shows inheritance for MyClass
     
RL Pipeline
================
- **algorithm_registry/**:
- **callbacks/**:

    Callback for saving a model every ``save_freq`` calls to ``env.step()``. 
    Additional information can be saved as needed (e.g., best evaluation models).

    (I NEED INIT)
    
    .. .. autoclass:: callbacks.callbacks.SaveDataCallback
    ..     :members:  # Includes all methods of MyClass
    ..     :undoc-members:  # Includes methods without docstrings
    ..     :show-inheritance:  # Shows inheritance for MyClass

- **rl_trainer/**:

- **schedulers/** that provides learning rate scheduling functions:

    #. Exponential decay learning rate scheduler:
        .. autofunction:: rl_pipeline.schedulers.exponential_schedule
    
    #. Cosine annealing learning rate scheduler:
        .. autofunction:: rl_pipeline.schedulers.cosine_schedule

Blocks
------

Configs
-------

Feature extractors
------------------


Ros_ws/src
================

interfaces_pkg
--------------
- **rl_pkg/msg**: Contains the message definitions for the ROS topics.
        - **HealthInfo**: Contains the use cases for the project.
        - **...**

- **use_cases/**: Contains the use cases for the project.
        - **agents**: Contains the use cases for the project.
            - **TankAction**: Contains the use cases for the project.
            - **...**
        - **uc1**
            - **UC1AgentAction**: Contains the use cases for the project.
            - **UC1EnvironmentStep**: Contains the use cases for the project.
            - **...**
        - **uc2**
            - **...**
        - **uc3**
            - **...**



rl_pkg
------
- **launch**: 
    - **deploy_launch**: 
    - **train_launch**:
    - **test_launch**:
- **resource**:
- **rl_pkg**:
    - **utils**:
    - **visualizers**:
    - **wrappers**:
    

ros_tcp_endpoint
----------------

- **launch**: 
    - **ros_tcp_endpoint_launch**: 
- **resource**:
- **ros_tcp_endpoint**:
    - **client.py**
    - **service.py**
    - **...**


Use cases
===========

- **uc1**: 
    - **deploy**: 
    - **environment**:
    - **train**:
    - **test**:
- **uc2**:
    - **...**: 
- **config.yml**:


Unity
===========
If wanting to change the Unity environment by creating new scenarios, the following files should be modified `Indra-RL-Lab-Unity <https://github.com/javi-carrera/Indra-RL-Lab-Unity>`_ 