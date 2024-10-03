Deployment
*************

#. Launch the Unity simulation. This can be done in the Unity Editor (for development) or by running the build (for deployment).

   
    * Running the scene from the Unity Editor

        * Open the Unity Project in `Playground  <../../Unity/.gitignore>`_

        * In the Unity Editor, open and play each of the User Cases, for example ``Use Case 1`` scene in `Unity Scene <../../Unity/Playground/Assets/Scenes/UC1Scene.unity>`_


    * Running the build
    
        * Open a terminal and run the file `Launch Unity Simulation <../../Unity/Playground/launch_unity_simulation.bash>`_
    
        .. code-block:: bash

            launch_unity_simulation.bat

#. (Optional) Run Windows X Server for Docker GUI visualization.

#. In Visual Studio Code attached to the running container, open two new terminals and run the following commands in each one of them:

  .. code-block:: bash

      bash launch_ros_tcp_endpoint.bash

  .. code-block:: bash

      bash launch_node.bash

The ``launch_node.bash`` file will launch the package and node specified in the configuration, executing the training logic.


.. image:: ../_static/img/deployment_steps.png
            :alt: Deployment Steps
            :align: center

.. Mermaid Diagram:
.. sequenceDiagram
..     actor User as Client
..     participant Docker
..     participant Unity
..     participant Windows X Server

..     %% Step 1: Launch Unity Simulation (Two Options)
..     Note left of User: Step 1: Launch Unity Simulation
..     alt Option A: Running from the Unity Editor (DEVELOPMENT)
..         User->>Unity: Open Unity Project in `Playground/`
..         Unity->>Unity: Open one of the use cases (e.g., UC1Scene.unity)
..     else Option B: Running the build (DEPLOYMENT)
..         User->>Docker: Open terminal in `Unity/Playground/`
..         Docker->>Docker: Run launch_unity_simulation.bash
..     end

..     %% Step 2: Run Windows X Server (Optional)
..     Note left of User: Step 2: (Optional) Run Windows X Server
..     User->>Windows X Server: For Docker GUI visualization.

..     %% Step 3: Open Terminals and Execute Commands
..     Note left of User: Step 3: Communications ROS <-> Unity
..     User->>Docker: Attach vscode to running container
..     Docker->>Unity: Run launch_ros_tcp_endpoint.bash
..     Docker->>Unity: Run launch_node.bash
