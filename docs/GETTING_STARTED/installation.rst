Installation
*************

#. Clone the repository.

#. Open the repository in Visual Studio Code and start Docker Desktop

#. Install the `Docker <https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker>`_ and `Dev Containers <https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers>`_ Visual Studio Code extensions.

    .. list-table::
        :widths: auto
        :header-rows: 0
        :align: center

        * - .. image:: ../../docs/_static/img/docker_extension.png

        * - .. image:: ../../docs/_static/img/dev_containers_extension.png

#. Navigate to the `docker-compose.yml <../../docker-compose.yml>`_. file and right-click `Compose Up` to start the container.
    
    .. image:: ../_static/img/docker_compose_up.png
            :alt: Docker Compose without GPU
            :align: center

    The first time the image is built this will take several minutes. Once the image has been built and the container is running, the output will show:

        ✔ **Network indra-rl-lab_default**  Created
        ✔ **Container indra-rl-lab**        Started

    .. note::
        If your PC lacks a dedicated Nvidia graphics card, use the `docker-compose-no-gpu.yml <../../docker-compose-no-gpu.yml>`_ file.

#. Attach a Visual Studio Code to the running container by right-clicking on the running container in the Docker extension tab, and selecting `Attach Visual Studio Code`.

    .. image:: ../_static/img/docker_attach_vscode.png
        :align: center

    A new instance of Visual Studio Code will open. Here you will have access to the container's files and python environment.

#. In the attached Visual Studio Code, open a new terminal and build the ROS workspace by running:

.. code-block:: bash

    bash build.bash

Expected output:

.. code-block:: text

    Starting >>> interfaces_pkg
    Finished <<< interfaces_pkg [26.1s]                      

    Summary: 1 package finished [27.1s]
    Starting >>> rl_pkg  
    Starting >>> ros_tcp_endpoint
    Finished <<< ros_tcp_endpoint [5.57s]                                      
    Finished <<< rl_pkg [5.94s]          

    Summary: 2 packages finished [6.53s]


.. .. rubric:: Footnotes

.. .. [1] *NOTE: If your PC lacks a dedicated Nvidia graphics card, use the ``docker-compose-no-gpu.yml`` file instead.*


Repository Structure
====================

The only files you should be modifying here are the *config files* and the *custom models* to add your own torch architectures.

.. code-block:: none

    ├── configs/
    │   └── algorithm_config.yaml
    ├── experiments/
    │   └── [Environment ID]/
    │       └── [Algorithm]/
    │           └── [Experiment Name]/
    ├── rl_pipeline/
    │   ├── models/
    │   │   ├── [Custom Blocks]
    │   │   └── [Custom Feature Extractors]
    │   ├── run/
    │   │   └── rl_trainer.py
    │   └── utils/
    │       └── [Utility Scripts]
    ├── play.py
    └── train.py

.. note::

    - **configs/**: Contains YAML configuration files for experiments. Here everything related to the trainig is set up, you'll find specific example configurations for each algorithm including all the hyperparameters. 
    - **experiments/**: Stores experiment data and results locally.
    - **rl_pipeline/**:
        - **models/**: Directory where one can add custom model architectures and blocks.
        - **run/**:
            - ```rl_trainer.py```: The main trainer class.
            - ```train.py```: Script to initiate training.
            - ```play.py```: Script to run a trained agent.
        - **utils/**: Utility scripts for environment setup and other functionalities.
