RL Tactic Tanks |version| documentation
#########################################
The Playground project provides a versatile framework designed for reinforcement learning (RL) and robotics simulations. It integrates Unity for physics simulation and ROS2 for handling communication between simulation environments and RL agents. 
This setup supports both development (in the Unity Editor) and deployment (with Unity standalone builds), allowing users to customize training environments, run parallel simulations, and manage the training process through Dockerized ROS2 nodes.

.. .. raw:: html

..    <div style="text-align: center;">
..       <img src="_static/gif/Booooooom.gif" alt="NGWS 2040 by FCAS" width="500">
..    </div>

.. _getting-help:

Getting help
============

Having trouble? We'd like to help!

* Try the :doc:`FAQ <faq>` -- it's got answers to some common questions.
* Looking for specific information? Try the :ref:`genindex` or :ref:`modindex`.
* Report bugs with Scrapy in our `issue tracker`_.
* Join the Discord community `INDRARL Discord`_.

.. _issue tracker: https://github.com/javi-carrera/Indra-RL-Lab/issues 
.. _INDRARL Discord: https://discord.gg/qVYCNpPR

.. raw:: html

    <div style="position: relative; padding-right: 60px;">
        <div style="text-align: right; z-index: 1;">
        </div>
        <img src="_static/img/tank_sticker.png" 
             style="position: absolute; right: 0; top: 50%; transform: translateY(-50%); width: 150px; height: auto; z-index: -1;" 
             alt="Tank Sticker" />
    </div>




Getting Started
================

.. toctree::
   :maxdepth: 2
   :caption: GETTING STARTED
   :hidden:
   
   GETTING_STARTED/README
   GETTING_STARTED/training_pipeline

:doc:`GETTING_STARTED/README`
   Prerequisites, installation, and deployment steps for connecting Unity, Docker, and ROS.

:doc:`GETTING_STARTED/training_pipeline`
   Overview of how to train the RL agents, including modifying configurations, adding custom models, and running training.

Use Cases
================
.. toctree::
   :maxdepth: 2
   :caption: USE CASES
   :hidden:

   UseCases/usecase1
   UseCases/usecase2
   UseCases/usecase3

:doc:`UseCases/usecase1`
   Autonomous navigation of a tank in a simulated environment.

:doc:`UseCases/usecase2`
   Reaching and shooting a static target


Developer Guide 
================
.. toctree::
   :maxdepth: 2
   :caption: DEVELOPER GUIDE
   :hidden:

   DEVELOPER_GUIDE/Customizing_environments
   DEVELOPER_GUIDE/Customizing_models
   DEVELOPER_GUIDE/Training_definition
   DEVELOPER_GUIDE/Saving_and_loading_models
   DEVELOPER_GUIDE/Wandb_Logging

:doc:`DEVELOPER_GUIDE/Customizing_environments`
   How to customize the Unity environment for training.

:doc:`DEVELOPER_GUIDE/Customizing_models` 
   How to add custom models and blocks to the RL pipeline.

:doc:`DEVELOPER_GUIDE/Training_definition`
   How to define and run training experiments.

:doc:`DEVELOPER_GUIDE/Saving_and_loading_models`
   How to save and load trained models.

:doc:`DEVELOPER_GUIDE/Wandb_Logging`
   How to use Weights & Biases for logging and monitoring.

Project Structure
=================

.. toctree::
   :maxdepth: 2
   :caption: PROJECT STRUCTURE
   :hidden:

   PROJECT_STRUCTURE/Docker
   PROJECT_STRUCTURE/Unity

:doc:`PROJECT_STRUCTURE/Docker`
   Overview of the Docker setup for the project, including the RL models, Use Cases and ROS configuration.

:doc:`PROJECT_STRUCTURE/Unity`
   Overview of the Unity project structure, including assets and settings.


Solving specific problems
=========================

.. toctree::
   :caption: Solving specific problems
   :hidden:

   faq

:doc:`faq`
    Get answers to most frequently asked questions.


Indices and tables
----------------------

* :ref:`search`

Citing RL Tactic Tanks
----------------------
To cite this project in publications:

.. code-block:: bibtex

  @article{stable-baselines3,
    author  = {},
    title   = {},
    journal = {},
    year    = {},
    volume  = {},
    number  = {},
    pages   = {},
    url     = {}
  }

Contributing
----------------------
.. .. raw:: html

..    <div style="float: right; margin-left: 10px;">
..       <img src="_static/gif/Fu.gif" alt="Welcome" width="200">
..       <div style="text-align: center;">Welcome</div>
..    </div>

To any interested in making the RL Tactic Tanks better, there are still some improvements
that need to be done.

You can check issues in the .

If you want to contribute, please read first.
