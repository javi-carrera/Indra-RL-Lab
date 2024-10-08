Indra RL Lab |version| documentation
#########################################

.. only:: html

   .. image:: _static/img/uc2/tanks-sensors.png
               :alt: Sensors
               :align: center



Getting help
------------

Having trouble? We'd like to help!

* Try the :doc:`FAQ <faq>` -- it's got answers to some common questions.
* Report bugs with Scrapy in our `issue tracker`_.
* Join the Discord community `Indra RL Lab Discord`_.

.. _issue tracker: https://github.com/javi-carrera/Indra-RL-Lab/issues 
.. _Indra RL Lab Discord: https://discord.gg/qVYCNpPR

Getting Started
================

.. toctree::
   :maxdepth: 2
   :caption: GETTING STARTED
   :hidden:
   
   GETTING_STARTED/prerequisites
   GETTING_STARTED/installation
   GETTING_STARTED/deployment
   GETTING_STARTED/training
   GETTING_STARTED/testing
   GETTING_STARTED/wandb

.. only:: html

   :doc:`GETTING_STARTED/prerequisites` Unity, Docker, and ROS.

   :doc:`GETTING_STARTED/installation` Unity, Docker, and ROS.

   :doc:`GETTING_STARTED/deployment` implementation of .

   :doc:`GETTING_STARTED/training`: Overview of how to train the RL agents, including modifying configurations, adding custom models, and running training.

   :doc:`GETTING_STARTED/testing`

   :doc:`GETTING_STARTED/wandb`: Weights & Biases Logging


Use Cases
================
.. toctree::
   :maxdepth: 2
   :caption: USE CASES
   :hidden:

   UseCases/usecase1
   UseCases/usecase2
   UseCases/usecase3

.. only:: html

   :doc:`UseCases/usecase1`: Autonomous navigation of a tank in a simulated environment.

   :doc:`UseCases/usecase2`: Reaching and shooting a static target.



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

.. only:: html

   :doc:`DEVELOPER_GUIDE/Customizing_environments`: How to customize the Unity environment for training.

   :doc:`DEVELOPER_GUIDE/Customizing_models`: How to add custom models and blocks to the RL pipeline.

   :doc:`DEVELOPER_GUIDE/Training_definition`: How to define and run training experiments.

   :doc:`DEVELOPER_GUIDE/Saving_and_loading_models`: How to save and load trained models.

   :doc:`DEVELOPER_GUIDE/Wandb_Logging`: How to use Weights & Biases for logging and monitoring.


Project Structure
=================

.. toctree::
   :maxdepth: 2
   :caption: PROJECT STRUCTURE
   :hidden:

   PROJECT_STRUCTURE/Docker

.. only:: html

   :doc:`PROJECT_STRUCTURE/Docker`: Overview of the Docker setup for the project, including the RL models, Use Cases and ROS configuration.


Solving specific problems
=========================

.. toctree::
   :caption: Solving specific problems
   :hidden:

   faq

.. only:: html

   :doc:`faq`: Get answers to most frequently asked questions.

