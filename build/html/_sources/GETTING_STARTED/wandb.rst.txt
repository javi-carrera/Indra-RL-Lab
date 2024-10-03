
Weights & Biases Logging
**************************

The framework integrates with Weights & Biases for experiment tracking.

W&B logging
============

#. Set ``use_wandb: true`` in the ``training`` section of your configuration file.

    .. code-block:: yaml  

      training:
        algorithm: 'ppo'
        pretrained_model: 'None'
        use_wandb: true

#. Run ``wandb login`` and enter your `API key <https://wandb.ai/authorize>`_.

Access W&B Logs
================

Visit `wandb.ai <https://wandb.ai>`_ to see the runs under the project named ``sb3`` (or adjust the project name in ``RLTrainer``).

  .. code-block:: python

    if self._config['use_wandb']:
            self._wandb_run = wandb.init(
                project="sb3",
                name=exp_name,
                group=wandb_group,
                config=config,
                sync_tensorboard=True,
                monitor_gym=True,
                save_code=False,
            )

Local Logging
=============

All experiment data is saved locally in the ``experiments/`` directory:

(MISSING) WHERE AND WY??:

.. code-block:: none

   experiments/
   └── [Environment ID]/
       └── [Algorithm]/
           └── [Experiment Name]/
               ├── videos/
               └── model.zip