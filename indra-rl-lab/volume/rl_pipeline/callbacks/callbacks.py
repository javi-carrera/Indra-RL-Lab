from stable_baselines3.common.callbacks import BaseCallback
import os
import wandb


class SaveDataCallback(BaseCallback):
    """
    Callback for saving a model every ``save_freq`` calls
    to ``env.step()``. Add more info to save if needed (like best eval models).

    :param save_freq: Save checkpoints every ``save_freq`` call of the callback.
    :param save_path: Path to the folder where the model will be saved.
    :param name_prefix: Common prefix to the saved models
    :param verbose: Verbosity level: 0 for no output, 2 for indicating when saving model checkpoint
    """

    def __init__(
        self,
        save_freq: int,
        save_path: str,
        name_prefix: str = "model",
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path
        self.name_prefix = name_prefix

    def _init_callback(self) -> None:
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _checkpoint_path(self, checkpoint_type: str = "", extension: str = "") -> str:
        # save_dir = os.path.join(self.save_path, f"{self.name_prefix}_{checkpoint_type}{self.num_timesteps}_steps.{extension}")
        save_dir = os.path.join(self.save_path, f"{self.name_prefix}.{extension}")
        return save_dir

    def _on_step(self) -> bool:
        if self.n_calls % self.save_freq == 0:
            model_path = self._checkpoint_path(extension="zip")
            self.model.save(model_path)
            if self.verbose >= 2:
                print(f"Saving model checkpoint to {model_path}")

        return True


class FitnessLoggerCallback(BaseCallback):
    def __init__(self):
        super(FitnessLoggerCallback, self).__init__()
        self.episode_fitness = []


    def _on_step(self) -> bool:
        dones = self.locals.get('dones', [])
        infos = self.locals.get('infos', [])
        for idx, done in enumerate(dones):
            if done:
                fitness = infos[idx].get('fitness')
                if fitness is not None:
                    self.episode_fitness.append(fitness)
                # Log the fitness score to WandB
                    wandb.log({'fitness/episode_fitness': fitness}, step=self.num_timesteps)
        return True