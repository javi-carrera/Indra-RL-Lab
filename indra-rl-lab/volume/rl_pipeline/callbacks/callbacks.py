from stable_baselines3.common.callbacks import BaseCallback
import os
from stable_baselines3.common.vec_env import SubprocVecEnv

class SelfPlayCallback(BaseCallback):

    def __init__(
        self,
        env: SubprocVecEnv,
        update_freq: int,
        log_dir: str = None,
        verbose: int = 0,
    ):
        super().__init__(verbose)

        self.env = env
        self.update_freq = update_freq
        self.log_dir = log_dir

    # def _init_callback(self) -> None:
    #     self.update_model(str(self.log_dir), 'model')
    
    def _on_step(self) -> bool:
        
        if self.n_calls % self.update_freq == 0:
            self.update_model(str(self.log_dir), 'best_model')

        return True
    
    def update_model(self, log_dir: str, checkpoint: str):

        self.env.env_method("update_pretrained_model", log_dir, checkpoint)
        if self.verbose >= 2:
            print(f"Updating pretrained model")

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

