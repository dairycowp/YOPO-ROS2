"""
Training Strategy
supervised learning, imitation learning, testing, rollout
"""
import os
import time
import atexit
from torch.nn import functional as F
from rich.progress import Progress
from torch.utils.data import DataLoader
from torch.utils.tensorboard.writer import SummaryWriter

from config.config import cfg
from loss.loss_function import YOPOLoss
from policy.yopo_network import YopoNetwork
from policy.yopo_dataset import YOPODataset
from policy.state_transform import *


class YopoTrainer:
    def __init__(
            self,
            learning_rate=0.001,
            batch_size=32,
            loss_weight=[],
            tensorboard_path=None,
            checkpoint_path=None,
            save_on_exit=False,
    ):
        self.batch_size = batch_size
        self.max_grad_norm = 0.1
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.loss_weight = loss_weight
        if save_on_exit: self._exit_func = atexit.register(self.save_model)
        # logger
        self.progress_log = Progress()
        self.tensorboard_path = self.get_next_log_path(tensorboard_path)
        self.tensorboard_log = SummaryWriter(log_dir=self.tensorboard_path)
        # params
        self.traj_num = cfg['traj_num']

        # network
        print("Loading network...")
        self.policy = YopoNetwork()
        self.policy = self.policy.to(self.device)
        try:
            state_dict = torch.load(checkpoint_path, weights_only=True)
            self.policy.load_state_dict(state_dict)
            print("Checkpoint ", checkpoint_path, " loaded successfully")
        except FileNotFoundError:
            print("Training from scratch")

        # loss
        self.yopo_loss = YOPOLoss()

        # optimizer
        self.optimizer = torch.optim.AdamW(self.policy.parameters(), lr=learning_rate, fused=True)
        print("Network Loaded! Loading Dataset...")

        # dataset
        self.train_dataloader = DataLoader(YOPODataset(mode='train'), batch_size=self.batch_size, shuffle=True,
                                           num_workers=1, pin_memory=True)
        self.val_dataloader = DataLoader(YOPODataset(mode='valid'), batch_size=self.batch_size, shuffle=False,
                                         num_workers=1, pin_memory=True)
        print("Dataset Loaded!")

    def train(self, epoch, save_interval=None):
        with self.progress_log:
            total_progress = self.progress_log.add_task("Training", total=epoch)
            for self.epoch_i in range(epoch):
                self.policy.train()
                self.train_one_epoch(self.epoch_i, total_progress)
                self.eval_one_epoch(self.epoch_i)
                if save_interval is not None and (self.epoch_i + 1) % save_interval == 0:
                    self.progress_log.console.log("Saving model...")
                    policy_path = self.tensorboard_path + "/epoch{}.pth".format(self.epoch_i + 1, 0)
                    torch.save(self.policy.state_dict(), policy_path)
            self.progress_log.console.log("Train YOPO Finish!")
            self.progress_log.remove_task(total_progress)

    def train_one_epoch(self, epoch: int, total_progress):
        one_epoch_progress = self.progress_log.add_task(f"Epoch: {epoch}", total=len(self.train_dataloader))
        inspect_interval = max(1, len(self.train_dataloader) // 16)
        traj_losses, score_losses, smooth_losses, safety_losses, goal_losses, start_time = [], [], [], [], [], time.time()
        for step, (depth, pos, rot, obs_b, map_id) in enumerate(self.train_dataloader):  # obs: body frame
            if depth.shape[0] != self.batch_size:  continue  # batch size == number of env

            self.optimizer.zero_grad()

            trajectory_loss, score_loss, smooth_cost, safety_cost, goal_cost = self.forward_and_compute_loss(depth, pos, rot, obs_b, map_id)

            loss = self.loss_weight[0] * trajectory_loss + self.loss_weight[1] * score_loss

            # Optimize the policy
            loss.backward()
            self.optimizer.step()

            traj_losses.append(self.loss_weight[0] * trajectory_loss.item())
            score_losses.append(self.loss_weight[1] * score_loss.item())
            smooth_losses.append(self.loss_weight[0] * smooth_cost.item())
            safety_losses.append(self.loss_weight[0] * safety_cost.item())
            goal_losses.append(self.loss_weight[0] * goal_cost.item())

            if step % inspect_interval == inspect_interval - 1:
                batch_fps = inspect_interval / (time.time() - start_time)
                self.progress_log.console.log(f"Epoch: {epoch}, Traj Loss: {np.mean(traj_losses):.3g}, "
                                              f"Score Loss: {np.mean(score_losses):.3g} "
                                              f"Batch FPS: {batch_fps:.3g}")
                self.tensorboard_log.add_scalar("Train/TrajLoss", np.mean(traj_losses), epoch * len(self.train_dataloader) + step)
                self.tensorboard_log.add_scalar("Train/ScoreLoss", np.mean(score_losses), epoch * len(self.train_dataloader) + step)
                self.tensorboard_log.add_scalar("Detail/SmoothLoss", np.mean(smooth_losses), epoch * len(self.train_dataloader) + step)
                self.tensorboard_log.add_scalar("Detail/SafetyLoss", np.mean(safety_losses), epoch * len(self.train_dataloader) + step)
                self.tensorboard_log.add_scalar("Detail/GoalLoss", np.mean(goal_losses), epoch * len(self.train_dataloader) + step)
                traj_losses, score_losses, smooth_losses, safety_losses, goal_losses, start_time = [], [], [], [], [], time.time()

            self.progress_log.update(one_epoch_progress, advance=1)
            self.progress_log.update(total_progress, advance=1 / len(self.train_dataloader))

        self.progress_log.remove_task(one_epoch_progress)

    @torch.inference_mode()
    def eval_one_epoch(self, epoch: int):
        one_epoch_progress = self.progress_log.add_task(f"Eval: {epoch}", total=len(self.val_dataloader))
        traj_losses, score_losses = [], []
        for step, (depth, pos, rot, obs_b, map_id) in enumerate(self.val_dataloader):  # obs: body frame
            if depth.shape[0] != self.batch_size:  continue  # batch size == num of env

            trajectory_loss, score_loss, _, _, _ = self.forward_and_compute_loss(depth, pos, rot, obs_b, map_id)

            traj_losses.append(self.loss_weight[0] * trajectory_loss.item())
            score_losses.append(self.loss_weight[1] * score_loss.item())
            self.progress_log.update(one_epoch_progress, advance=1)

        self.progress_log.console.log(f"Eval: {epoch}, Traj Loss: {np.mean(traj_losses):.3g}, Score Loss: {np.mean(score_losses):.3g} ")
        self.tensorboard_log.add_scalar("Eval/TrajLoss", np.mean(traj_losses), epoch)
        self.tensorboard_log.add_scalar("Eval/ScoreLoss", np.mean(score_losses), epoch)
        self.progress_log.remove_task(one_epoch_progress)

    def forward_and_compute_loss(self, depth, pos, rot, obs_b, map_id):
        depth, pos, rot, obs_b, map_id = [x.to(self.device) for x in [depth, pos, rot, obs_b, map_id]]

        # 1. pre-process
        goal_w, start_vel_w, start_acc_w = state_body2world(pos, rot, obs_b[:, 6:9], obs_b[:, 0:3], obs_b[:, 3:6])
        start_state_w = torch.stack([pos, start_vel_w, start_acc_w], dim=1)

        # 2. forward propagation
        endstate, score = self.policy.inference(depth, obs_b)

        # 3. post-process [B, V, H, 9] -> [B*V*H, 9]
        endstate_flat = endstate.permute(0, 2, 3, 1).reshape(self.batch_size * self.traj_num, 9)
        score_flat = score.reshape(self.batch_size * self.traj_num)

        pos_expanded = pos.repeat_interleave(self.traj_num, dim=0)  # [B*V*H, 3]
        rot_expanded = rot.repeat_interleave(self.traj_num, dim=0)  # [B*V*H, 3, 3]
        start_state_w = start_state_w.repeat_interleave(self.traj_num, dim=0)  # [B*V*H, 3, 3]
        goal_w = goal_w.repeat_interleave(self.traj_num, dim=0)  # [B*V*H, 3]

        # [B*V*H, 3] [B*V*H, 3] [B*V*H, 3]
        end_pos_w, end_vel_w, end_acc_w = state_body2world(
            pos_expanded, rot_expanded,
            endstate_flat[:, 0:3],
            endstate_flat[:, 3:6],
            endstate_flat[:, 6:9]
        )
        # [B*V*H, 3, 3]: [px, py, pz; vx, vy, vz; ax, ay, az]
        end_state_w = torch.stack([end_pos_w, end_vel_w, end_acc_w], dim=1)

        smooth_cost, safety_cost, goal_cost = self.yopo_loss(start_state_w, end_state_w, goal_w, map_id)
        trajectory_loss = (smooth_cost + safety_cost + goal_cost).mean()

        score_label = (smooth_cost + safety_cost + goal_cost).clone().detach()
        score_loss = F.smooth_l1_loss(score_flat, score_label)
        return trajectory_loss, score_loss, smooth_cost.mean(), safety_cost.mean(), goal_cost.mean()

    def save_model(self):
        if hasattr(self, "epoch_i"):
            self.progress_log.console.log("Saving model...")
            policy_path = self.tensorboard_path + "/epoch{}.pth".format(self.epoch_i + 1, 0)
            torch.save(self.policy.state_dict(), policy_path)
            atexit.unregister(self._exit_func)

    def get_next_log_path(self, base_path):
        nums = [int(name.split("_")[1])
                for name in os.listdir(base_path)
                if os.path.isdir(os.path.join(base_path, name)) and name.startswith("YOPO_") and name.split("_")[1].isdigit()]
        next_n = max(nums, default=-1) + 1
        next_path = os.path.join(base_path, f"YOPO_{next_n}")
        os.makedirs(next_path, exist_ok=False)
        print("record tensorboard log to ", next_path)
        return next_path
