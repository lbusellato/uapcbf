import torch
import torch.nn as nn
import torch.nn.functional as F


class RNNWRotation(nn.Module):
    def __init__(self, input_dim=3, quat_dim=4, hidden_dim=64, T_in=30, T_out=30, num_layers=1):
        super(RNNWRotation, self).__init__()

        # Separate encoders
        self.gru_pos_enc = nn.GRU(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )
        self.gru_quat_enc = nn.GRU(
            input_size=quat_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )

        # Separate decoders
        self.gru_pos_dec = nn.GRU(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )
        self.gru_quat_dec = nn.GRU(
            input_size=quat_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )

        self.fc_pos = nn.Linear(hidden_dim, 3)
        self.fc_quat = nn.Linear(hidden_dim, 4)

        self.T_in = T_in
        self.T_out = T_out

    def _decode(self, x, q):
        """
        Internal helper for decoding.

        Args:
            x: Tensor of shape [B, T, input_dim] (positions).
            q: Tensor of shape [B, T, quat_dim] (quaternions).

        Returns:
            predictions_pos: Tensor of shape [B, T_out, 3]
            predictions_quat: Tensor of shape [B, T_out, 4]
        """
        B, T, _ = x.shape

        # Encode input sequences.
        _, hidden_pos = self.gru_pos_enc(x)
        _, hidden_quat = self.gru_quat_enc(q)

        # Use the final layer's hidden state.
        hidden_pos = hidden_pos[-1].unsqueeze(0)
        hidden_quat = hidden_quat[-1].unsqueeze(0)

        predictions_pos, predictions_quat = [], []

        # Use the last time step of the input sequence as the first input for decoding.
        current_input_pos = x[:, -1, :].unsqueeze(1)  # shape: [B, 1, input_dim]
        current_input_quat = q[:, -1, :].unsqueeze(1)  # shape: [B, 1, quat_dim]

        # Decoding loop.
        for _ in range(self.T_out):
            out_pos, hidden_pos = self.gru_pos_dec(current_input_pos, hidden_pos)
            out_quat, hidden_quat = self.gru_quat_dec(current_input_quat, hidden_quat)

            pos_out = self.fc_pos(out_pos.squeeze(1))  # [B, 3]
            quat_out = F.normalize(self.fc_quat(out_quat.squeeze(1)), p=2, dim=1)  # [B, 4]

            predictions_pos.append(pos_out.unsqueeze(1))
            predictions_quat.append(quat_out.unsqueeze(1))

            # Next input is the prediction.
            current_input_pos = pos_out.unsqueeze(1)
            current_input_quat = quat_out.unsqueeze(1)

        predictions_pos = torch.cat(predictions_pos, dim=1)  # [B, T_out, 3]
        predictions_quat = torch.cat(predictions_quat, dim=1)  # [B, T_out, 4]
        return predictions_pos, predictions_quat

    def forward(self, x, q):
        """
        Standard forward pass that returns predicted positions and quaternions.

        Args:
            x: Tensor of shape [B, T, input_dim] for positions.
            q: Tensor of shape [B, T, quat_dim] for quaternions.

        Returns:
            predictions_pos: Tensor of shape [B, T_out, 3]
            predictions_quat: Tensor of shape [B, T_out, 4]
        """
        return self._decode(x, q)

    def forward_with_covariance(self, x, q):
        """
        Forward pass that returns the predictions and computes a 3D covariance directly
        from the T_out predicted 3D positions.

        Args:
            x: Tensor of shape [B, T, input_dim] for positions.
            q: Tensor of shape [B, T, quat_dim] for quaternions.

        Returns:
            predictions_pos: Tensor of shape [B, T_out, 3]
            predictions_quat: Tensor of shape [B, T_out, 4]
            covariances: Tensor of shape [B, 3, 3] computed from the T_out predicted points.
        """
        pred_pos, pred_quat = self.forward(x, q)
        B, T_out, _ = pred_pos.shape
        covariances = []
        for i in range(B):
            pts = pred_pos[i]  # [T_out, 3]
            mean = pts.mean(dim=0, keepdim=True)  # [1, 3]
            centered = pts - mean
            cov = torch.matmul(centered.t(), centered) / (T_out - 1)  # [3, 3]
            covariances.append(cov)
        covariances = torch.stack(covariances, dim=0)  # [B, 3, 3]
        return pred_pos, pred_quat, covariances
