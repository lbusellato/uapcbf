import torch
import torch.nn as nn


class LeapUncertainty(nn.Module):
    def __init__(self, input_dim: int = 3, hidden_dim: int = 128, num_layers: int = 2, T_out: int = 30, T_in=30, *args, **kwargs):
        super().__init__()
        self.T_in = T_in
        # Encoder LSTM for input sequence
        self.encoder = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        # Decoder LSTM for autoregressive generation
        self.decoder = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        # Final layer now outputs both mean and log-variance (2 * input_dim)
        self.fc = nn.Linear(hidden_dim, input_dim * 2)
        self.T_out = T_out

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """
        x: [B, T_in, 3]
        returns:
          mu:  [B, T_out, 3]   (predicted means)
          logvar: [B, T_out, 3] (predicted log-variances)
        """
        _, (h, c) = self.encoder(x)
        decoder_input = x[:, -1, :].unsqueeze(1)  # [B, 1, 3]
        mus = []
        logvars = []
        for _ in range(self.T_out):
            out, (h, c) = self.decoder(decoder_input, (h, c))
            stats: torch.Tensor = self.fc(out)  # [B, 1, 6]
            mu, logvar = stats.chunk(2, dim=-1)  # each [B, 1, 3]
            mus.append(mu)
            logvars.append(logvar)
            decoder_input = mu  # autoregressive
        mu = torch.cat(mus, dim=1)  # [B, T_out, 3]
        logvar = torch.cat(logvars, dim=1)  # [B, T_out, 3]
        return mu, logvar


def gauss_nll(mu: torch.Tensor, logvar: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """
    Gaussian negative log-likelihood assuming independent per-coordinate Gaussians:
    N(mu, sigma^2), where logvar = log(sigma^2).
    """
    # Compute per-point NLL: 0.5*(logvar + (target - mu)^2 / exp(logvar))
    precision = torch.exp(-logvar)
    nll = 0.5 * (logvar + (target - mu).pow(2) * precision)
    # Sum over coords and timesteps, mean over batch
    return nll.sum(dim=-1).sum(dim=-1).mean()



def __test__():
    xin = torch.randn(32, 30, 3)  # [B, T_in, 3]
    model = LeapUncertainty(input_dim=3, hidden_dim=128, num_layers=2, T_out=30)
    model.eval()
    with torch.no_grad():
        mu, logvar = model(xin)
        print("mu shape:", mu.shape)
        print("logvar shape:", logvar.shape)    
    
    print("Model loaded successfully.")

if __name__ == "__main__":
    __test__()