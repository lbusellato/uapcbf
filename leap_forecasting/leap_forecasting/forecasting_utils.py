import rclpy
import torch


def uncertainty_heuristic(input: torch.Tensor, output: torch.Tensor, verbose: bool = False) -> torch.Tensor:
    """
    Compute a confidence score (between 0 and 1) for the forecasted final position.
    A score of 1 indicates high confidence (i.e., low uncertainty), while scores near 0 indicate high uncertainty.

    The heuristic considers two penalties:
      - Speed Penalty: If the forecasted displacement (from the last input to the final output)
        is greater than the overall input displacement (from the first to last input), a penalty is incurred.
      - Direction Penalty: When the input trajectory is sufficiently dynamic, we compare the alignment
        of the forecasted displacement to the overall input direction. However, if the forecasted
        displacement is very small relative to the input speed (indicating a stop), we treat it as
        aligned and set the cosine similarity to 1.

    The final confidence score is given by:
        confidence = 1 / (1 + (alpha * speed_penalty + beta * direction_penalty))
    """
    epsilon = 1e-6

    # Compute overall input displacement (from first to last input)
    in_speed = input[1:] - input[:-1]

    # Compute forecasted displacement: from last input to final predicted position
    pred_speed = output[1:] - output[:-1]

    # Speed Penalty: only penalize if the forecasted displacement exceeds the input displacement.
    speed_penalty = torch.abs(pred_speed - in_speed).mean()

    # Compute the variance of the input positions as a measure of dynamism.
    input_dir = input[-1] - input[0]
    output_dir = output[-1] - output[0]

    # compute angle between input and output directions
    cos_sim = torch.dot(input_dir, output_dir) / (torch.norm(input_dir) * torch.norm(output_dir) + epsilon)
    # Map the cosine similarity to a penalty in [0, 1]:
    # With this scaling, perfect alignment (cos_sim = 1) yields 0 penalty,
    # orthogonal movement (cos_sim = 0) yields 0.5 penalty,
    # and opposite movement (cos_sim = -1) yields 1 penalty.
    direction_penalty = 0.5 * (1 - cos_sim)

    # calculate variance of input, if it low, consider the direction penalty as 0
    # to avoid penalizing for small movements
    input_variance = torch.std(input, dim=0).mean()
    if input_variance < 0.05:  # 5 cm
        direction_penalty = 0

    # Combine the two penalties with weights.
    alpha = 0.5  # weight for the speed penalty
    beta = 0.5  # weight for the direction penalty
    combined_penalty = speed_penalty + direction_penalty

    # Convert the penalty into a confidence score in [0, 1]:
    # No penalty (0) yields confidence 1, and increasing penalty reduces confidence.
    # confidence = 1 / (1 + combined_penalty)

    if verbose:
        rclpy.logging.get_logger("forecasting_node").debug(
            f"Speed penalty: {speed_penalty}, Direction penalty: {direction_penalty}"
        )

    return combined_penalty


def uncertainty_covariance_det(covariance: torch.Tensor) -> torch.Tensor:
    """
    Compute a confidence score (between 0 and 1) for the forecasted final position.
    A score of 1 indicates high confidence (i.e., low uncertainty), while scores near 0 indicate high uncertainty.

    The heuristic considers the determinant of the covariance matrix.
    A small determinant indicates a more concentrated distribution, suggesting higher confidence in the prediction.
    """
    # Compute the determinant of the covariance matrix
    det = torch.linalg.det(covariance)

    # Normalize the determinant to be between 0 and 1
    # Assuming that the covariance matrix is positive definite, we can use its eigenvalues
    # to compute a normalized measure of uncertainty.
    eigenvalues = torch.linalg.eigvals(covariance)
    det = torch.prod(eigenvalues).real

    # Confidence score: higher determinant means lower confidence
    # so we invert it to get a score between 0 and 1.
    # A determinant of 0 (perfect certainty) gives a score of 1,
    # while a determinant of 1 (maximum uncertainty) gives a score of 0.
    return 1 - det


def uncertainty_covariance_trace(covariance: torch.Tensor) -> torch.Tensor:
    # Compute the trace (sum of variances) for the first (or only) batch element.
    trace_val = torch.trace(covariance[0])  # cov is of shape [B, 3, 3]

    # # Assume you have determined expected min and max values for the trace, e.g.,
    # min_trace, max_trace = 0.1, 10.0  # These values would come from calibration/validation.

    # # Normalize: when trace_val is near min_trace, we want high confidence (value ~1)
    # # and when near max_trace, we want low confidence (value ~0).
    # overall_certainty = 1 - (trace_val - min_trace) / (max_trace - min_trace)
    # # Clip to [0, 1]:
    # overall_certainty = torch.clamp(overall_certainty, 0, 1)

    alpha = 0.5  # A scaling factor to tune based on your data.
    overall_certainty = torch.exp(-alpha * trace_val)

    return overall_certainty
