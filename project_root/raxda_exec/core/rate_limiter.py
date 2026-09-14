def rate_limit(
    value: float,
    last_value: float,
    max_delta: float,
) -> float:
    delta = value - last_value
    if abs(delta) > max_delta:
        return last_value + max_delta * (1 if delta > 0 else -1)
    return value