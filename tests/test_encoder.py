from feetech_hls_rpi.encoder import IncrementalEncoder, movement_magnitude, wrapped_sdk_delta


def test_reverse_zero_crossing():
    assert wrapped_sdk_delta(-660, 349) == -1009


def test_reverse_magnitude_rollover():
    assert wrapped_sdk_delta(-4, -31755) == -1017


def test_forward_boundary_with_direction_hint():
    tracker = IncrementalEncoder(max_step_counts=8192)
    tracker.update(32745, direction_hint=1)
    update = tracker.update(-15, direction_hint=1)
    assert update.delta == 38
    assert update.continuous_counts == 38


def test_forward_observed_stream_never_jumps():
    # Adjacent samples copied from the user's measured ~125 Hz stream around
    # the positive-magnitude to negative-magnitude boundary.
    values = [
        32584,
        32624,
        32664,
        32705,
        32745,
        -15,
        -55,
        -93,
        -133,
        -173,
    ]
    tracker = IncrementalEncoder(max_step_counts=8192)
    previous_total = 0
    for value in values:
        update = tracker.update(value, direction_hint=1)
        assert update.valid
        assert update.continuous_counts >= previous_total
        assert 0 <= update.delta < 100
        previous_total = update.continuous_counts


def test_reverse_observed_boundaries_never_jump():
    # Two separate observed reverse-wheel boundaries:
    #   349 -> -660 is the zero/sign transition.
    #   -31755 -> -4 is the 15-bit magnitude rollover.
    tracker = IncrementalEncoder(max_step_counts=8192)
    tracker.update(349, direction_hint=-1)
    first = tracker.update(-660, direction_hint=-1)
    assert first.delta == -1009

    tracker.reset()
    tracker.update(-31755, direction_hint=-1)
    second = tracker.update(-4, direction_hint=-1)
    assert second.delta == -1017


def test_movement_magnitude_examples():
    assert movement_magnitude(-4, -31755) == 1017
    assert movement_magnitude(-660, 349) == 1009
    assert movement_magnitude(-15, 32745) == 38
