"""
Steering controller for autobus.

Converts a pixel offset (from sidewalk_detect.analyze_frame) into a servo
steering angle. Deliberately simple (P control + deadband + smoothing) --
get this behaving well before reaching for a full PID.

This module has NO hardware dependency, so it can be unit-tested / run
against recorded video before it ever touches the PCA9685.
"""

from dataclasses import dataclass


@dataclass
class SteeringConfig:
    center_angle: float = 90.0      # servo angle that points wheels straight
    min_angle: float = 60.0         # mechanical limit, right (measured on hardware)
    max_angle: float = 120.0        # mechanical limit, left (measured on hardware)
    gain: float = -0.05              # degrees of correction per pixel of offset
    deadband_px: int = 15           # ignore offsets smaller than this
    smoothing: float = 0.3          # 0 = no smoothing, 1 = ignore new readings
    lost_confidence_threshold: float = 0.3   # below this, treat sidewalk as "lost"
    lost_frame_limit: int = 10      # consecutive lost frames before triggering stop


class SteeringController:
    def __init__(self, config: SteeringConfig = None):
        self.cfg = config or SteeringConfig()
        self._smoothed_offset = 0.0
        self._last_angle = self.cfg.center_angle
        self._lost_streak = 0

    def update(self, offset_px, confidence):
        """
        Call once per frame.

        Returns (angle, should_stop):
            angle       -- servo angle to command, always within safe limits
            should_stop -- True if the sidewalk has been lost for too many
                           consecutive frames and the drive motor should be
                           cut / vehicle should stop, rather than steering
                           on stale or fabricated data.
        """
        cfg = self.cfg

        if confidence < cfg.lost_confidence_threshold or offset_px is None:
            self._lost_streak += 1
            should_stop = self._lost_streak >= cfg.lost_frame_limit
            # Hold last known angle while briefly lost; don't chase noise.
            return self._last_angle, should_stop

        self._lost_streak = 0

        # Exponential smoothing on the raw offset.
        self._smoothed_offset = (
            cfg.smoothing * self._smoothed_offset + (1 - cfg.smoothing) * offset_px
        )

        effective_offset = self._smoothed_offset
        if abs(effective_offset) < cfg.deadband_px:
            effective_offset = 0.0

        angle = cfg.center_angle + effective_offset * cfg.gain
        angle = max(cfg.min_angle, min(cfg.max_angle, angle))

        self._last_angle = angle
        return angle, False

