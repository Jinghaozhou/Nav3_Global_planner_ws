"""Configuration shared by the tomography and planner algorithms."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class MapConfig:
    resolution: float = 0.10
    ground_h: float = 0.0
    slice_dh: float = 0.5


@dataclass(frozen=True)
class TraversabilityConfig:
    kernel_size: int = 7
    interval_min: float = 0.50
    interval_free: float = 0.65
    slope_max: float = 0.40
    step_max: float = 0.17
    standable_ratio: float = 0.20
    cost_barrier: float = 50.0
    safe_margin: float = 0.4
    inflation: float = 0.2


@dataclass(frozen=True)
class PlannerConfig:
    use_quintic: bool = True
    max_heading_rate: float = 10.0
    start_cost_threshold: float = 20.0
    safe_cost_margin: float = 15.0
    step_cost_weight: float = 0.2


@dataclass(frozen=True)
class PCTConfig:
    map: MapConfig = field(default_factory=MapConfig)
    trav: TraversabilityConfig = field(default_factory=TraversabilityConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)

    def validate(self):
        if self.map.resolution <= 0 or self.map.slice_dh <= 0:
            raise ValueError("map resolution and slice height must be positive")
        if self.trav.kernel_size < 1 or self.trav.kernel_size % 2 != 1:
            raise ValueError("traversability kernel_size must be a positive odd number")
        if not 0 < self.trav.interval_min <= self.trav.interval_free:
            raise ValueError("interval_min and interval_free must be positive and ordered")
        if not 0 <= self.trav.standable_ratio <= 1:
            raise ValueError("standable_ratio must be between 0 and 1")
        if self.trav.safe_margin < 0 or self.trav.inflation < 0:
            raise ValueError("safe_margin and inflation must be nonnegative")
        if self.trav.step_max < 0 or self.trav.slope_max < 0:
            raise ValueError("step_max and slope_max must be nonnegative")
