from dataclasses import dataclass


@dataclass
class GcThresholds:
    """Thresholds for the garbage collector."""

    gen_0: int
    gen_1: int
    gen_2: int


@dataclass
class GcConfig:
    disable: bool
    show_stats: bool
    """Enable or disable the garbage collector."""
    thresholds: GcThresholds
