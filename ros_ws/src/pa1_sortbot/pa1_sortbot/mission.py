"""Task data kept independent of ROS for simple unit testing."""

ITEMS = (
    ("sample_1", -0.72, -0.65, -1.00, -0.65),
    ("sample_2", 0.0, -0.65, -0.28, -0.65),
    ("sample_3", 0.72, -0.65, 0.44, -0.65),
)

BIN_POSES = {
    "paper": (1.25, 0.72),
    "plastic": (1.25, 0.0),
    "metal": (1.25, -0.72),
}

BIN_APPROACH_OFFSET = 0.43
CATEGORY_NAMES = frozenset(BIN_POSES)
