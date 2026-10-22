"""
Tactical Maneuver State Enumeration.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

from enum import IntEnum


class ManeuverState(IntEnum):
    """
    8 Tactical Driving Decisions for Autonomous Vehicles.
    """
    ACCELERATE = 1
    BRAKE = 2
    OVERTAKE = 3
    WAIT = 4
    STOP = 5
    EMERGENCY_STOP = 6
    MOVE_LEFT = 7
    MOVE_RIGHT = 8

    @property
    def readable_name(self) -> str:
        names = {
            ManeuverState.ACCELERATE: "Accelerate",
            ManeuverState.BRAKE: "Brake",
            ManeuverState.OVERTAKE: "Overtake",
            ManeuverState.WAIT: "Wait",
            ManeuverState.STOP: "Stop",
            ManeuverState.EMERGENCY_STOP: "Emergency Stop",
            ManeuverState.MOVE_LEFT: "Move Left",
            ManeuverState.MOVE_RIGHT: "Move Right",
        }
        return names.get(self, "Unknown")

    @property
    def is_lateral(self) -> bool:
        return self in (ManeuverState.OVERTAKE, ManeuverState.MOVE_LEFT, ManeuverState.MOVE_RIGHT)

    @property
    def is_braking(self) -> bool:
        return self in (ManeuverState.BRAKE, ManeuverState.WAIT, ManeuverState.STOP, ManeuverState.EMERGENCY_STOP)
