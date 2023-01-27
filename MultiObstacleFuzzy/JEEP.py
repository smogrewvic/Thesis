import enum

class JEEP(enum.Enum):
    wheelbase = 2.795
    width = 2.00
    steeringRatio = 0.0652778
    maxTireAngle = 45  # degrees
    maxTireAngleRads = maxTireAngle * 0.01745329  # radians