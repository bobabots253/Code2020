{
    # Class names of motor controllers used.
    # Options:
    # 'Spark'
    # 'Victor'
    # 'VictorSP'
    # 'PWMTalonSRX'
    # 'PWMVictorSPX'
    # 'WPI_TalonSRX'
    # 'WPI_VictorSPX'
    "controllerTypes": ["WPI_TalonSRX"],
    # Ports for the motors
    "motorPorts": [5],
    # Inversions for the motors
    "motorsInverted": [False],
    # Unit of analysis
    # Options:
    # 'Degrees'
    # 'Radians'
    # 'Rotations'
    "units": "Radians",
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the arm*, and so
    # should take into account gearing between the encoder and the arm
    "encoderEPR": 8192,
    # Ports for the encoder
    "encoderPorts": (2, 4),
    # Whether the encoder is inverted
    "encoderInverted": True,
    # Offset of your encoder zero from horizontal
    "offset": -0.8203047,
}

