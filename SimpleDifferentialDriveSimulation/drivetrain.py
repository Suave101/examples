import math
import wpimath.controller
import wpimath.kinematics
import wpimath.system.plant
import wpilib.simulation
import wpilib
import ntcore


class Drivetrain:
    def __init__(self):
        # 3 meters per second
        kMaxSpeed = 3
        # 1/2 rotation per second.
        kMaxAngularSpeed = math.pi

        kTrackWidth = 0.381 * 2
        kWheelRadius = 0.0508
        kEncoderResolution = -4096

        self.leftLeader = wpilib.PWMSparkMax(1)
        self.leftFollower = wpilib.PWMSparkMax(2)
        self.rightLeader = wpilib.PWMSparkMax(3)
        self.rightFollower = wpilib.PWMSparkMax(4)

        self.leftEncoder = wpilib.Encoder(0, 1)
        self.rightEncoder = wpilib.Encoder(2, 3)

        self.leftPIDController = wpimath.controller.PIDController(8.5, 0, 0)
        self.rightPIDController = wpimath.controller.PIDController(8.5, 0, 0)

        self.gyro = wpilib.AnalogGyro(0)

        self.kinematics = wpimath.kinematics.DifferentialDriveKinematics(kTrackWidth)
        self.odometry = wpimath.kinematics.DifferentialDriveOdometry(self.gyro.getRotation2d(),
                                                                     self.leftEncoder.getDistance(),
                                                                     self.rightEncoder.getDistance())

        # Gains are for example purposes only - must be determined for your own
        # robot!
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)

        # Simulation classes help us simulate our robot
        self.gyroSim = wpilib.simulation.AnalogGyroSim(self.gyro)
        self.leftEncoderSim = wpilib.simulation.EncoderSim(self.leftEncoder)
        self.rightEncoderSim = wpilib.simulation.EncoderSim(self.rightEncoder)
        self.fieldSim = wpilib.Field2d()
        self.drivetrainSystem = wpimath.system.plant.LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
        self.drivetrainSimulator = wpilib.simulation.DifferentialDrivetrainSim(
            self.drivetrainSystem,
            wpimath.system.plant.DCMotor.CIM(2),
            8,
            kTrackWidth,
            kWheelRadius,
            None
        )

    # Subsystem constructor
    def Drivetrain(self):
        self.leftLeader
