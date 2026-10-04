package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.units.Unit;

public class Constants {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            // REVLib 2027 has no encoder conversion factors: the driving encoder reports motor
            // rotations / RPM and TritonTechCore's SwerveModule converts using
            // ModuleConstants.kDrivingEncoderPositionFactor.
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
            drivingConfig.closedLoop.feedForward.kV(ModuleConstants.kDrivingFF);
            drivingConfig.voltageCompensation(12.0);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(ModuleConstants.kTurningEncoderInverted); // reports rotations; SwerveModule converts to radians
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kTurningP,ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route. (REVLib 2027 dropped the input range setting; it wraps over the
                    // absolute encoder's native 0-1 rotation.)
                    .positionWrappingEnabled(true);
            turningConfig.voltageCompensation(12.0);
        }
    }
    
    public static final class HardwareConstants{
        // SystemCore has several CAN buses; everything on this robot is on the first one
        public static final CANPort kCanBus = CANPort.CAN_S0;

        // SystemCore's built-in IMU replaces the NavX. Only the mounting plane matters for yaw:
        // FLAT = lying flat, LANDSCAPE = standing on its long edge, PORTRAIT = on its short edge.
        // Check on the robot: the dashboard "Angle" must INCREASE when the robot turns
        // counter-clockwise (seen from above). If it decreases (e.g. SystemCore mounted label-down),
        // set kGyroInverted = true.
        public static final MountOrientation kImuMountOrientation = MountOrientation.FLAT;
        public static final boolean kGyroInverted = false;

        // SPARK MAX CAN IDs 
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kFrontLeftTurningCanId = 2;
        public static final int kFrontRightDrivingCanId =  5;
        public static final int kFrontRightTurningCanId = 4;
        public static final int kRearLeftDrivingCanId = 9;
        public static final int kRearLeftTurningCanId = 8;
        public static final int kRearRightDrivingCanId = 7;
        public static final int kRearRightTurningCanId = 6;

        public static final int kFloorIntakeCanId = 12;
        // REV Power Distribution Hub (1 is REV's factory default CAN ID)
        public static final int kPdhCanId = 1;
        public static final int kHopperCanId = 13;
    
        public static final double kBumperDistance = Units.inchesToMeters(16.0);
        public static final double kDefaultGyroBias = 0.0035;  // Was 0.0025
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 1.8;  // Was 5.6
        public static final double kMaxAngularSpeed = 0.4 * Math.PI; // Was 2 radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 1.8; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kRearLeftChassisAngularOffset = Math.PI;
        public static final double kRearRightChassisAngularOffset = Math.PI / 2;   

        public static final boolean kGyroReversed = false;

        public static final double kAutoAlignMaxAccel = 6;
        public static final double kAutoAlignMaxVelo = 1;
        
        public static final double kAutoAlignP = 1;
        public static final double kAutoAlignI = 0.0;
        public static final double kAutoAlignD = 0.0;

        public static final double kAutoAlignS = 0.2;
        public static final double kAutoAlignV = 0.2;
        public static final double kAutoAlignA = 0.00;

        public static final double kAlignVelocityMod = 0.018;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        // This accounts for stuff such as wheel wear
        public static final double kXFactor = 1.0;  // if actual is smaller than odo go down  

        public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) * kXFactor; // meters
        public static final double kDrivingEncoderVelocityFactor = (((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0) * kXFactor; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        // REV closed-loop gains act on native units in REVLib 2027 (RPM for driving, rotations for
        // turning) instead of the converted m/s and radians used in 2026. The 2026-tuned values are
        // kept as the first factor and rescaled so the controllers behave the same.
        public static final double kDrivingP = 0.17 * kDrivingEncoderVelocityFactor; // 2026: 0.17 per m/s
        public static final double kDrivingI = 0.0;
        public static final double kDrivingD = 0.0;
        // 2026 velocityFF was 0.21 duty cycle per m/s; feedForward.kV is volts per RPM (12 V nominal)
        public static final double kDrivingFF = 0.21 * 12.0 * kDrivingEncoderVelocityFactor;

        public static final double kDrivingA = .4;
        public static final double kDrivingS = 0.17;//0.2;
        public static final double kDrivingV = 2.28;
        
        // public static final double kDrivingA = 0.44218;
        // public static final double kDrivingS = 0.17491;
        // public static final double kDrivingV = 2.7538;

        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1.5 * kTurningEncoderPositionFactor; // 2026: 1.5 per radian
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        public static final double kPAcceleration = 0.005;

    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
    
    public static final class OIConstants {
        public static final int kdriverEngineerPort = 0;
        public static final int kDriverControllerPort = 0;
        public static final int kEngineerControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
        public static final double kSuperSlow = 0.05;
        public static final double kMildSlow = 0.50;
    }

 



    public static final class VisionConstants {
        public static final double kDistanceCorrection = Units.inchesToMeters(-6.0);

        // Primary Camera
        public static final double kCameraZOffset = 17.0 + 5.0/8.0; // Todo - update based on robot measurements
        public static final double kAlignmentTargetDistance = 48; // Todo - update based on measurements
        public static final double kVerticalBullseyeMeasurement = 20.25; // as measured from the robot on a flat surface
        public static final double kPitchOffset = Math.atan((kVerticalBullseyeMeasurement - kCameraZOffset) / kAlignmentTargetDistance);
        public static final double kHorizontalBullseyeMeasurement = -1.0;
        public static final double kYawOffset = Math.atan(kHorizontalBullseyeMeasurement / kAlignmentTargetDistance);
        public static final String kPhoton = "photonvision";
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(14.5 - (3 + 9/64)), Units.inchesToMeters(14.5 - (2 + 5/32)), Units.inchesToMeters(kCameraZOffset)), new Rotation3d(0, 0, 0)); // X, Y, Z, Rotation
        
        // Auxiallary Camera
        public static final double kAuxCameraZOffset = 15.0 + 7.0/8.0;
        public static final double kAuxAlignmentTargetDistance = 48;
        public static final double kAuxVerticalBullseyeMeasurement = 20.25; // as measured from the robot on a flat surface
        public static final double kAuxPitchOffset = Math.atan((kAuxVerticalBullseyeMeasurement - kAuxCameraZOffset) / kAuxAlignmentTargetDistance);
        public static final double kAuxHorizontalBullseyeMeasurement = -1.0;
        public static final double kAuxYawOffset = Math.atan(kAuxHorizontalBullseyeMeasurement / kAuxAlignmentTargetDistance);
        public static final String kAuxPhoton = "Aux Camera";
        public static final Transform3d kAuxRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(14.5 - (6 + 3/4)), Units.inchesToMeters(14.5 - (10 + 1/2)), Units.inchesToMeters(kAuxCameraZOffset)), new Rotation3d(0, 0, Units.degreesToRadians(90))); // X, Y, Z, Rotation
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        // TODO - Tune the value to be more accurate.  The X-value was off, so i set it to zero (should be 5.0 inches)...
        
        // The layout of the AprilTags on the field
        /*private static AprilTag testTag = new AprilTag(9, new Pose3d(0.0, Units.inchesToMeters(36.0), Units.inchesToMeters(14.0), Rotation3d.kZero));
        private static List<AprilTag> testTags = new ArrayList<AprilTag>();
        public static AprilTagFieldLayout kTagLayout;
        static {
            testTags.add(testTag);
            kTagLayout = new AprilTagFieldLayout(testTags, 2.0, 2.0);
        }*/

        public static final Field kTagLayout = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2.5, 2.5, 5);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.06, 0.06, .12);
 
    }

    public static final class AutoConstants {
        public static final String kDefaultAuto = "Taxi";
        public static final String kCustomAuto = "Score";

        public static final double kMaxSpeedMetersPerSecond = 0.2;// 0.1
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;// 0.1
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 4.0; // 6.0
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4.0;  //6.0

        public static final String kLeftAuto = "Left Auto";
        public static final String kRightAuto = "Right Auto";
        public static final String kCenterAuto = "Center Auto";

        // public static final double kPXController = 1;
        // public static final double kPYController = 1;
        // public static final double kPThetaController = 1;

        public static final double kPXController = 1.5;// 1.0
        public static final double kPYController = 1.5;// 1.0
        public static final double kPThetaController = 1.0;// 0.75
        
        
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class FloorIntakeConstants{
        // NEO current limit for the floor intake and hopper (they share Configs.FloorIntakeConfig).
        // 40 A matches a 40 A breaker channel and protects the NEO if a roller stalls/jams.
        public static final int kCurrentLimit = 40; // amps
        public static final double kP = 0.00;  // Originally 0.0 --> 0.00165
        public static final double kPReverse = 0.0002;  // Originally 0.0002
        public static final double kI = 0.0;  // Originally 0.0
        public static final double kD = 0.00;  // Originally 0.00 -- 0.00075
        public static final double kFF = 0.0;
        public static final double kV = 0.00205;  // Origianlly 0.00205
        public static final double kS = .12;   // Origianlly 0.12
        public static final boolean kCoralHolderInverted = true;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
        public static final double kIntake = 3000;
        public static final double kOff = 0;
        public static final double kReverse = -1000;//-2500
        public static final double kReverseSlow = -1000;
        public static final double kHolding = 50;
        public static final double kShoot = 6000;
    }
}
