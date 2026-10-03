package frc.robot;

import org.tritontech.core.*;

import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.controller.PIDController;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.MAXSwerveModule;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Hopper;

public class RobotContainer {
    Vision myVision = null;


    SwerveModule frontLeft = new SwerveModule(HardwareConstants.kCanBus,
                                              HardwareConstants.kFrontLeftDrivingCanId, 
                                              MotorControllerType.SPARK_FLEX, 
                                              MotorControllerType.SPARK_FLEX, 
                                              HardwareConstants.kFrontLeftTurningCanId, 
                                              DriveConstants.kFrontLeftChassisAngularOffset, 
                                              "FrontLeft", 
                                              MAXSwerveModule.drivingConfig, 
                                              MAXSwerveModule.turningConfig, 
                                              new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA),
                                              ModuleConstants.kDrivingEncoderPositionFactor);

    SwerveModule frontRight = new SwerveModule(HardwareConstants.kCanBus,
                                              HardwareConstants.kFrontRightDrivingCanId, 
                                              MotorControllerType.SPARK_FLEX, 
                                              MotorControllerType.SPARK_FLEX, 
                                              HardwareConstants.kFrontRightTurningCanId, 
                                              DriveConstants.kFrontRightChassisAngularOffset, 
                                              "FrontRight", 
                                              MAXSwerveModule.drivingConfig, 
                                              MAXSwerveModule.turningConfig, 
                                              new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA),
                                              ModuleConstants.kDrivingEncoderPositionFactor);

    SwerveModule rearLeft = new SwerveModule(HardwareConstants.kCanBus,
                                              HardwareConstants.kRearLeftDrivingCanId, 
                                             MotorControllerType.SPARK_FLEX, 
                                             MotorControllerType.SPARK_FLEX, 
                                             HardwareConstants.kRearLeftTurningCanId, 
                                             DriveConstants.kRearLeftChassisAngularOffset, 
                                             "RearLeft", 
                                             MAXSwerveModule.drivingConfig, 
                                             MAXSwerveModule.turningConfig, 
                                             new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA),
                                              ModuleConstants.kDrivingEncoderPositionFactor);
    SwerveModule rearRight = new SwerveModule(HardwareConstants.kCanBus,
                                              HardwareConstants.kRearRightDrivingCanId, 
                                              MotorControllerType.SPARK_FLEX, 
                                              MotorControllerType.SPARK_FLEX, 
                                              HardwareConstants.kRearRightTurningCanId, 
                                              DriveConstants.kRearRightChassisAngularOffset, 
                                              "RearRight", 
                                              MAXSwerveModule.drivingConfig, 
                                              MAXSwerveModule.turningConfig, 
                                              new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA),
                                              ModuleConstants.kDrivingEncoderPositionFactor);  
                                                                        
    public final DriveTrain m_DriveTrain = new DriveTrain(frontLeft,
                                             frontRight,
                                             rearLeft,
                                             rearRight,
                                             DriveConstants.kDriveKinematics,
                                             new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate),
                                             new SlewRateLimiter(DriveConstants.kRotationalSlewRate),
                                             myVision,
                                             null,
                                             HardwareConstants.kImuMountOrientation);

    CommandXboxController m_driverController; // Initialized by DriveTrain
    CommandXboxController m_engineerController = new CommandXboxController(OIConstants.kEngineerControllerPort);

    public final FloorIntake floorIntake = new FloorIntake();
    public final Hopper hopper = new Hopper();

    public RobotContainer() {
        m_DriveTrain.setGyroInverted(HardwareConstants.kGyroInverted);

        m_DriveTrain.setChassisConstants(DriveConstants.kTrackWidth, 
                                         HardwareConstants.kBumperDistance);

        m_DriveTrain.setDriveConstants(DriveConstants.kDirectionSlewRate, 
                                       DriveConstants.kMaxSpeedMetersPerSecond, 
                                       DriveConstants.kMaxAngularSpeed);

        m_DriveTrain.setVisionConstants(VisionConstants.kTagLayout, VisionConstants.kDistanceCorrection);

        m_driverController = m_DriveTrain.getDefaultDriveController(OIConstants.kDriverControllerPort, OIConstants.kSuperSlow, OIConstants.kMildSlow);

        configureButtionBindings();
    }

    private void configureButtionBindings() {
        // The DriveTrain comes with the following buttons pre-defined, override here, if you wish
        // view().onTrue -- resets the throttle (was back() before WPILib 2027)
        // rightBumper().onTrue -- sets throttle factor to bumperFactor
        // rightBumper().onFalse -- resets throttle factor to 1.0
        // rightTrigger().onTrue -- sets throttle factor to triggerFactor
        // rightTrinner().onFalse -- resets throttle factor to 1.0

        // Floor Intake
        m_engineerController.x().onTrue(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(-2000)).withTimeout(1.0) , new InstantCommand(() -> hopper.setSetpoint(-10000) )));
        m_engineerController.x().onFalse(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(0)).withTimeout(1.0), new InstantCommand(() -> hopper.setSetpoint(0))));

        // Dump out load Intake
        m_engineerController.b().onTrue(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(2000)).withTimeout(1.0) , new InstantCommand(() -> hopper.setSetpoint(10000) )));
        m_engineerController.b().onFalse(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(0)).withTimeout(1.0), new InstantCommand(() -> hopper.setSetpoint(0))));

        // Shoot
        m_engineerController.y().onTrue(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(-4000)).withTimeout(1.0), new InstantCommand(() -> hopper.setSetpoint(4000))));
        m_engineerController.y().onFalse(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(0)).withTimeout(1.0), new InstantCommand(() -> hopper.setSetpoint(0))));

        // Dump to human player
        //m_engineerController.b().onTrue(null);

        // Shoot
        //m_engineerController.y().onTrue(null);
    }

    public void resetHeading() {
        m_DriveTrain.zeroHeading();
    }

    public void subsystemInit() {

    }

    public void setResetInitialPose(Pose2d pose) {
        m_DriveTrain.hardResetPose(pose);
        m_DriveTrain.resetMeasuredOdometry(pose);
    }

    public void scheduleTrajectory(String trajectory) {
        Command traj = m_DriveTrain.buildTrajectory(trajectory, new PIDController(6.0, 0.0, 0.3), true);
        SequentialCommandGroup scg = null;
        if(traj != null) {
            if (trajectory.contains("Shoot")){
             scg = new SequentialCommandGroup(
                    //    new WaitCommand(1.0), 
                        traj, 
                        new InstantCommand(() -> m_DriveTrain.stopModules()),
                        new InstantCommand(() -> floorIntake.setSetpoint(-4000)).withTimeout(1.0) , 
                        new InstantCommand(() -> hopper.setSetpoint(4000) ).withTimeout(1.0),
                        new WaitCommand(5.0),
                        new InstantCommand(() -> floorIntake.setSetpoint(0)).withTimeout(1.0), 
                        new InstantCommand(() -> hopper.setSetpoint(0)).withTimeout(1.0) );
            } else {
               scg = new SequentialCommandGroup(new WaitCommand(1.0), traj); 
            }
            if (scg != null){
                CommandScheduler.getInstance().schedule(scg);
            }
            
        }
    }

    public void forceShooterOff() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new InstantCommand(() -> floorIntake.setSetpoint(0)).withTimeout(1.0), new InstantCommand(() -> hopper.setSetpoint(0))));
    }
}
