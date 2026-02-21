package frc.robot;

import org.tritontech.core.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.MAXSwerveModule;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.GlobalVariables.ClimberMode;
import frc.robot.commands.ReadyClimberPosition;
import frc.robot.subsystems.ClimberPosition;

public class RobotContainer {
    Vision myVision = null;

    public final GlobalVariables m_variables = new GlobalVariables();

    final ClimberPosition climber = new ClimberPosition();

    SwerveModule frontLeft = new SwerveModule(HardwareConstants.kFrontLeftDrivingCanId, 
                                              MotorControllerType.SPARK_FLEX, 
                                              MotorControllerType.SPARK_FLEX, 
                                              HardwareConstants.kFrontLeftTurningCanId, 
                                              DriveConstants.kFrontLeftChassisAngularOffset, 
                                              "FrontLeft", 
                                              MAXSwerveModule.drivingConfig, 
                                              MAXSwerveModule.turningConfig, 
                                              new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA));

    SwerveModule frontRight = new SwerveModule(HardwareConstants.kFrontRightDrivingCanId, 
                                              MotorControllerType.SPARK_FLEX, 
                                              MotorControllerType.SPARK_FLEX, 
                                              HardwareConstants.kFrontRightTurningCanId, 
                                              DriveConstants.kFrontRightChassisAngularOffset, 
                                              "FrontRight", 
                                              MAXSwerveModule.drivingConfig, 
                                              MAXSwerveModule.turningConfig, 
                                              new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA));

    SwerveModule rearLeft = new SwerveModule(HardwareConstants.kRearLeftDrivingCanId, 
                                             MotorControllerType.SPARK_FLEX, 
                                             MotorControllerType.SPARK_FLEX, 
                                             HardwareConstants.kRearLeftTurningCanId, 
                                             DriveConstants.kRearLeftChassisAngularOffset, 
                                             "RearLeft", 
                                             MAXSwerveModule.drivingConfig, 
                                             MAXSwerveModule.turningConfig, 
                                             new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA));
    SwerveModule rearRight = new SwerveModule(HardwareConstants.kRearRightDrivingCanId, 
                                              MotorControllerType.SPARK_FLEX, 
                                              MotorControllerType.SPARK_FLEX, 
                                              HardwareConstants.kRearRightTurningCanId, 
                                              DriveConstants.kRearRightChassisAngularOffset, 
                                              "RearRight", 
                                              MAXSwerveModule.drivingConfig, 
                                              MAXSwerveModule.turningConfig, 
                                              new SimpleMotorFeedforward(ModuleConstants.kDrivingS, 
                                                                        ModuleConstants.kDrivingV, 
                                                                        ModuleConstants.kDrivingA));  
                                                                        
    public final DriveTrain m_DriveTrain = new DriveTrain(frontLeft,
                                             frontRight,
                                             rearLeft,
                                             rearRight,
                                             DriveConstants.kDriveKinematics,
                                             new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate),
                                             new SlewRateLimiter(DriveConstants.kRotationalSlewRate),
                                             myVision,
                                             null);

    CommandXboxController m_driverController; // Initialized by DriveTrain
    CommandXboxController m_engineerController = new CommandXboxController(OIConstants.kEngineerControllerPort);

    public RobotContainer() {
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
        // back().onTrue -- resets the throttle
        // rightBumper().onTrue -- sets throttle factor to bumperFactor
        // rightBumper().onFalse -- resets throttle factor to 1.0
        // rightTrigger().onTrue -- sets throttle factor to triggerFactor
        // rightTrinner().onFalse -- resets throttle factor to 1.0

        // Floor Intake
        m_engineerController.x().onTrue(null);

        // Dump to human player
        m_engineerController.b().onTrue(null);

        // Shoot
        m_engineerController.y().onTrue(null);

        // Climber to Home position
        m_engineerController.rightTrigger().onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> m_variables.setClimberMode(ClimberMode.HOME)),
                        new ReadyClimberPosition(m_variables, climber)));

        // Climber to full extension
        m_engineerController.rightBumper().onTrue(null);

        // Climber stop
        m_engineerController.a().onTrue(null);
    }
}
