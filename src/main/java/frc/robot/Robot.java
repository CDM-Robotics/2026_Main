// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  // WPILib 2027 replaced SendableChooser/SmartDashboard.getNumber with tunables
  private final Selectable<String> m_chooser = new Selectable<>();
  private final TunableDouble m_steeringBias =
      Tunables.addDouble("Steering Bias", Constants.HardwareConstants.kDefaultGyroBias);

  private final RobotContainer m_rc;

  // REV PDH, read for battery voltage / total current telemetry
  private final PowerDistribution m_pdh = new PowerDistribution(
      Constants.HardwareConstants.kCanBus, Constants.HardwareConstants.kPdhCanId, ModuleType.REV);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.addDefault("DO NOTHING", "HALT");
    m_chooser.add("Right-Side Shoot Only", "RightSideShootOnly");
    m_chooser.add("Center Shoot Only", "CenterShootOnly");
    m_chooser.add("Left-Side Shoot Only", "LeftSideShootOnly");
    m_chooser.add("Square Dance", "SquareDance");

    Tunables.publish("AUTO", m_chooser);

    addPeriodic(this::logPowerTelemetry, Constants.HardwareConstants.kPowerTelemetryPeriod);

    m_rc = new RobotContainer();
    m_rc.subsystemInit();
    m_rc.resetHeading();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** Battery voltage / total current from the PDH; runs at kPowerTelemetryPeriod (1 Hz), not every loop. */
  private void logPowerTelemetry() {
    Telemetry.log("Battery Voltage", m_pdh.getVoltage());
    Telemetry.log("PDH Total Current", m_pdh.getTotalCurrent());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    Alliance alliance = MatchState.getAlliance().orElse(Alliance.BLUE);
    String routine = m_chooser.getSelected();

    if(routine.compareToIgnoreCase("HALT") == 0) {
      return;
    }

    m_rc.scheduleTrajectory(routine);

/*     if (alliance == Alliance.RED) {
      String flippedTraj = routine.concat("Red");
      m_rc.scheduleTrajectory(flippedTraj);
    } else {
      m_rc.scheduleTrajectory(routine);
    }  */

    m_rc.m_DriveTrain.setGyroBias(m_steeringBias.get());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_rc.m_DriveTrain.setGyroBias(m_steeringBias.get());
    m_rc.forceShooterOff();;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void utilityInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void utilityPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
