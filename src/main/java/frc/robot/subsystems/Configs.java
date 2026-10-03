package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberPositionConstants;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.HardwareConstants;

public class Configs {
    public static final class ClimberConfig{
        public static final SparkMaxConfig m_climberConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_climberSlaveConfig = new SparkMaxConfig();

        static {
                m_climberConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ClimberPositionConstants.kRightInverted)
                        .disableFollowerMode()
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                // REVLib 2027: no encoder conversion factor, so the controller works in motor
                // rotations. Gains are scaled from kConversionFactor units (ClimberPosition converts
                // positions/setpoints). kFF is dropped: feedforward kV isn't applied in position mode.
                // uvwAverageDepth/uvwMeasurementPeriod no longer exist in REVLib 2027.
                m_climberConfig.closedLoop
                        .pid(ClimberPositionConstants.kP * ClimberPositionConstants.kConversionFactor,
                             ClimberPositionConstants.kI * ClimberPositionConstants.kConversionFactor,
                             ClimberPositionConstants.kD * ClimberPositionConstants.kConversionFactor)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput);
                m_climberConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor).reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

                m_climberSlaveConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ClimberPositionConstants.kRightInverted)
                        //.follow(HardwareConstants.kClimberRightCanId, false)
                        .disableFollowerMode()
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_climberSlaveConfig.closedLoop
                        .pid((ClimberPositionConstants.kP + 0.015) * ClimberPositionConstants.kConversionFactor,
                             ClimberPositionConstants.kI * ClimberPositionConstants.kConversionFactor,
                             ClimberPositionConstants.kD * ClimberPositionConstants.kConversionFactor)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput);
                m_climberSlaveConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor).reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }
    }

    public static final class FloorIntakeConfig{
        public static final SparkMaxConfig m_elevatorShooterConfig = new SparkMaxConfig();

        static{
                m_elevatorShooterConfig
                        .disableFollowerMode()
                        .idleMode(IdleMode.kCoast)
                        .inverted(FloorIntakeConstants.kCoralHolderInverted)
                        .smartCurrentLimit(80)
                        .voltageCompensation(12.0);
                m_elevatorShooterConfig.encoder
                        .quadratureAverageDepth(2)
                        .quadratureMeasurementPeriod(10);
                m_elevatorShooterConfig.closedLoop
                        //.pid(FloorIntakeConstants.kP, FloorIntakeConstants.kI, FloorIntakeConstants.kD)
                        .pid(FloorIntakeConstants.kP, FloorIntakeConstants.kI, FloorIntakeConstants.kD) // kFF is 0
                        .outputRange(FloorIntakeConstants.kMinOutput, FloorIntakeConstants.kMaxOutput)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // REVLib 2027 has no limit-switch enable flag; "keep moving" is the equivalent of disabled
                m_elevatorShooterConfig.limitSwitch
                        .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                        .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }
    }
}
