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
                m_climberConfig.closedLoop
                        .pidf(ClimberPositionConstants.kP, ClimberPositionConstants.kI, ClimberPositionConstants.kD, ClimberPositionConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput);
                m_climberConfig.encoder
                        .uvwAverageDepth(2)
                        .uvwMeasurementPeriod(10)
                        .positionConversionFactor(ClimberPositionConstants.kConversionFactor);
                m_climberConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor).reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

                m_climberSlaveConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ClimberPositionConstants.kRightInverted)
                        //.follow(HardwareConstants.kClimberRightCanId, false)
                        .disableFollowerMode()
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_climberSlaveConfig.closedLoop
                        .pidf(ClimberPositionConstants.kP + 0.015, ClimberPositionConstants.kI, ClimberPositionConstants.kD, ClimberPositionConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput);
                m_climberSlaveConfig.encoder
                        .uvwAverageDepth(2)
                        .uvwMeasurementPeriod(10)
                        .positionConversionFactor(ClimberPositionConstants.kConversionFactor);
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
                        .pidf(FloorIntakeConstants.kP, FloorIntakeConstants.kI, FloorIntakeConstants.kD, FloorIntakeConstants.kFF)
                        .outputRange(FloorIntakeConstants.kMinOutput, FloorIntakeConstants.kMaxOutput)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                m_elevatorShooterConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }
}
