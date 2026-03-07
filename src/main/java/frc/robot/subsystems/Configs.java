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
        public static final SparkMaxConfig m_climberRightConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_climberLeftConfig = new SparkMaxConfig();

        static {
            m_climberRightConfig
                .idleMode(IdleMode.kBrake)
                .inverted(ClimberPositionConstants.kInverted)
                .disableFollowerMode()
                .voltageCompensation(12.0)
                .smartCurrentLimit(40);

            m_climberRightConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput)
                .pid(
                    ClimberPositionConstants.kP,
                    ClimberPositionConstants.kI,
                    ClimberPositionConstants.kD
                )
                .feedForward.kS(ClimberPositionConstants.kS);
                        /* .kG(ClimberPositionConstants.kG)
                        .kV(ClimberPositionConstants.kV); */
                

            m_climberRightConfig.encoder
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(10)
                .positionConversionFactor(ClimberPositionConstants.kConversionFactor);
            m_climberRightConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor).reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

            m_climberLeftConfig
                .idleMode(IdleMode.kBrake)
                .inverted(ClimberPositionConstants.kInverted)
                .follow(HardwareConstants.kClimberRightCanId)
                .voltageCompensation(12.0)
                .smartCurrentLimit(40);

            m_climberLeftConfig.closedLoop
                .pid(
                    ClimberPositionConstants.kP,
                    ClimberPositionConstants.kI,
                    ClimberPositionConstants.kD
                )
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput)
                .pid(
                    ClimberPositionConstants.kP,
                    ClimberPositionConstants.kI,
                    ClimberPositionConstants.kD
                )
                .feedForward.kS(ClimberPositionConstants.kS);
                        /* .kG(ClimberPositionConstants.kG)
                        .kV(ClimberPositionConstants.kV); */

            m_climberLeftConfig.encoder
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(10)
                .positionConversionFactor(ClimberPositionConstants.kConversionFactor);
            m_climberLeftConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor).reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

        }       
    }

    public static final class FloorIntakeConfig{
        public static final SparkMaxConfig m_elevatorShooterConfig = new SparkMaxConfig();

        static{
                m_elevatorShooterConfig
                        .disableFollowerMode()
                        .idleMode(IdleMode.kBrake)
                        .inverted(FloorIntakeConstants.kCoralHolderInverted)
                        .smartCurrentLimit(80)
                        .voltageCompensation(12.0);
                m_elevatorShooterConfig.encoder
                        .quadratureAverageDepth(2)
                        .quadratureMeasurementPeriod(10);
                m_elevatorShooterConfig.closedLoop
                        .pidf(FloorIntakeConstants.kP, FloorIntakeConstants.kI, FloorIntakeConstants.kD, FloorIntakeConstants.kFF)
                        .outputRange(FloorIntakeConstants.kMinOutput, FloorIntakeConstants.kMaxOutput)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                m_elevatorShooterConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }
}
