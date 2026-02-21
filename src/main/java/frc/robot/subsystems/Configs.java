package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberPositionConstants;
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
}
