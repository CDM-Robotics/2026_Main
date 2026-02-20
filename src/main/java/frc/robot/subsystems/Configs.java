package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberPositionConstants;

public class Configs {
    public static final class ClimberConfig{
        public static final SparkMaxConfig m_climberConfig = new SparkMaxConfig();

        static {
            m_climberConfig
                .idleMode(IdleMode.kBrake)
                .inverted(ClimberPositionConstants.kInverted)
                .disableFollowerMode()
                .voltageCompensation(12.0)
                .smartCurrentLimit(40);

            m_climberConfig.closedLoop
                .pid(
                    ClimberPositionConstants.kP,
                    ClimberPositionConstants.kI,
                    ClimberPositionConstants.kD
                )
                .feedForward(
                    new FeedForwardConfig()
                        .kG(ClimberPositionConstants.kG)
                        .kS(ClimberPositionConstants.kS)
                        .kV(ClimberPositionConstants.kV)
                )
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(ClimberPositionConstants.kMinOutput, ClimberPositionConstants.kMaxOutput);

            m_climberConfig.encoder
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(10)
                .positionConversionFactor(ClimberPositionConstants.kConversionFactor);
            m_climberConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor).reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

        }       
    }
}
