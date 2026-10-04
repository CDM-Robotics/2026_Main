package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.HardwareConstants;

public class Configs {
    public static final class FloorIntakeConfig{
        public static final SparkMaxConfig m_elevatorShooterConfig = new SparkMaxConfig();

        static{
                m_elevatorShooterConfig
                        .disableFollowerMode()
                        .idleMode(IdleMode.kCoast)
                        .inverted(FloorIntakeConstants.kCoralHolderInverted)
                        .smartCurrentLimit(FloorIntakeConstants.kCurrentLimit)
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
