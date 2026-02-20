package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberPositionConstants;
import frc.robot.Constants.HardwareConstants;

public class ClimberPosition extends SubsystemBase {
    private final SparkMax climberMotor;

    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberController;

    private Constraints climberConstraints = new Constraints(ClimberPositionConstants.kMaxVel, ClimberPositionConstants.kMaxAccel);

    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private boolean resetMode = false;

    public ClimberPosition() {
        // Initialize climber NEO motor 
        climberMotor = new SparkMax(HardwareConstants.kClimberCanId, SparkMax.MotorType.kBrushless);

        // Initialize encoder and controller
        climberEncoder = climberMotor.getEncoder();
        climberController = climberMotor.getClosedLoopController();

        // Configure motor settings
        climberMotor.configure(Configs.ClimberConfig.m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
