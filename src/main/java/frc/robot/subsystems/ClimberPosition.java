package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberPositionConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ClimberIO.ClimberIoInputs;

public class ClimberPosition extends SubsystemBase {

    // NEO motors with their CAN IDs
    private final SparkMax climberMotor;
    private final SparkMax climberMotorSlave;

    // Encoder and controller for elevator control
    private final RelativeEncoder elevatorEncoder;
    private final SparkClosedLoopController elevatorController;

    // Constraints for trapezoidal motion profile
    private Constraints elevatorConstraints = new Constraints(ClimberPositionConstants.kMaxVel, ClimberPositionConstants.kMaxAccel);

    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private boolean resetMode = false;

    public ClimberPosition() {
        // Initialize elevator motors
        climberMotor = new SparkMax(HardwareConstants.kClimberRightCanId, SparkMax.MotorType.kBrushless);
        climberMotorSlave = new SparkMax(HardwareConstants.kClimberLeftCanId, SparkMax.MotorType.kBrushless);

        // Initialize encoder and controller
        elevatorEncoder = climberMotor.getEncoder();
        elevatorController = climberMotor.getClosedLoopController();

        // Configure motor settings
        climberMotor.configure(Configs.ClimberConfig.m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberMotorSlave.configure(Configs.ClimberConfig.m_climberSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Set initial state of setpoints and goal
        resetEncoder();

    }

    // Returns the current elevator position
    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    // Gets the current drawn by the right motor
    public double getClimberCurrent() {
        return climberMotor.getOutputCurrent();
    }
    
    
    // Gets the current goal
    public double getGoal() {
        return goal.position;
    }

    public double getSetpoint() {
        return setpoint.position;
    }
    
    // Checks if the elevator is within the goal tolerance
    public boolean inPosition() {
        double pos = getPosition();
        if(pos < 0) {
            pos = 0.0;
        }

        SmartDashboard.putNumber("Elevator Position", pos);
        return (Math.abs(pos - getGoal()) < ClimberPositionConstants.kTolerance);
    }

    public boolean killOutput() {
        SmartDashboard.putString("Elevator Output", "Killed");
        elevatorController.setSetpoint(0.01, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        return true;
    }

    // Updates input values for logging or debugging
    public void updateInputs(ClimberIoInputs inputs) {
        inputs.m_elevatorGoal = goal.position;
        inputs.m_elevatorPos = getPosition();
        inputs.m_elevatorCurrent = getClimberCurrent();
        inputs.m_elevatorInPosition = inPosition();
        inputs.m_elevatorSetpoint = setpoint.position;
        inputs.m_climberVoltage = climberMotor.getBusVoltage();
    }

    // Sets a new goal for the elevator position
    public void setGoal(double p_elevatorGoal) {
        elevatorConstraints = new Constraints(ClimberPositionConstants.kMaxVel, ClimberPositionConstants.kMaxAccel);

        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = new TrapezoidProfile.State(p_elevatorGoal, 0);
        SmartDashboard.putString("Elevator Output", "Active");
    }

    // Resets the encoder and motion profile states
    public void resetEncoder() {
        goal = setpoint = new TrapezoidProfile.State(0, 0);
        resetMode = false;
    }

    // Checks if the elevator is in reset mode
    public boolean isResetMode() {
        return resetMode;
    }

    // Sets reset mode
    public void reset(boolean reset) {
        resetMode = reset;
    }

    // Periodic method for controlling the elevator
    @Override
    public void periodic() {
        if (resetMode) {
            elevatorController.setSetpoint(-1, ControlType.kVoltage);
            //elevatorController.setReference(-1, ControlType.kVoltage);
        } else {
            // Generate motion profile and update setpoint
            var profile = new TrapezoidProfile(elevatorConstraints).calculate(0.02, setpoint, goal);
            setpoint = profile;
            elevatorController.setSetpoint(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            //elevatorController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // Update global variables for telemetry
        GlobalVariables.g_climberExtension = getPosition();
    }
}
