package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberPositionConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ClimberIO.ClimberIoInputs;

public class ClimberPosition extends SubsystemBase {
    private final SparkMax climberLeftMotor;
    private final SparkMax climberRightMotor;

    //private final RelativeEncoder climberLeftEncoder;
    private final RelativeEncoder climberRightEncoder;
    //private final SparkClosedLoopController climberLeftController;
    private final SparkClosedLoopController climberRightController;

    //private Constraints climberLeftConstraints = new Constraints(ClimberPositionConstants.kMaxVel, ClimberPositionConstants.kMaxAccel);
    private Constraints climberRightConstraints = new Constraints(ClimberPositionConstants.kMaxVel, ClimberPositionConstants.kMaxAccel);

    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private boolean resetMode = false;

    public ClimberPosition() {
        // Initialize climber NEO motor 
        climberLeftMotor = new SparkMax(HardwareConstants.kClimberLeftCanId, SparkMax.MotorType.kBrushless);
        climberRightMotor = new SparkMax(HardwareConstants.kClimberRightCanId, SparkMax.MotorType.kBrushless);

        // Initialize encoder and controller
        //climberLeftEncoder = climberLeftMotor.getEncoder();
        climberRightEncoder = climberRightMotor.getEncoder();
        //climberLeftController = climberLeftMotor.getClosedLoopController();
        climberRightController = climberRightMotor.getClosedLoopController();

        // Configure motor settings
        climberRightMotor.configure(Configs.ClimberConfig.m_climberRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberLeftMotor.configure(Configs.ClimberConfig.m_climberLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Returns the current elevator position
    public double getPosition() {
        return climberRightEncoder.getPosition();
    }

    
    // Gets the current drawn by the left motor
    public double getLeftCurrent() {
        return climberLeftMotor.getOutputCurrent();
    }

    // Gets the current drawn by the right motor
    public double getRightCurrent() {
        return climberRightMotor.getOutputCurrent();
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
        climberRightController.setSetpoint(0.01, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // The left motor is slaved to the right motor, don't uncomment unless you want independant control
        //climberLeftController.setSetpoint(0.01, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        
        return true;
    }

    // Updates input values for logging or debugging
    public void updateInputs(ClimberIoInputs inputs) {
        inputs.m_climberGoal = goal.position;
        inputs.m_climberPos = getPosition();
        inputs.m_climberCurrent = getLeftCurrent() + getRightCurrent();
        inputs.m_climberInPosition = inPosition();
        inputs.m_climberSetpoint = setpoint.position;
        inputs.m_leftVoltage = climberLeftMotor.getBusVoltage();
        inputs.m_rightVoltage = climberRightMotor.getBusVoltage();
    }

    // Sets a new goal for the elevator position
    public void setGoal(double p_climberGoal) {
        climberRightConstraints = new Constraints(ClimberPositionConstants.kMaxVel, ClimberPositionConstants.kMaxAccel);

        setpoint = new TrapezoidProfile.State(getPosition(), 0);
        goal = new TrapezoidProfile.State(p_climberGoal, 0);
        SmartDashboard.putString("Climber Output", "Active");
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
            climberRightController.setSetpoint(-1, ControlType.kVoltage);
            //climberLeftController.setSetpoint(-1, , ControlType.kVoltage);
        } else {
            // Generate motion profile and update setpoint
            var profileRight = new TrapezoidProfile(climberRightConstraints).calculate(0.02, setpoint, goal);
            setpoint = profileRight;
            climberRightController.setSetpoint(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // Update global variables for telemetry
        GlobalVariables.m_climberExtension = getPosition();
    }
}
