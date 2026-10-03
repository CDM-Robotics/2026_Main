package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.SimpleMotorFeedforward;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.SubsystemBase;
import frc.robot.subsystems.Configs;
import frc.robot.subsystems.Configs.FloorIntakeConfig;
import frc.robot.Constants;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.HardwareConstants;

public class Hopper extends SubsystemBase {

    public static class ElevatorShooterIOInputs {
        public double m_intakeCurrent;
        public double m_intakeVelocity;
        public double m_intakeReference;
    }

    private SparkMax elevatorShooterMotor;
    private final SparkClosedLoopController m_elevatorShooterController;
    private final RelativeEncoder m_shooterEncoder;
    private SimpleMotorFeedforward m_elevatorShooterFeedforward = new SimpleMotorFeedforward(FloorIntakeConstants.kS, FloorIntakeConstants.kV);
    private final ElevatorShooterIOInputs inputs = new ElevatorShooterIOInputs();

    private double m_shooterReference;

    

    public Hopper() {
        elevatorShooterMotor = new SparkMax(Constants.HardwareConstants.kCanBus, Constants.HardwareConstants.kHopperCanId, MotorType.kBrushless);

        //initialize PID controller
        m_elevatorShooterController = elevatorShooterMotor.getClosedLoopController();

        //initalize encoder
        m_shooterEncoder = elevatorShooterMotor.getEncoder();

        //apply config
        elevatorShooterMotor.configure(Configs.FloorIntakeConfig.m_elevatorShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset target speed in init
        m_shooterReference = 0;
    }

    public void setSetpoint(double p_rpm){
        m_elevatorShooterFeedforward = new SimpleMotorFeedforward(FloorIntakeConstants.kS, FloorIntakeConstants.kV);
        if(p_rpm < 0){
            FloorIntakeConfig.m_elevatorShooterConfig.closedLoop.p(FloorIntakeConstants.kPReverse);
        }else{
            FloorIntakeConfig.m_elevatorShooterConfig.closedLoop.p(FloorIntakeConstants.kP);
        }
        elevatorShooterMotor.configure(FloorIntakeConfig.m_elevatorShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_shooterReference = p_rpm;
    }

    public double getSetpoint(){
        return m_shooterReference;
    }

    public void PID() {
        m_elevatorShooterController.setSetpoint(m_shooterReference, ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, m_elevatorShooterFeedforward.calculate(getSetpoint()));
    }

    public double getVelocity(){
        return m_shooterEncoder.getVelocity().get();
    }

    public void updateInputs(ElevatorShooterIOInputs inputs){
        inputs.m_intakeReference = getSetpoint();
        inputs.m_intakeCurrent = getCurrent();
        inputs.m_intakeVelocity = getVelocity();
    }

    @Override
    public void periodic(){
        updateInputs(inputs);
        PID();
    }

    public double getCurrent() {
        return elevatorShooterMotor.getOutputCurrent().get();
    }

}
