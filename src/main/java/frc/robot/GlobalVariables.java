package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberPositionConstants;

public class GlobalVariables extends SubsystemBase{
    public static double m_climberExtension = 0;

    public enum RobotState {
        HOME,
        CLIMB
    }

    RobotState m_robotState = RobotState.HOME;

    public void setRobotState(RobotState s) {
        m_robotState = s;
    }

    public RobotState getRobotState() {
        return m_robotState;
    }

    public boolean isRobotState(RobotState s) {
        return (m_robotState == s);
    }

    public enum ClimberMode {
        HOME,
        EXTEND
    }

    ClimberMode m_climberMode = ClimberMode.HOME;

    public void setClimberMode(ClimberMode m) {
        m_climberMode = m;
    }

    public boolean isClimberMode(ClimberMode m) {
        return m_climberMode == m;
    }

    public ClimberMode getClimberMode() {
        return m_climberMode;
    }

    public double getDesiredClimberGoal() {
        double pos;

        switch (getClimberMode()) {
            case HOME:
                pos = ClimberPositionConstants.kHome;
                break;

            case EXTEND:
                pos = ClimberPositionConstants.kMaxHeight;
        
            default:
                pos = ClimberPositionConstants.kHome;
                break;
        }

        return pos;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("RobotState", getRobotState().toString());
        SmartDashboard.putString("Mode", getClimberMode().toString());
    }
}
