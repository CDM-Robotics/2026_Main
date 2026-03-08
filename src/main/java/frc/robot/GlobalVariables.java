package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberPositionConstants;

public class GlobalVariables extends SubsystemBase{
    public static double m_climberExtension = 0;
    public static double g_climberExtension = 0;

    public enum RobotState {
        HOME,
        CLIMB
    }

    RobotState m_robotState = RobotState.HOME;
    
    public void setRobotState(RobotState p_robotState){
        m_robotState = p_robotState;
    }

    public boolean isRobotState(RobotState p_robotState){
        return m_robotState == p_robotState;
    }

    public RobotState getRobotState(){
        return m_robotState;
    }

    public enum Mode{
        HOME
        ,EXTEND
    }

    Mode m_mode = Mode.HOME;
  
    public void setMode(Mode p_mode){
        m_mode = p_mode;
    }

    public boolean isMode(Mode p_mode){
        return m_mode == p_mode;
    }

    public Mode getMode(){
        return m_mode;
    }

    public double getDesiredElevatorGoal() {
    double pos;
        switch (getMode()) {
            case HOME:
                pos = ClimberPositionConstants.kHome;
                break;

            case EXTEND:
                pos = ClimberPositionConstants.kMaxHeight;
                break;
                
            default:
                pos = ClimberPositionConstants.kHome;
        }

        return pos;
  }

    @Override
    public void periodic() {
        SmartDashboard.putString("RobotState", getRobotState().toString());
        SmartDashboard.putString("mode", getMode().toString());
    }
}
