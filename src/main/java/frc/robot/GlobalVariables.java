package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberPositionConstants;

public class GlobalVariables extends SubsystemBase{
    public static double m_climberExtension = 0;
    public static double g_climberExtension = 0;
    public static double g_currentPosition = 0.0;
    public static double g_desiredPosition = 0.0;

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
        ,EXTEND,
        CLIMB,
        CUSTOM_UP,
        CUSTOM_DOWN,
        CUSTOM_STOP
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

    public static void setCurrentPosition(double pos) {
        g_currentPosition = pos;
    }

    public static double getCurrentPosition() {
        return g_currentPosition;
    }

    public static double getDesiredPosition() {
        return g_desiredPosition;
    }

    public double getDesiredElevatorGoal() {
        double pos = getCurrentPosition();
        switch (getMode()) {
            case HOME:
                g_desiredPosition = g_currentPosition;
                pos = ClimberPositionConstants.kHome;
                break;

            case EXTEND:
                g_desiredPosition = g_currentPosition;
                pos = ClimberPositionConstants.kMaxHeight;
                break;

            case CLIMB:
                g_desiredPosition = g_currentPosition;
                pos = ClimberPositionConstants.kClimbTarget;
                break;
                
            case CUSTOM_UP:
                g_desiredPosition += 10.0;
                if(g_desiredPosition > ClimberPositionConstants.kMaxHeight) {
                    g_desiredPosition = ClimberPositionConstants.kMaxHeight;
                }
                pos = g_desiredPosition;

                System.out.println("Custom up");

                break;

            case CUSTOM_DOWN:
                g_desiredPosition -= 10.0;
                if(g_desiredPosition < 0.0) {
                    g_desiredPosition = 0.0;
                }
                pos = g_desiredPosition;

                System.out.println("Custom Down");

                break;

            case CUSTOM_STOP:
                g_desiredPosition = g_currentPosition;
                pos = g_currentPosition;

                System.out.println("Custom Stop");
                break;

            default:
                System.out.println("#### BAD VALUE FOR getDesiredElevatorGoal");
                g_desiredPosition = g_currentPosition;
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
