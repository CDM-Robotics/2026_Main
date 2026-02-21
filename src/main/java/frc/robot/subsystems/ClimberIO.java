package frc.robot.subsystems;

public interface ClimberIO {
    public static class ClimberIoInputs {
        public double m_climberPos;
        public double m_climberGoal;
        public double m_climberCurrent;
        public double m_climberSetpoint;
        public double m_leftVoltage;
        public double m_rightVoltage;
        public double m_speed;
        public boolean m_climberInPosition;
    }

    public default void updateInputs(ClimberIoInputs inputs) {}

    public default void setGoal(double p_ref) {}

    public default double getGoal() {return 0.0;}

    public default double getPosition() {return 0.0;}

    public default double getCurrent() {return 0.0;}

    public default double getSetpoint() {return 0.0;}

    public default boolean inPosition() {return true;}

    public default void PID() {}

    public default void resetEncoder() {}

    public default boolean isResetMode() {return false;}

    public default void reset(boolean m_reset) {}
}
