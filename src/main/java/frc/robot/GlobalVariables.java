package frc.robot;

public class GlobalVariables {
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
}
