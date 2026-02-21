package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberPosition;

public class ClimberInPosition extends Command {
    ClimberPosition m_climber;

    public ClimberInPosition(ClimberPosition p_climber){
        m_climber = p_climber;
    }

    @Override
    public boolean isFinished(){
        return m_climber.inPosition();
    }
}
