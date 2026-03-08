package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberPosition;

public class ClimberInPosition extends Command{
    ClimberPosition m_elevator;
    public ClimberInPosition(ClimberPosition p_elevator){
        m_elevator = p_elevator;
    }

    @Override
    public boolean isFinished(){
        return m_elevator.inPosition();
    }
}
