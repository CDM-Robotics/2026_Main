package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;

public class ReadyClimberPosition extends SequentialCommandGroup {
    public ReadyClimberPosition(GlobalVariables variables, CimberPosition pos) {
        addCommands(
            new InstantCommand(() -> pos.setGoal(variables.getDesiredClimberGoal))
            , new ClimberInPosition(climber)
        );
    }
}
