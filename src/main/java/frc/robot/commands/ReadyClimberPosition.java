package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ClimberPosition;


public class ReadyClimberPosition extends SequentialCommandGroup {
    public ReadyClimberPosition(GlobalVariables variables, ClimberPosition climber) {
        addCommands(
            new InstantCommand(() -> climber.setGoal(variables.getDesiredClimberGoal()))
            , new ClimberInPosition(climber)
        );
    }
}
