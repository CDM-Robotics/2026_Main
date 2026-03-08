package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberPosition;
import frc.robot.GlobalVariables;

public class ReadyClimberPosition extends SequentialCommandGroup{
    public ReadyClimberPosition(GlobalVariables variables, ClimberPosition elevator){
        addCommands(
            new InstantCommand(()-> elevator.setGoal(variables.getDesiredElevatorGoal()))
            ,new ClimberInPosition(elevator)
            //,new InstantCommand(()-> pivot.setAngle(ElevatorPivotConstants.kReady))
            //,new PivotInPosition(pivot)
        );
    }
}