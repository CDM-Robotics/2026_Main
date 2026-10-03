package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
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