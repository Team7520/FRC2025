package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TuskSubsystem;


public class AlgaeHigh extends SequentialCommandGroup {
    public AlgaeHigh(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, TuskSubsystem tuskSubsystem, double conveyorSpeed) {
        addCommands(
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DOWN),
            tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.DOWN),
            new WaitCommand(0.5),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.HIGHALG),
            new WaitCommand(1)
        );
    }
}