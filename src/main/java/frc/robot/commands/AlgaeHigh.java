package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TuskSubsystem;


public class AlgaeHigh extends SequentialCommandGroup {
    public AlgaeHigh(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, TuskSubsystem tuskSubsystem, double conveyorSpeed) {
        addCommands(
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DUNK),
            new WaitUntilCommand(() -> endEffector.handOut()),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.HIGHALG),
            new WaitCommand(1)
        );
    }
}