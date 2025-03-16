package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TuskSubsystem;


public class AlgaeLow extends SequentialCommandGroup {
    public AlgaeLow(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, TuskSubsystem tuskSubsystem, double conveyorSpeed) {
        addCommands(
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DUNK),
            new WaitUntilCommand(() -> endEffector.handOut()),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG),
            new WaitCommand(1)
        );
    }
}