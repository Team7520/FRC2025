package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;


public class L4CommandDunk extends SequentialCommandGroup {
    public L4CommandDunk(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, double conveyorSpeed) {
        addCommands(
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.L4DOWN),
            new WaitUntilCommand(() -> endEffector.handOut()),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.HIGH),
            new WaitCommand(1.5),
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DUNK),
            new WaitCommand(0.5),
            endEffector.setConveyorSpeedCommand(conveyorSpeed)
                .withTimeout(0.65) // Run for 2 seconds
        );
    }
}