package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;


public class L3Command extends SequentialCommandGroup {
    public L3Command(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, double conveyorSpeed) {
        addCommands(
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DOWN),
            new WaitCommand(0.5),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.MID),
            new WaitCommand(1.5),
            endEffector.setConveyorSpeedCommand(conveyorSpeed)
                .withTimeout(2) // Run for 2 seconds
        );
    }
}