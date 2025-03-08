package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TuskSubsystem;


public class ElevatorDownFromIntake extends SequentialCommandGroup {
    public ElevatorDownFromIntake(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, TuskSubsystem tuskSubsystem, double conveyorSpeed) {
        addCommands(
            tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.UP),
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DOWN),
            new WaitCommand(0.1),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.GROUND),
            new WaitCommand(0.25)
        );
    }
}