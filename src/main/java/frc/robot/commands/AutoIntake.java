package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.RampSubsystem;


public class AutoIntake extends ParallelCommandGroup {
    public AutoIntake(RampSubsystem rampSubsystem, EndEffectorSubsystem endEffector, double conveyorSpeed, double RAMP_SPEED) {
        addCommands(
            endEffector.setConveyorSpeedCommand(conveyorSpeed)
                .until(() -> endEffector.StopWithSensor()), // Run for 2 seconds
            rampSubsystem.run(RAMP_SPEED)
                .until(() -> endEffector.StopWithSensor()) // Run for 2 seconds
        );
    }
}