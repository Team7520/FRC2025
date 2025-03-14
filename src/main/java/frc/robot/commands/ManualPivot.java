package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ManualPivot extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final DoubleSupplier input;
    private static final double MANUAL_ADJUSTMENT_RATE = 10; // Rotations per execution

    public ManualPivot(EndEffectorSubsystem endEffectorSubsystem, DoubleSupplier input) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.input = input;
        addRequirements(endEffectorSubsystem);
    }

    @Override
    public void execute() {
        double adjustedInput = input.getAsDouble() * MANUAL_ADJUSTMENT_RATE*2;
        if (Math.abs(adjustedInput) > 0.1) { // Add deadband
            endEffectorSubsystem.manual(adjustedInput);
        } else {
            endEffectorSubsystem.holdConveyorPosition();
        }
        //endEffectorSubsystem.stopConveyor();
    }
}
