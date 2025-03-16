package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevator extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier input;
    private static final double MANUAL_ADJUSTMENT_RATE = 3; // Rotations per execution

    public ManualElevator(ElevatorSubsystem elevator, DoubleSupplier input) {
        this.elevator = elevator;
        this.input = input;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double adjustedInput = input.getAsDouble() * MANUAL_ADJUSTMENT_RATE*2;
        if (Math.abs(adjustedInput) > 0.1) { // Add deadband
            elevator.addToPosition(adjustedInput);
        }
    }
}
