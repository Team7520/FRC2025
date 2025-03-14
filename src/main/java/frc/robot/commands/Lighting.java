package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightingSubsystem;

public class Lighting extends Command {
    private final LightingSubsystem lightingSubsystem;
    private final DoubleSupplier analogInput;

    public Lighting(LightingSubsystem lightingSubsystem, DoubleSupplier analogInput) {
        this.lightingSubsystem = lightingSubsystem;
        this.analogInput = analogInput;
        addRequirements(lightingSubsystem);
    }

    @Override
    public void initialize() {
        lightingSubsystem.clearAnimation();
    }

    @Override
    public void execute() {
        if (analogInput.getAsDouble() <= 1.8) {
            lightingSubsystem.StrobeAnimate(0, 255, 0);
        } else {
            lightingSubsystem.FlashingWhite();
        }
    }
}
