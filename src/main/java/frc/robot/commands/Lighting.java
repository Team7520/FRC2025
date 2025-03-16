package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
        lightingSubsystem.setSideLEDs(255, 0, 0);
    }

    @Override
    public void execute() {
        if (analogInput.getAsDouble() <= 1.8) {
            lightingSubsystem.setSideLEDs(255, 0, 0);
        } else {
            lightingSubsystem.setSideLEDs(255, 255, 255);
        }
        if (LimelightHelpers.getFiducialID("") == 0) {// Check if fiducial ID is 0, which means Limelight is disconnected, and change side LED colour accordingly
            lightingSubsystem.StrobeAnimate(255, 0, 0);// Red for Limelight disconnect
        }
        else if (LimelightHelpers.getFiducialID("") == -1) {
            lightingSubsystem.StrobeAnimate(255, 255, 0);// Yellow for Limelight connected but no AprilTag detected
        }
        else {
            lightingSubsystem.StrobeAnimate(0, 255, 0);// Green for Apriltag detected
        }
    }
}
