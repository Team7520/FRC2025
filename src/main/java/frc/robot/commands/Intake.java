// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class Intake extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final SensorSubsystem sensorSubsystem;
    private final BooleanSupplier spinIntake;
    private final BooleanSupplier reverseIntake;
    private final DoubleSupplier fireShooter;

    /**
     * Creates a new ExampleCommand.
     *
     * @param intakeSubsystem The subsystem used by this command.
     */
    public Intake(IntakeSubsystem intakeSubsystem, SensorSubsystem sensorSubsystem, BooleanSupplier spinIntake, BooleanSupplier reverseIntake, DoubleSupplier fireShooter) {
        this.intakeSubsystem = intakeSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.spinIntake = spinIntake;
        this.reverseIntake = reverseIntake;
        this.fireShooter = fireShooter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(fireShooter.getAsDouble() > 0.5){
            intakeSubsystem.setFeederSpeed(1);
            intakeSubsystem.setRingSpeed(-0.75);
        } else if (spinIntake.getAsBoolean() && !sensorSubsystem.getColorSensorProximity()){
            intakeSubsystem.setSpeed(-0.5);
            intakeSubsystem.setRingSpeed(-0.75);
            intakeSubsystem.setFeederSpeed(0.5);
        } else if (reverseIntake.getAsBoolean()){
            intakeSubsystem.setSpeed(0.25*0.5);
            intakeSubsystem.setRingSpeed(0.75*0.5);
            intakeSubsystem.setFeederSpeed(-0.75*0.5);
        } else {
            intakeSubsystem.setSpeed(0);
            intakeSubsystem.setRingSpeed(0);
            intakeSubsystem.setFeederSpeed(0);
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
