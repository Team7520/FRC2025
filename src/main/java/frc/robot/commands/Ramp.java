package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RampSubsystem;

public class Ramp extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RampSubsystem rampSubsystem;
    private final BooleanSupplier spinRamp;

    /**
     * Creates a new ExampleCommand.
     *
     * @param intakeSubsystem The subsystem used by this command.
     */
    public Ramp(RampSubsystem rampSubsystem, BooleanSupplier spinRamp) {
        this.rampSubsystem = rampSubsystem;
        this.spinRamp = spinRamp;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rampSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(spinRamp.getAsBoolean()){
            rampSubsystem.setSpeed(0.3);
        } else {
            rampSubsystem.setSpeed(0);
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