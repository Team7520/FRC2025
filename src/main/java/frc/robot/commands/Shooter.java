package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class Shooter extends Command {
    private final ShooterSubsystem shooterSubsystem;
    // private final DoubleSupplier horizontalTraverse;
    // private final DoubleSupplier verticalTraverse;
    private final DoubleSupplier throttleSup;
    // private final BooleanSupplier shooterStay;

    public Shooter(ShooterSubsystem shooterSubsystem, DoubleSupplier throttleSup) {
        this.shooterSubsystem = shooterSubsystem;
        // this.horizontalTraverse = horizontalTraverse;
        // this.verticalTraverse = verticalTraverse;
        this.throttleSup = throttleSup;
        
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // double horizontal = horizontalTraverse.getAsDouble()*0.12*-1;
        // double vertical = verticalTraverse.getAsDouble()*0.2*-1; 
        double throttle = throttleSup.getAsDouble()*0.9; //Math.max(throttleSup.getAsDouble(), (slowThrottleSup.getAsDouble() * 0.25)) * (invertSup.getAsBoolean() ? -1 : 1) * 1;
        // if (shooterStay.getAsBoolean()){
        //     if(vertical <= 0.9){
        //         vertical += 0.06;
        //     }
        // }
        //shooterSubsystem.setPivotSpeed(vertical);
        // shooterSubsystem.setHorizontalSpeed(horizontal);
        shooterSubsystem.setSpeed(throttle, false);   
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
