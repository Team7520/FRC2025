package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.PivotConstants;
import frc.robot.Constants.ShooterConstants.TraverseConstants;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class ManualTurretControl extends Command {
    private final TurretSubsystem turret;
    private final XboxController controller;
    private static final double DEADBAND = 0.05;
    
    public ManualTurretControl(TurretSubsystem turret, XboxController controller) {
        this.turret = turret;
        this.controller = controller;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double pivotRaw = -controller.getRightY();
        double traverseRaw = -controller.getRightX();

        // Handle pivot
        if (Math.abs(pivotRaw) > DEADBAND) {
            double pivotSpeed = applyDeadbandAndScaling(pivotRaw);
            turret.setPivotSpeed(pivotSpeed * PivotConstants.maxManualSpeed);
        } else {
            turret.holdCurrentPosition();
        }

        // Handle traverse (unchanged)
        double traverseSpeed = applyDeadbandAndScaling(traverseRaw);
        turret.setTraverseSpeed(traverseSpeed * TraverseConstants.maxManualSpeed);
    }

    private double applyDeadbandAndScaling(double input) {
        // Apply deadband
        if (Math.abs(input) < DEADBAND) {
            return 0.0;
        }
        if (input < 0){
            input = input * 0.5;
        }
        
        
        return input;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setPivotSpeed(0);
        turret.setTraverseSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}