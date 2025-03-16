package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightStatus extends Command {
    private final boolean update;
    public LimelightStatus(boolean update) {
        this.update = update;      
    }

    @Override
    public void execute() {
        if(update) {
            CommandSwerveDrivetrain.UpdatedPose = true;
        } else {
            CommandSwerveDrivetrain.UpdatedPose = false;
        }
    }
}
