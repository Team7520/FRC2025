// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RampSubsystem;
import frc.robot.subsystems.TuskSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.EndEffectorConstants.PivotPosition;
import frc.robot.Constants;
import frc.robot.commands.ManualElevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1); // Port 1
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final RampSubsystem rampSubsystem = RampSubsystem.getInstance();
    private final EndEffectorSubsystem endEffector = EndEffectorSubsystem.getInstance();
    private final TuskSubsystem tuskSubsystem = TuskSubsystem.getInstance();

    // Constants for speeds
    private static final double CONVEYOR_INTAKE_SPEED = 0.1;
    private static final double CONVEYOR_EJECT_SPEED = -0.1;
    private static final double RAMP_SPEED = 0.3;

    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        registerAutos();

        configureBindings();
    }

    private void registerAutos() {

        registerNamedCommands();

        autoChooser.setDefaultOption("1m F + 90 CW", drivetrain.getPPAutoCommand("1m F + 90 CW", true));
        autoChooser.addOption("2m F", drivetrain.getPPAutoCommand("2m F", true));
        autoChooser.addOption("1m F", drivetrain.getPPAutoCommand("1m F", true));
        autoChooser.addOption("0.5m F", drivetrain.getPPAutoCommand("0.5m F", true));
        autoChooser.addOption("0.5m B", drivetrain.getPPAutoCommand("0.5m B", true));
        autoChooser.addOption("1m B", drivetrain.getPPAutoCommand("1m B", true));
        autoChooser.addOption("2m B", drivetrain.getPPAutoCommand("2m B", true));
        autoChooser.addOption("1m R", drivetrain.getPPAutoCommand("1m R", true));
        autoChooser.addOption("1m F + 1m R", drivetrain.getPPAutoCommand("1m F + 1m R", true));
        autoChooser.addOption("45 CCW", drivetrain.getPPAutoCommand("45 CCW", true));
        autoChooser.addOption("45 CW", drivetrain.getPPAutoCommand("45 CW", true));
        autoChooser.addOption("180 CCW", drivetrain.getPPAutoCommand("180 CCW", true));
        autoChooser.addOption("90 CCW", drivetrain.getPPAutoCommand("90 CCW", true));
        autoChooser.addOption("90 CW", drivetrain.getPPAutoCommand("90 CW", true));
        autoChooser.addOption("180 CW", drivetrain.getPPAutoCommand("180 CW", true));
        autoChooser.addOption("135 CW", drivetrain.getPPAutoCommand("135 CW", true));
        autoChooser.addOption("135 CCW", drivetrain.getPPAutoCommand("135 CCW", true));
        autoChooser.addOption("1m F + 90 CW + 1m R", drivetrain.getPPAutoCommand("1m F + 90 CW + 1m R", true));
        
        SmartDashboard.putData("AutoPaths", autoChooser);
    }

    private void registerNamedCommands() {

        // Example
        NamedCommands.registerCommand("elevatorGround", new InstantCommand(() -> elevator.moveToPosition(ElevatorPosition.GROUND)));
        NamedCommands.registerCommand("elevatorLow", new InstantCommand(() -> elevator.moveToPosition(ElevatorPosition.LOW)));
        NamedCommands.registerCommand("elevatorMid", new InstantCommand(() -> elevator.moveToPosition(ElevatorPosition.MID)));
        NamedCommands.registerCommand("elevatorHigh", new InstantCommand(() -> elevator.moveToPosition(ElevatorPosition.HIGH)));
        NamedCommands.registerCommand("elevatorLowAlgae", new InstantCommand(() -> elevator.moveToPosition(ElevatorPosition.LOWALG)));
        NamedCommands.registerCommand("elevatorHighAlgae", new InstantCommand(() -> elevator.moveToPosition(ElevatorPosition.HIGHALG)));
        NamedCommands.registerCommand("pivotUp", new InstantCommand(() -> endEffector.setPivotPositionCommand(PivotPosition.UP)));
        NamedCommands.registerCommand("pivotDown", new InstantCommand(() -> endEffector.setPivotPositionCommand(PivotPosition.DOWN)));
        NamedCommands.registerCommand("pivotDunk", new InstantCommand(() -> endEffector.setPivotPositionCommand(PivotPosition.DUNK)));
        NamedCommands.registerCommand("pivotAlgae", new InstantCommand(() -> endEffector.setPivotPositionCommand(PivotPosition.ALG)));
        NamedCommands.registerCommand("conveyorIntake", new InstantCommand(() -> endEffector.setConveyorSpeedCommand(CONVEYOR_INTAKE_SPEED)));
        NamedCommands.registerCommand("conveyorEject", new InstantCommand(() -> endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED)));
        NamedCommands.registerCommand("conveyorStop", new InstantCommand(() -> endEffector.stopConveyorCommand()));
        NamedCommands.registerCommand("tuskUp", new InstantCommand(() -> tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.UP)));
        NamedCommands.registerCommand("tuskDown", new InstantCommand(() -> tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.DOWN)));
        NamedCommands.registerCommand("rampIntake", new InstantCommand(() -> rampSubsystem.run(RAMP_SPEED)));
        NamedCommands.registerCommand("rampReverse", new InstantCommand(() -> rampSubsystem.run(-RAMP_SPEED)));
        NamedCommands.registerCommand("rampStop", new InstantCommand(() -> rampSubsystem.run(0)));

        /*
        NamedCommands.registerCommand("log", new InstantCommand(() -> System.out.println("eeeeeeeeeeeeeeeeeeeeeeeee")));
        NamedCommands.registerCommand("intakeOut", new AutoIntake(Position.INTAKE));
        NamedCommands.registerCommand("intake", new InstantCommand(() -> intakeSubsystem.setSpeed(Position.INTAKE.getSpeed())));
        NamedCommands.registerCommand("stopIntaking", new InstantCommand(() -> intakeSubsystem.setSpeed(0)));
        NamedCommands.registerCommand("intakeIn", new AutoIntake(Position.SHOOT));
        NamedCommands.registerCommand("stopShoot", new AutoShoot(0, false));
        NamedCommands.registerCommand("AutoNotePickUp", new AutoNotePickUp());
*/
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        rampSubsystem.setDefaultCommand(rampSubsystem.run(0));
        endEffector.setDefaultCommand(endEffector.run(0));
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Elevator Position Controls - using command factory method
        operatorController.a().onTrue(elevator.moveToPosition(ElevatorPosition.GROUND));
        operatorController.x().onTrue(elevator.moveToPosition(ElevatorPosition.LOW));
        operatorController.y().onTrue(elevator.moveToPosition(ElevatorPosition.MID));
        operatorController.b().onTrue(elevator.moveToPosition(ElevatorPosition.HIGH));
        operatorController.button(9).onTrue(elevator.moveToPosition(ElevatorPosition.LOWALG));
        operatorController.button(10).onTrue(elevator.moveToPosition(ElevatorPosition.HIGHALG));



        // EndEffector Pivot Controls
        operatorController.povUp().onTrue(endEffector.setPivotPositionCommand(PivotPosition.UP));
        operatorController.povDown().onTrue(endEffector.setPivotPositionCommand(PivotPosition.DOWN));
        operatorController.povRight().onTrue(endEffector.setPivotPositionCommand(PivotPosition.DUNK));
        operatorController.povLeft().onTrue(endEffector.setPivotPositionCommand(PivotPosition.ALG));

                
        // Conveyor Controls (using triggers)
        operatorController.rightTrigger().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_INTAKE_SPEED));
        operatorController.leftTrigger().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED));

        // operatorController.rightBumper().whileTrue(endEffector.run(0.05));
        // operatorController.leftBumper().whileTrue(endEffector.run(-0.1));
        operatorController.button(7).onTrue(tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.UP));
        operatorController.button(8).onTrue(tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.DOWN));
        // Ramp Controls (using bumpers)
        operatorController.rightBumper().whileTrue(rampSubsystem.run(RAMP_SPEED));
        operatorController.leftBumper().whileTrue(rampSubsystem.run(-RAMP_SPEED));

        // Default commands to stop when not actively controlled

        endEffector.setDefaultCommand(endEffector.stopConveyorCommand());
        rampSubsystem.setDefaultCommand(rampSubsystem.run(0));

        // Manual elevator control
        elevator.setDefaultCommand(
            new ManualElevator(elevator, () -> -operatorController.getRightY())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
