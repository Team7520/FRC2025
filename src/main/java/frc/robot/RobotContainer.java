// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Make povRight reset

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.AnalogInput;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RampSubsystem;
import frc.robot.subsystems.TuskSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.EndEffectorConstants.PivotPosition;
import frc.robot.Constants;
import frc.robot.commands.AlgaeHigh;
import frc.robot.commands.AlgaeLow;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorDownAuto;
import frc.robot.commands.L2Command;
import frc.robot.commands.L3Command;
import frc.robot.commands.L4Command;
import frc.robot.commands.AutoIntake;

import frc.robot.commands.ManualElevator;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


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
    private final LightingSubsystem lightingSubsystem = LightingSubsystem.getInstance();

    // Constants for speeds
    private static final double CONVEYOR_INTAKE_SPEED = 0.1;
    private static final double CONVEYOR_EJECT_SPEED = -0.1;
    private static final double RAMP_SPEED = 0.3;
    private static boolean speedCutOff = false;

    private SendableChooser<Command> autoChooser;

    
     

    public RobotContainer() {

        registerAutos();

        configureBindings();

        lightingSubsystem.AnimateTeam();// Flashing red
        //lightingSubsystem.FlashingWhite();// Flashing white
        //lightingSubsystem.FireAnimate();// Fire animation
        //lightingSubsystem.setLEDs(0, 0, 255);// Set a colour    
        //lightingSubsystem.RainbowAnimate();// Rainbow animation
    }

    private void registerAutos() {
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.setDefaultOption("Middle Barge to Reef G", drivetrain.getPPAutoCommand("Middle Barge to Reef G", true));
        autoChooser.addOption("Testy", drivetrain.getPPAutoCommand("Testy", true));
        autoChooser.addOption("Curvy", drivetrain.getPPAutoCommand("Curvy", true));
        autoChooser.addOption("Start 1 to F to A Three Coral", drivetrain.getPPAutoCommand("Start 1 to F to A Three Coral", true));
        autoChooser.addOption("Start 1 to F Three Coral", drivetrain.getPPAutoCommand("Start 1 to F Three Coral", true));
        autoChooser.addOption("Start 1 to F Two Coral", drivetrain.getPPAutoCommand("Start 1 to F Two Coral", true));
        autoChooser.addOption("Start 1 to F One Coral", drivetrain.getPPAutoCommand("Start 1 to F One Coral", true));
        autoChooser.addOption("Start 1 to E to F Three Coral", drivetrain.getPPAutoCommand("Start 1 to E to F Three Coral", true));
        autoChooser.addOption("Start 1 to E to F Two Coral", drivetrain.getPPAutoCommand("Start 1 to E to F Two Coral", true));
        autoChooser.addOption("Start 1 to E One Coral", drivetrain.getPPAutoCommand("Start 1 to E One Coral", true));
        
        //All of section 3 autos
        autoChooser.addOption("3-c", drivetrain.getPPAutoCommand("3-c", true));
        autoChooser.addOption("3-c-y-b", drivetrain.getPPAutoCommand("3-c-y-b", true));
        autoChooser.addOption("3-c-y-b-y-b", drivetrain.getPPAutoCommand("3-c-y-b-y-b", true));
        autoChooser.addOption("3-b", drivetrain.getPPAutoCommand("3-b", true));
        autoChooser.addOption("3-b-y-b", drivetrain.getPPAutoCommand("3-b-y-b", true));
        autoChooser.addOption("3-b-y-b-y-b", drivetrain.getPPAutoCommand("3-b-y-b-y-b", true));
        autoChooser.addOption("3-b-y-b-y-a", drivetrain.getPPAutoCommand("3-b-y-b-y-a", true));
        
        autoChooser.addOption("2-d One Coral", drivetrain.getPPAutoCommand("2-d One Coral", true));
        //autoChooser.addOption("2-d-auto", drivetrain.getPPAutoCommand("2-d-auto", false));
        SmartDashboard.putData("AutoPaths", autoChooser);
    }

    private void registerNamedCommands() {

        // Example
        NamedCommands.registerCommand("elevatorGround", new ElevatorDownAuto(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorLow", new L2Command(elevator, endEffector, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorMid", new L3Command(elevator, endEffector, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorHigh", new L4Command(elevator, endEffector, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorLowAlgae", new AlgaeLow(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorHighAlgae", new AlgaeHigh(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
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
        NamedCommands.registerCommand("intake", new AutoIntake(rampSubsystem, endEffector, CONVEYOR_EJECT_SPEED, RAMP_SPEED));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> rampSubsystem.run(0)).alongWith(new InstantCommand(() -> endEffector.stopConveyorCommand())).raceWith(new WaitCommand(0.01)));
        NamedCommands.registerCommand("StopLimelight", drivetrain.LimelightStatus(false));
        NamedCommands.registerCommand("StartLimelight", drivetrain.LimelightStatus(true));
        // NamedCommands.registerCommand("AutoAlignLeft", new InstantCommand(() -> {
        //     if(LimelightHelpers.getTV("") == true) {
        //         var cmd = AutoBuilder.followPath(drivetrain.GoLeft(1));
        //         cmd.schedule();}   
        //     }            
        // ));
        // NamedCommands.registerCommand("AutoAlignRight", new InstantCommand(() -> {
        //     if(LimelightHelpers.getTV("") == true) {
        //           var cmd = AutoBuilder.followPath(drivetrain.GoRight(1));
        //           cmd.schedule();}
        //     }
        // ));

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
                drive.withVelocityX(drivetrain.limitSpeed(-driveController.getLeftY() * MaxSpeed, speedCutOff)) // Drive forward with negative Y (forward)
                    .withVelocityY(drivetrain.limitSpeed(-driveController.getLeftX() * MaxSpeed, speedCutOff)) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate)
                    .withDeadband(MaxSpeed * drivetrain.changeDeadband(0.1, speedCutOff)) // Drive counterclockwise with negative X (left)
            )
        );

        //Cancel current path when Joystick is moved
        new Trigger(() -> 
            (Math.abs(driveController.getLeftX()) > 0.2 || 
            Math.abs(driveController.getLeftY()) > 0.2 || 
            Math.abs(driveController.getRightX()) > 0.2 || 
            Math.abs(driveController.getRightY()) > 0.2) 
            && CommandSwerveDrivetrain.pathActive
        ).onTrue(new InstantCommand(() -> {
            var cmd = AutoBuilder.followPath(drivetrain.GoRight(-1));
            cmd.schedule();
        }));
        
        driveController.y().onTrue(new InstantCommand(
            () -> speedCutOff = !speedCutOff
        ));
        
        driveController.a().onTrue(drivetrain.LimelightStatus(true));
        driveController.rightBumper().onTrue(drivetrain.LimelightStatus(false));

        //auto movements to reef
        driveController.povLeft().onTrue(new InstantCommand(() -> {
            if(LimelightHelpers.getTV("") == true) {
                var cmd = AutoBuilder.followPath(drivetrain.GoLeft(1));
                cmd.schedule();}   
            }            
        ));
                
        driveController.povRight().onTrue(new InstantCommand(() -> {
            if(LimelightHelpers.getTV("") == true) {
                  var cmd = AutoBuilder.followPath(drivetrain.GoRight(1));
                  cmd.schedule();}
            }
        ));
        
        rampSubsystem.setDefaultCommand(rampSubsystem.run(0));
        endEffector.setDefaultCommand(endEffector.run(0));
        driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Elevator Position Controls - using command factory method
        operatorController.a().onTrue(elevator.moveToPosition(ElevatorPosition.GROUND));
        //operatorController.x().onTrue(elevator.moveToPosition(ElevatorPosition.LOW));
        operatorController.x().onTrue(new L2Command(elevator, endEffector, 0));
        operatorController.y().onTrue(new L3Command(elevator, endEffector, 0));
        operatorController.b().onTrue(new L4Command(elevator, endEffector, 0));
        // Algae - press joystick inwards
        operatorController.button(9).onTrue(new AlgaeLow(elevator, endEffector, tuskSubsystem, 0));
        operatorController.button(10).onTrue(new AlgaeHigh(elevator, endEffector, tuskSubsystem, 0));

        // EndEffector Pivot Controls
        operatorController.povUp().onTrue(endEffector.setPivotPositionCommand(PivotPosition.UP));
        operatorController.povUp().onTrue(elevator.moveToPosition(ElevatorPosition.INTAKE));
        operatorController.povDown().onTrue(endEffector.setPivotPositionCommand(PivotPosition.DOWN));
        //operatorController.povRight().onTrue(endEffector.setPivotPositionCommand(PivotPosition.DUNK));
        driveController.start().onTrue(elevator.resetEncoderCommand());
        operatorController.povLeft().onTrue(endEffector.setPivotPositionCommand(PivotPosition.ALG));
                
        // Conveyor Controls (using triggers)
        operatorController.rightTrigger().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_INTAKE_SPEED));
        operatorController.leftTrigger().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED));
        

        // operatorController.rightBumper().whileTrue(endEffector.run(0.05));
        // operatorController.leftBumper().whileTrue(endEffector.run(-0.1));
        operatorController.button(7).onTrue(tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.UP));
        operatorController.button(8).onTrue(tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.DOWN));
        // Ramp Controls (using bumpers)
        operatorController.rightBumper().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED).until(() -> endEffector.StopWithSensor()));
        operatorController.rightBumper().whileTrue(rampSubsystem.run(RAMP_SPEED).until(() -> endEffector.StopWithSensor()));
        operatorController.leftBumper().whileTrue(rampSubsystem.run(-RAMP_SPEED));

        //operatorController.povDownRight().onTrue(elevator.resetEncoderCommand());

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
        return autoChooser.getSelected();
    }
}
