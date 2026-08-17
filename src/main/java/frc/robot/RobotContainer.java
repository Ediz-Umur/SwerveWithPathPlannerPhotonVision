// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.MoveAndAlign;
import frc.robot.commands.ShooterCalculate;
import frc.robot.commands.WheelStabilize;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class RobotContainer {
    private boolean activateAutonomous = true;

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandPS5Controller joystick = new CommandPS5Controller(0);
    public final static Intake intake = new Intake();
    public final static Shooter shooter = new Shooter();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Constants.drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            Constants.drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        if (Constants.xLock){
            Constants.drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        }

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            Constants.drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.triangle().onTrue(drivetrain.sysIdDynamic(Direction.kForward).withTimeout(3));
        //joystick.circle().onTrue(drivetrain.sysIdDynamic(Direction.kReverse).withTimeout(3));
        //joystick.cross().onTrue(drivetrain.sysIdQuasistatic(Direction.kForward).withTimeout(5));
        //joystick.square().onTrue(drivetrain.sysIdQuasistatic(Direction.kReverse).withTimeout(5));

        // reset the field-centric heading on left bumper press
        joystick.options().onTrue(Constants.drivetrain.runOnce(() -> Constants.drivetrain.seedFieldCentric()));
        joystick.L1().onTrue(new InstantCommand(()-> intake.runIntake()));
        joystick.R1().onTrue(new InstantCommand(()-> intake.StopIntake()));

        joystick.square().onTrue(new AutoAlign());
        joystick.triangle().onTrue(new ShooterCalculate());
        joystick.cross().toggleOnTrue(new MoveAndAlign());
        joystick.circle().toggleOnTrue(new InstantCommand(() -> Constants.drivetrain.setControl(new SwerveRequest.SwerveDriveBrake())).repeatedly());
        joystick.L2().onTrue(new ShooterCalculate());
        
        Constants.drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        if (activateAutonomous){
        return new PathPlannerAuto("AutoTestMiddle");
        } else {
            return new PrintCommand("The autonomous path is disabled. There is a variable in the code to enable it");
        }
    }

}
