// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DropCoral;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.InitializePivot;
import frc.robot.commands.InitializePivotAndElevator;
import frc.robot.commands.IntakeMotorsOff;
import frc.robot.commands.MoveAfterAlgaePickedUp;
import frc.robot.commands.PickUpAlgae;
import frc.robot.commands.PickUpAlgaeAndReturnToHome;
import frc.robot.commands.PickUpAlgaeFromGround;
import frc.robot.commands.PickUpAlgaeFromReef;
import frc.robot.commands.PickUpCoral;
import frc.robot.commands.PickUpCoralFromSource;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreAlgaeNetLeft;
import frc.robot.commands.ScoreAlgaeNetRight;
import frc.robot.commands.Windmill;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroPivot;
import frc.robot.commands.ToClimb;
import frc.robot.commands.states.ToAlgaeGround;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.commands.states.ToCoralDropOff2;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.commands.states.ToCoralSource;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
        private final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final CoralSubsystem coralSubsystem = new CoralSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
        private final LimelightSubsystem limelight = new LimelightSubsystem(driveSubsystem);

        // The driver's controller
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperaterControllerPort);
        GenericHID driverControllerRumble = driverController.getHID();
        GenericHID operatorControllerRumble = operatorController.getHID();

        public void adjustJoystickValues() {
                double rawX = driverController.getLeftX();
                double rawY = driverController.getLeftY();

        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                driveSubsystem.setDefaultCommand(
                                // The right trigger controls the speed of the robot.
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> driveSubsystem.drive(
                                                                MathUtil.applyDeadband(
                                                                                Math.pow(driverController
                                                                                                .getRightTriggerAxis(),
                                                                                                2),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                Math.pow(driverController.getRightX(),
                                                                                                3),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                driveSubsystem));

                // Smart Dashboard Buttons
                // SmartDashboard.putData("Windmill Home",
                // new Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.Home, false));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */

        private void configureButtonBindings() {

                // Manual Zero buttons for elevator and pivot
                final Trigger zeroElevator = operatorController.start();
                final Trigger zeroPivot = operatorController.back();

                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                zeroElevator.onTrue(new ZeroElevator(elevatorSubsystem));
                zeroPivot.onTrue(new InitializePivotAndElevator(pivotSubsystem, elevatorSubsystem));

                // Reset heading of robot for field relative drive
                final Trigger zeroHeadingButton = driverController.start();
                zeroHeadingButton.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem));

                // Driver operations
                final Trigger ejectCoral = driverController.b();
                final Trigger pickUpCoral = driverController.leftTrigger();
                final Trigger ejectAlgae = driverController.a();
                final Trigger limelightTrigger1 = driverController.y();
                final Trigger limelightTrigger2 = driverController.x();
                final Trigger shootAlgaeLeft = driverController.leftBumper();
                final Trigger shootAlgaeRight = driverController.rightBumper();
                final Trigger raiseClimber = driverController.povUp();
                final Trigger lowerClimber = driverController.povDown();
                // final Trigger intakeMotorsOff = driverController.back();
                final Trigger altButton = driverController.back();
                final Trigger deployClimber = driverController.povLeft();

                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
                ejectCoral.and(altButton).onTrue(new DropCoral(coralSubsystem, elevatorSubsystem, pivotSubsystem));
                pickUpCoral.onTrue(new PickUpCoralFromSource(coralSubsystem, elevatorSubsystem, pivotSubsystem, false));
                // pickUpCoral.and(altButton).whileTrue(new RunCommand(() ->
                // coralSubsystem.pickup(), coralSubsystem))
                // .onFalse(new SequentialCommandGroup(
                // new InstantCommand(coralSubsystem::stop),
                // new WaitCommand(0.25),
                // new Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.Home, false)));
                pickUpCoral.and(altButton)
                                .onTrue(new PickUpCoralFromSource(coralSubsystem, elevatorSubsystem, pivotSubsystem,
                                                true));

                ejectAlgae.onTrue(new ScoreAlgae(algaeSubsystem));
                shootAlgaeLeft.onTrue(new ScoreAlgaeNetLeft(algaeSubsystem, elevatorSubsystem, pivotSubsystem,
                                coralSubsystem));
                shootAlgaeRight.onTrue(new ScoreAlgaeNetRight(algaeSubsystem, elevatorSubsystem, pivotSubsystem,
                                coralSubsystem));
                raiseClimber.onTrue(new RunCommand(() -> climberSubsystem.moveUp(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));
                deployClimber.onTrue(
                                new frc.robot.commands.ToClimb(elevatorSubsystem, pivotSubsystem, climberSubsystem));
                lowerClimber.onTrue(new RunCommand(() -> climberSubsystem.moveDown(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));

                // Operator Controls
                final Trigger manualCoral = operatorController.rightTrigger();
                final Trigger manualAlgae = operatorController.leftTrigger();

                manualCoral.whileTrue(
                                new SequentialCommandGroup(new ToCoralSource(elevatorSubsystem, pivotSubsystem, false),
                                                new RunCommand(() -> coralSubsystem.pickup(), coralSubsystem)))
                                .onFalse(new SequentialCommandGroup(
                                                new InstantCommand(coralSubsystem::stop),
                                                new WaitCommand(0.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home, false)));

                manualAlgae.whileTrue(new RunCommand(() -> algaeSubsystem.pickup(), algaeSubsystem))
                                .onFalse(new InstantCommand(algaeSubsystem::holdAlgae));

                // position controls
                final Trigger armHome = operatorController.leftBumper();
                final Trigger algaeGround = operatorController.povDown();
                final Trigger algaeReef2 = operatorController.povRight();
                final Trigger algaeReef3 = operatorController.povUp();
                final Trigger algaeProcessor = operatorController.povLeft();
                final Trigger coralDropOff4 = operatorController.y();
                final Trigger coralDropOff3 = operatorController.x();
                final Trigger coralDropOff2 = operatorController.b();
                final Trigger coralDropOff1 = operatorController.a();
                final Trigger altPositionRight = operatorController.rightBumper();

                armHome.onTrue(new ToHomeCommand(elevatorSubsystem, pivotSubsystem, coralSubsystem));
                algaeGround.onTrue(new PickUpAlgaeFromGround(algaeSubsystem, elevatorSubsystem, pivotSubsystem));
                algaeReef2.onTrue(new PickUpAlgaeFromReef(algaeSubsystem, elevatorSubsystem, pivotSubsystem, false,
                                false));
                algaeReef3.onTrue(new PickUpAlgaeFromReef(algaeSubsystem, elevatorSubsystem, pivotSubsystem, true,
                                false));
                algaeProcessor.onTrue(new ToAlgaeGround(elevatorSubsystem, pivotSubsystem));
                coralDropOff1.onTrue(new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, false));
                coralDropOff2.onTrue(new ToCoralDropOff2(elevatorSubsystem, pivotSubsystem, false));
                coralDropOff3.onTrue(new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, false));
                coralDropOff4.onTrue(new ToCoralDropOff4(elevatorSubsystem, pivotSubsystem, false));

                algaeReef2.and(altPositionRight).onTrue(new PickUpAlgaeFromReef(algaeSubsystem, elevatorSubsystem,
                                pivotSubsystem, false, true));
                algaeReef3.and(altPositionRight).onTrue(
                                new PickUpAlgaeFromReef(algaeSubsystem, elevatorSubsystem, pivotSubsystem, true, true));

                coralDropOff1.and(altPositionRight)
                                .onTrue(new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, true));
                coralDropOff2.and(altPositionRight)
                                .onTrue(new ToCoralDropOff2(elevatorSubsystem, pivotSubsystem, true));
                coralDropOff3.and(altPositionRight)
                                .onTrue(new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, true));
                coralDropOff4.and(altPositionRight)
                                .onTrue(new ToCoralDropOff4(elevatorSubsystem, pivotSubsystem, true));

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = operatorController.axisLessThan(4,
                                -0.25);

                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .onTrue(new InstantCommand(() -> pivotSubsystem.start()))
                                .onFalse(new InstantCommand(
                                                () -> pivotSubsystem.holdPivotArmManual()));
                pivotCounterClockwise.onTrue(new InstantCommand(() -> pivotSubsystem.reverse()))
                                .onFalse(new InstantCommand(
                                                () -> pivotSubsystem.holdPivotArmManual()));
                raiseElevator.whileTrue(
                                new InstantCommand(() -> elevatorSubsystem.moveUp())
                                                .andThen(new WaitUntilCommand(
                                                                () -> elevatorSubsystem.ElevatorRaiseLimitHit()))
                                                .andThen(new InstantCommand(() -> elevatorSubsystem.holdHeight())))
                                .onFalse(new InstantCommand(
                                                () -> elevatorSubsystem.holdHeight()));
                lowerElevator.whileTrue(
                                new InstantCommand(() -> elevatorSubsystem.moveDown())
                                                .andThen(new WaitUntilCommand(
                                                                () -> elevatorSubsystem.ElevatorLowerLimitHit()))
                                                .andThen(new InstantCommand(() -> elevatorSubsystem.holdHeight())))
                                .onFalse(new InstantCommand(
                                                () -> elevatorSubsystem.holdHeight()));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
                                config);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                driveSubsystem::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                driveSubsystem::setModuleStates,
                                driveSubsystem);

                // Reset odometry to the starting pose of the trajectory.
                driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0, false));
        }

        public Command rumbleDriverCommand() {
                return new RunCommand(() -> rumbleDriverCtrl()).withTimeout(2).finallyDo(() -> stopRumbleDriverCtrl());
        }

        public void rumbleDriverCtrl() {
                driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        }

        public void stopRumbleDriverCtrl() {
                driverControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                operatorControllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        }

}
