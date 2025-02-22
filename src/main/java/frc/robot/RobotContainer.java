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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.PickUpAlgaeL3;
import frc.robot.commands.PickUpCoral;
import frc.robot.commands.Windmill;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroPivot;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final PivotSubsystem m_PivotArm = new PivotSubsystem();
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
        private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
        private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperaterControllerPort);

        public void adjustJoystickValues() {
                double rawX = m_driverController.getLeftX();
                double rawY = m_driverController.getLeftY();

        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The right trigger controls the speed of the robot.
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                MathUtil.applyDeadband(
                                                                                m_driverController
                                                                                                .getRightTriggerAxis(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));

                // Smart Dashboard Buttons
                SmartDashboard.putData("Windmill Home",
                                new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                                Constants.Windmill.WindmillState.Home, false));

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
                final Trigger zeroElevator = m_operatorController.start();
                final Trigger zeroPivot = m_operatorController.back();

                final Trigger ejectCoral = m_driverController.b();
                final Trigger pickUpCoral = m_driverController.x();
                final Trigger ejectAlgae = m_driverController.a();

                final Trigger armHome = m_operatorController.y();
                final Trigger pickUpAlgae = m_operatorController.a();

                final Trigger altPositionLeft = m_operatorController.leftBumper();
                final Trigger altPositionRightIntake = m_operatorController.rightBumper();

                // // final Trigger raiseClimber = m_operatorController.leftBumper();
                // // final Trigger lowerClimber = m_operatorController.rightBumper();

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = m_operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = m_operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = m_operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = m_operatorController.axisLessThan(4,
                                -0.25);

                // // Set dpad to coral/elevator positions
                final Trigger coralPickup = m_operatorController.povLeft();
                // final Trigger coral4Dropoff = m_operatorController.povUp();
                // final Trigger coral3Dropoff = m_operatorController.povUpRight();
                // final Trigger coral2DropOff = m_operatorController.povRight();
                // final Trigger coral1DropOff = m_operatorController.povDownRight();
                final Trigger algaeGround = m_operatorController.povDown();
                final Trigger algaeReef2 = m_operatorController.povRight();
                final Trigger algaeReef3 = m_operatorController.povUp();

                // Zero elevator - carriage must be below stage 1 or it will zero where it is
                zeroElevator.onTrue(new ZeroElevator(m_ElevatorSubsystem));
                zeroPivot.onTrue(new ZeroPivot(m_PivotArm));

                // coral subsystem
                pickUpCoral.onTrue(new PickUpCoral(m_CoralSubsystem, m_ElevatorSubsystem, m_PivotArm));
                ejectCoral.onTrue(new EjectCoral(m_CoralSubsystem, m_ElevatorSubsystem, m_PivotArm));

                // algae subsystem
                pickUpAlgae.whileTrue(new RunCommand(() -> m_AlgaeSubsystem.pickup(), m_AlgaeSubsystem))
                                .onFalse(new PickUpAlgaeL3(m_AlgaeSubsystem, m_ElevatorSubsystem, m_PivotArm));

                ejectAlgae.whileTrue(new RunCommand(() -> m_AlgaeSubsystem.eject(), m_AlgaeSubsystem))
                                .onFalse(new RunCommand(() -> m_AlgaeSubsystem.stop(), m_AlgaeSubsystem));

                // raiseClimber.onTrue(new RunCommand(() -> m_ClimberSubsystem.moveUp(),
                // m_ClimberSubsystem))
                // .onFalse(new RunCommand(() -> m_ClimberSubsystem.stop(),
                // m_ClimberSubsystem));

                // lowerClimber.onTrue(new RunCommand(() -> m_ClimberSubsystem.moveDown(),
                // m_ClimberSubsystem))
                // .onFalse(new RunCommand(() -> m_ClimberSubsystem.stop(),
                // m_ClimberSubsystem));

                // pivot subsystem
                pivotClockwise
                                .whileTrue(new RunCommand(() -> m_PivotArm.start(), m_PivotArm))
                                .onFalse(new InstantCommand(() -> m_PivotArm.keepPivot(m_PivotArm.getCurrentRotation()),
                                                m_PivotArm));

                pivotCounterClockwise.whileTrue(new RunCommand(() -> m_PivotArm.reverse(), m_PivotArm))
                                .onFalse(new InstantCommand(() -> m_PivotArm.keepPivot(m_PivotArm.getCurrentRotation()),
                                                m_PivotArm));

                // elevator subsystem
                raiseElevator.whileTrue(
                                new InstantCommand(m_ElevatorSubsystem::moveUp))
                                .onFalse(new InstantCommand(
                                                () -> m_ElevatorSubsystem.keepHeight(
                                                                m_ElevatorSubsystem.getCurrentMotorPosition())));

                lowerElevator.whileTrue(
                                new InstantCommand(m_ElevatorSubsystem::moveDown))
                                .onFalse(new InstantCommand(
                                                () -> m_ElevatorSubsystem.keepHeight(
                                                                m_ElevatorSubsystem.getCurrentMotorPosition())));

                // Positions

                // final Trigger pickUpCoral = m_operatorController.x();
                // final Trigger armHome = m_operatorController.y();
                // final Trigger pickUpAlgae = m_operatorController.a();
                // final Trigger ejectAlgae = m_operatorController.b();

                armHome.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm, Constants.Windmill.WindmillState.Home,
                                false));
                coralPickup.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralPickup, false));
                algaeGround.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpFloor, false));
                pickUpAlgae.and(altPositionLeft).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff1, false));
                pickUpAlgae.and(altPositionRightIntake).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff1, true));
                ejectAlgae.and(altPositionLeft).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff2, false));
                ejectAlgae.and(altPositionRightIntake).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff2, true));
                pickUpCoral.and(altPositionLeft).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff3, false));
                pickUpCoral.and(altPositionRightIntake).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff3, true));
                armHome.and(altPositionLeft).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff4, false));
                armHome.and(altPositionRightIntake).onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.CoralDropOff4, true));

                algaeReef2.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpReef2, false));
                algaeReef3.onTrue(new Windmill(m_ElevatorSubsystem, m_PivotArm,
                                Constants.Windmill.WindmillState.AlgaePickUpReef3, false));

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
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, 0, false));
        }
}
