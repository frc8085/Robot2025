// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DropCoral;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.InitializePivotAndElevator;
import frc.robot.commands.LockPivotAndElevatorCommand;
import frc.robot.commands.PickUpAlgaeFromGround;
import frc.robot.commands.PickUpCoralFromSource;
import frc.robot.commands.RetractClimb;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreAlgaeNetLeft;
import frc.robot.commands.ScoreAlgaeNetRight;
import frc.robot.commands.ScoreCoralL4;
import frc.robot.commands.Windmill;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.autoCommands.AutoYCoral1;
import frc.robot.commands.DeployClimb;
import frc.robot.commands.scoring.DriveToCoralBlue;
import frc.robot.commands.scoring.DriveToCoralYellow;
import frc.robot.commands.scoring.ResetOperatorInputs;
import frc.robot.commands.sequences.AutoRemoveAlgaeL3ScoreL3;
import frc.robot.commands.PickUpAlgaeFromReef;
import frc.robot.commands.states.ToAlgaeGround;
import frc.robot.commands.states.ToCoralDropOff1;
import frc.robot.commands.states.ToCoralDropOff2;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.commands.states.ToCoralSource;
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

        private final SendableChooser<Command> autoChooser;
        protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

        private final Field2d field;

        public boolean automated = true; // Controls automation state

        // Register Named Commands for PathPlanner
        private void configureAutoCommands() {
                NamedCommands.registerCommand("InitializePE",
                                new InitializePivotAndElevator(pivotSubsystem, elevatorSubsystem));
                NamedCommands.registerCommand("AutoYCoral1",
                                new AutoYCoral1(coralSubsystem, elevatorSubsystem, pivotSubsystem, true));
        }

        public enum ScoreDirection {
                LEFT,
                RIGHT,
                UNDECIDED
        }

        public enum AlgaeLevel {
                TWO,
                THREE,
                NONE,
                UNDECIDED
        }

        public enum CoralLevel {
                ONE,
                TWO,
                THREE,
                FOUR,
                NONE,
                UNDECIDED
        }

        // TODO: Implement buttons that change these values. Consider using LEDs to show
        // the target mode
        public static ScoreDirection scoreDirection = ScoreDirection.UNDECIDED;
        public static AlgaeLevel algaeLevel = AlgaeLevel.UNDECIDED;
        public static CoralLevel coralLevel = CoralLevel.UNDECIDED;

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

                // Register Named Commands for Pathplanner
                configureAutoCommands();

                // testing alternate drive
                // MoveCommand moveCommand = new MoveCommand(this.driveSubsystem,
                // driverController);
                // driveSubsystem.setDefaultCommand(moveCommand);

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
                                                                                0),
                                                                -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                Math.pow(driverController.getRightX(),
                                                                                                3),
                                                                                OIConstants.kTurnDeadband),
                                                                true),
                                                driveSubsystem));

                // Another option that allows you to specify the default auto by its name
                autoChooser = AutoBuilder.buildAutoChooser("Test Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);

                field = new Field2d();
                SmartDashboard.putData("Field", field);

                // Smart Dashboard Buttons
                SmartDashboard.putBoolean("Automation", getAutomated());
                SmartDashboard.putBoolean("Direction Chosen", scoreDirectionChosen());
                SmartDashboard.putBoolean("Algae Chosen", algaeLevelChosen());
                SmartDashboard.putBoolean("Coral Chosen", coralLevelChosen());
                SmartDashboard.putNumber("driver Y", driverController.getLeftY());
                SmartDashboard.putNumber("driver X", driverController.getLeftX());
        }

        public boolean getAutomated() {
                return automated;
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

                // // Limelight Buttons
                final Trigger limelightTrigger1 = driverController.x();
                final Trigger limelightTrigger2 = driverController.y();

                // Pressing the trigger in automation mode will run this command.
                limelightTrigger1.onTrue(new DriveToCoralBlue(driveSubsystem,
                                limelight)).and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return automated;
                                                        }
                                                });
                // Pressing the trigger NOT in automation mode will run this one.
                limelightTrigger1.onTrue(new DriveToCoralBlue(driveSubsystem,
                                limelight)).and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return !automated;
                                                        }
                                                });

                limelightTrigger2.onTrue(new DriveToCoralYellow(driveSubsystem,
                                limelight)).and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return automated;
                                                        }
                                                });
                // Pressing the trigger NOT in automation mode will run this one.
                limelightTrigger2.onTrue(new DriveToCoralYellow(driveSubsystem,
                                limelight)).and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return !automated;
                                                        }
                                                });

                // Driver operations
                final Trigger ejectCoral = driverController.b();
                final Trigger pickUpCoral = driverController.leftTrigger();
                final Trigger ejectAlgae = driverController.a();
                final Trigger shootAlgaeLeft = driverController.leftBumper();
                final Trigger shootAlgaeRight = driverController.rightBumper();
                final Trigger raiseClimber = driverController.povUp();
                final Trigger lowerClimber = driverController.povDown();
                // final Trigger intakeMotorsOff = driverController.back();
                final Trigger altButton = driverController.back();
                final Trigger toggleClimber = driverController.povLeft();

                // commands that go with driver operations
                ejectCoral.onTrue(new EjectCoral(coralSubsystem, elevatorSubsystem,
                                pivotSubsystem));
                ejectCoral.and(altButton).onTrue(new DropCoral(coralSubsystem,
                                elevatorSubsystem, pivotSubsystem));
                pickUpCoral.onTrue(new PickUpCoralFromSource(coralSubsystem,
                                elevatorSubsystem, pivotSubsystem, false));
                // pickUpCoral.and(altButton).whileTrue(new RunCommand(() ->
                // coralSubsystem.pickup(), coralSubsystem))
                // .onFalse(new SequentialCommandGroup(
                // new InstantCommand(coralSubsystem::stop),
                // new WaitCommand(0.25),
                // new Windmill(elevatorSubsystem, pivotSubsystem,
                // Constants.Windmill.WindmillState.Home, false)));
                pickUpCoral.and(altButton)
                                .onTrue(new PickUpCoralFromSource(coralSubsystem, elevatorSubsystem,
                                                pivotSubsystem,
                                                true));

                ejectAlgae.onTrue(new ScoreAlgae(algaeSubsystem));
                shootAlgaeLeft.onTrue(new ScoreAlgaeNetLeft(algaeSubsystem,
                                elevatorSubsystem, pivotSubsystem,
                                coralSubsystem));
                shootAlgaeRight.onTrue(new ScoreAlgaeNetRight(algaeSubsystem,
                                elevatorSubsystem, pivotSubsystem,
                                coralSubsystem));
                raiseClimber.onTrue(new RunCommand(() -> climberSubsystem.moveUp(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));

                toggleClimber.and(altButton).toggleOnTrue(new ConditionalCommand(
                                new DeployClimb(climberSubsystem),
                                new RetractClimb(climberSubsystem),
                                climberSubsystem::climberAtHomePosition));

                toggleClimber.and(altButton).onTrue(new LockPivotAndElevatorCommand(elevatorSubsystem,
                                pivotSubsystem)
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

                lowerClimber.onTrue(new RunCommand(() -> climberSubsystem.moveDown(),
                                climberSubsystem))
                                .onFalse(new RunCommand(() -> climberSubsystem.stop(),
                                                climberSubsystem));

                // Operator Controls
                // final Trigger manualCoral = operatorController.rightTrigger();
                // final Trigger manualAlgae = operatorController.leftTrigger();

                final Trigger moveLeft = operatorController.leftTrigger();
                final Trigger moveRight = operatorController.rightTrigger();

                moveLeft.onTrue(new InstantCommand(() -> {
                        scoreDirection = ScoreDirection.LEFT;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                // Pressing the trigger NOT in automation mode will run this one.
                moveLeft.onTrue(new SequentialCommandGroup(new ToCoralSource(elevatorSubsystem, pivotSubsystem, false),
                                new RunCommand(() -> coralSubsystem.pickup(), coralSubsystem)))
                                .onFalse(new SequentialCommandGroup(
                                                new InstantCommand(coralSubsystem::stop),
                                                new WaitCommand(0.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home, false)))
                                .and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return !automated;
                                                        }
                                                });

                moveRight.onTrue(new InstantCommand(() -> {
                        scoreDirection = ScoreDirection.RIGHT;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                moveRight.onTrue(new RunCommand(() -> algaeSubsystem.pickup(), algaeSubsystem))
                                .onFalse(new InstantCommand(algaeSubsystem::holdAlgae))
                                .and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return !automated;
                                                        }
                                                });

                // position controls
                final Trigger initializeInputs = operatorController.leftBumper();
                final Trigger algaeGround = operatorController.povDown();
                final Trigger algaeReef2 = operatorController.povRight();
                final Trigger algaeReef3 = operatorController.povUp();
                final Trigger algaeProcessor = operatorController.povLeft();
                final Trigger coralDropOff4 = operatorController.y();
                final Trigger coralDropOff3 = operatorController.x();
                final Trigger coralDropOff2 = operatorController.b();
                final Trigger coralDropOff1 = operatorController.a();
                final Trigger altPositionRight = operatorController.rightBumper();

                // initializeInputs.onTrue(new ResetOperatorInputs());

                initializeInputs.onTrue(new AutoRemoveAlgaeL3ScoreL3(driveSubsystem, elevatorSubsystem, pivotSubsystem,
                                algaeSubsystem, coralSubsystem, true));

                algaeGround.onTrue(new InstantCommand(() -> {
                        algaeLevel = AlgaeLevel.NONE;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                algaeGround.onTrue(new PickUpAlgaeFromGround(algaeSubsystem, elevatorSubsystem, pivotSubsystem)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return !automated;
                                        }
                                });

                algaeReef2.onTrue(new InstantCommand(() -> {
                        algaeLevel = AlgaeLevel.TWO;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                algaeReef2.onTrue(new PickUpAlgaeFromReef(algaeSubsystem, elevatorSubsystem, pivotSubsystem, false,
                                false)).and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return !automated;
                                                        }
                                                });
                algaeReef3.onTrue(new InstantCommand(() -> {
                        algaeLevel = AlgaeLevel.THREE;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                algaeReef3.onTrue(new PickUpAlgaeFromReef(algaeSubsystem, elevatorSubsystem, pivotSubsystem, true,
                                false)).and(
                                                new BooleanSupplier() {
                                                        @Override
                                                        public boolean getAsBoolean() {
                                                                return !automated;
                                                        }
                                                });

                algaeProcessor.onTrue(new InstantCommand(() -> {
                        algaeLevel = AlgaeLevel.NONE;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                algaeProcessor.onTrue(new ToAlgaeGround(elevatorSubsystem, pivotSubsystem)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return !automated;
                                        }
                                });

                // coralDropOff1.onTrue(new InstantCommand(() -> {
                // coralLevel = CoralLevel.ONE;
                // })).and(
                // new BooleanSupplier() {
                // @Override
                // public boolean getAsBoolean() {
                // return automated;
                // }
                // });

                coralDropOff1.onTrue(new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, false)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                coralDropOff1.onTrue(new ToCoralDropOff1(elevatorSubsystem, pivotSubsystem, false)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return !automated;
                                        }
                                });

                coralDropOff2.onTrue(new InstantCommand(() -> {
                        coralLevel = CoralLevel.TWO;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                coralDropOff2.onTrue(new ToCoralDropOff2(elevatorSubsystem, pivotSubsystem, false)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return !automated;
                                        }
                                });

                coralDropOff3.onTrue(new InstantCommand(() -> {
                        coralLevel = CoralLevel.THREE;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                coralDropOff3.onTrue(new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, false)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return !automated;
                                        }
                                });

                coralDropOff4.onTrue(new InstantCommand(() -> {
                        coralLevel = CoralLevel.FOUR;
                })).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return automated;
                                        }
                                });

                coralDropOff4.onTrue(new ScoreCoralL4(elevatorSubsystem, pivotSubsystem, coralSubsystem, false)).and(
                                new BooleanSupplier() {
                                        @Override
                                        public boolean getAsBoolean() {
                                                return !automated;
                                        }
                                });

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
                                .onTrue(new ScoreCoralL4(elevatorSubsystem, pivotSubsystem, coralSubsystem, true));

                // Set Left Joystick for manual elevator/pivot movement
                final Trigger raiseElevator = operatorController.axisLessThan(1, -0.25);
                final Trigger lowerElevator = operatorController.axisGreaterThan(1, 0.25);
                final Trigger pivotClockwise = operatorController.axisGreaterThan(4, 0.25);
                final Trigger pivotCounterClockwise = operatorController.axisLessThan(4,
                                -0.25);

                // commands that go with manual elevator/pivot movement
                pivotClockwise
                                .onTrue(new InstantCommand(pivotSubsystem::start, pivotSubsystem))
                                .onFalse(new InstantCommand(pivotSubsystem::holdPivotArmManual, pivotSubsystem));
                pivotCounterClockwise.onTrue(new InstantCommand(pivotSubsystem::reverse, pivotSubsystem))
                                .onFalse(new InstantCommand(pivotSubsystem::holdPivotArmManual, pivotSubsystem));
                raiseElevator.whileTrue(
                                new InstantCommand(elevatorSubsystem::moveUp, elevatorSubsystem)
                                                .andThen(new WaitUntilCommand(
                                                                () -> elevatorSubsystem.ElevatorRaiseLimitHit()))
                                                .andThen(new InstantCommand(elevatorSubsystem::holdHeight,
                                                                elevatorSubsystem)))
                                .onFalse(new InstantCommand(elevatorSubsystem::holdHeight, elevatorSubsystem));
                lowerElevator.whileTrue(
                                new InstantCommand(elevatorSubsystem::moveDown, elevatorSubsystem)
                                                .andThen(new WaitUntilCommand(
                                                                () -> elevatorSubsystem.ElevatorLowerLimitHit()))
                                                .andThen(new InstantCommand(elevatorSubsystem::holdHeight,
                                                                elevatorSubsystem)))
                                .onFalse(new InstantCommand(elevatorSubsystem::holdHeight,
                                                elevatorSubsystem));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // driveSubsystem::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // driveSubsystem::setModuleStates,
        // driveSubsystem);

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, 0,
        // false));
        // }

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

        public void turnOffAutomated() {
                automated = false; // Controls automation state
        }

        public void turnOnAutomated() {
                automated = true;
        }

        public boolean scoreDirectionChosen() {
                if (scoreDirection == ScoreDirection.UNDECIDED) {
                        return false;
                } else {
                        return true;
                }
        }

        public boolean algaeLevelChosen() {
                if (algaeLevel == AlgaeLevel.UNDECIDED) {
                        return false;
                } else {
                        return true;
                }
        }

        public boolean coralLevelChosen() {
                if (coralLevel == CoralLevel.UNDECIDED) {
                        return false;
                } else {
                        return true;
                }
        }
}
