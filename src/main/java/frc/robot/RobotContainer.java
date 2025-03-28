// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TestPosition.TestState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private int m_lastPov = -1;
  private int m_lastPose = 0;

  private boolean m_toggle = false;
  private double m_sign = 0.0;

  private int ticks = 0;

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand("drive", gotoCommand(4, false));

    NamedCommands.registerCommand("L1", gotoCommand(0, false));
    NamedCommands.registerCommand("L2", gotoCommand(1, false));
    NamedCommands.registerCommand("L3", gotoCommand(2, false));
    NamedCommands.registerCommand("L4", gotoCommand(3, false));

    NamedCommands.registerCommand("L1DR", gotoCommand(0, true));
    NamedCommands.registerCommand("L2DR", gotoCommand(1, true));
    NamedCommands.registerCommand("L3DR", gotoCommand(2, true));
    NamedCommands.registerCommand("L4DR", gotoCommand(3, true));

    NamedCommands.registerCommand("ingest", gotoCommand(5, false));
    NamedCommands.registerCommand("flooringest", gotoCommand(6, false));
    NamedCommands.registerCommand("aprilleft", getfindAprilLeftCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false),
            m_robotDrive));
  }

  public void teleopRunning() {
    // if elevator calibration failed...
    if (m_ElevatorSubsystem.elevatorCalibrationFailed) {
        // don't take input from the controller
        return;
    }

    // the X (blue, left) and B (red, right) buttons control ingest
    if (m_driverController.getXButtonPressed()) {
        // set human player ingest elevator/arm/hand position
        RobotGoto(5);
    } else if (m_driverController.getBButtonPressed()) {
        // swivel for scoring on reef
        m_ArmSubsystem.fingerStop();
        if (m_toggle) {
            m_ArmSubsystem.swivelPlus();
        } else {
            m_ArmSubsystem.swivelMinus();
        }
        m_toggle = ! m_toggle;
    }

    // the elevator up/down is controlled by the left bumper/left trigger
    if (m_driverController.getLeftBumperButton()) {
        m_ElevatorSubsystem.elevatorUp();
    } else if (m_driverController.getLeftTriggerAxis() > .1) {
        m_ElevatorSubsystem.elevatorDown();
    } else {
        m_ElevatorSubsystem.elevatorStop();
    }

    // the arm up/down is controlled by the right bumper/right trigger
    if (m_driverController.getRightBumperButton()) {
        m_ArmSubsystem.armUp();
    } else if (m_driverController.getRightTriggerAxis() > .1) {
        m_ArmSubsystem.armDown();
    } else {
        m_ArmSubsystem.armStop();
    }

    // the hand up/down is controlled by the Y (yellow, up) and A (green, down) buttons
    if (m_driverController.getYButton()) {
        m_ArmSubsystem.handUp();
    } else if (m_driverController.getAButton()) {
        m_ArmSubsystem.handDown();
    } else {
        m_ArmSubsystem.handStop();
    }

    // the "back" (left) and "start" (right) buttons control the fingers
    if (m_driverController.getBackButton()) {
        // start grabbing on "back" until the button is released (we need a beam break!)
        m_ArmSubsystem.fingerGrab();
    } else if (m_driverController.getBackButtonReleased()) {
        // stop grabbing wehen the button is released
        m_ArmSubsystem.fingerStop();
    } else if (m_driverController.getStartButtonPressed()) {
        // reverse fingers on "start" for 1/10 second
        m_ArmSubsystem.fingerRelease();
    }

    if (m_driverController.getLeftStickButtonPressed()) {
        RobotGoto(4);
    }

    if (m_driverController.getRightStickButtonPressed()) {
        //gotoCommand(m_lastPose, true).schedule();
        gotoCommand(6, false).schedule();
    }

    // the POV control selects preset poses 1 (north), 2 (east), 3 (south), 4 (west)
    int pov = m_driverController.getPOV();
    if (pov != m_lastPov) {
        if (pov%90 == 0) {
            int pose = pov/90;
            RobotGoto(pose);
            m_lastPose = pose;
        }
        m_lastPov = pov;
    }

    if (ticks++%100==0) System.out.println("TELEOP RUNNING");
}

public Command gotoCommand(int pose, boolean downRelease)
{
    // N.B. we use DeferredCommand() so that testElevatorPosition() is called at Command run time, rather than at construction time
    return new DeferredCommand(() -> {
        double armPos;
        double handPos;
        double elevatorPos;
        double armDelay = 0;
        double elevatorDelay = 0;

        if (! downRelease) {
            if (pose == 0) {
                m_ArmSubsystem.swivelZero();
                m_toggle = true;
            } else if (pose == 1 || pose == 2 || pose == 3) {
                m_ArmSubsystem.swivelMinus();
                m_toggle = true;
            } else if (pose == 5) {
                m_ArmSubsystem.swivelZero();
                m_ArmSubsystem.fingerGrab();
                m_toggle = false;
            } else if (pose == 6) {
                m_ArmSubsystem.swivelMinus();
                m_ArmSubsystem.fingerGrab();
                m_toggle = false;
            }
        }

        // get position targets for arm, hand, and elevator
        armPos = PoseConstants.poses[pose][0];
        handPos = PoseConstants.poses[pose][1] - ((downRelease && pose != 0)?0.08:0);
        elevatorPos = PoseConstants.poses[pose][2] - ((downRelease && pose != 0)?0.24:0);

        // test the elevator target direction
        TestState state = m_ElevatorSubsystem.testElevatorPosition(elevatorPos);
        if (state == TestState.GOING_UP) {
            // going up -- move elevator first!
            armDelay = 1;
        } else if (state == TestState.GOING_DOWN) {
            // going down -- move arm/hand first!
            elevatorDelay = 1;
        }

        // set new targets giving elevator or arm/hand a chance to move first
        Command command = new ParallelCommandGroup(
                new SequentialCommandGroup(new WaitCommand(armDelay),
                                            new InstantCommand(() -> { m_ArmSubsystem.armGoto(armPos); }, m_ArmSubsystem)),

                new SequentialCommandGroup(new WaitCommand(elevatorDelay),
                                            new InstantCommand(() -> { m_ElevatorSubsystem.elevatorGoto(elevatorPos); }, m_ElevatorSubsystem))
        );
        if (downRelease) {
            command = new SequentialCommandGroup(new InstantCommand(() -> { m_ArmSubsystem.handGoto(handPos); }, m_ArmSubsystem ),
                                                 command,
                                                 new WaitCommand(0.5),
                                                 new InstantCommand(() -> { m_ArmSubsystem.fingerRelease(); }, m_ArmSubsystem));
        } else {
            command = new SequentialCommandGroup(new InstantCommand(() -> { m_ArmSubsystem.handGoto(handPos); }, m_ArmSubsystem ),
                                                 command);
        }
        return command;
    }, Set.of(m_ArmSubsystem, m_ElevatorSubsystem));
}

public void RobotGoto(int pose)
{
    // cancel any previous targets
    m_ArmSubsystem.armHold();
    m_ElevatorSubsystem.elevatorHold();

    // schedule the new pose
    gotoCommand(pose, false).schedule();
}

// return true if still moving
public boolean RobotTestMoving(int pose)
{
    double armPos;
    double handPos;
    double elevatorPos;
    TestState armState;
    TestState handState;
    TestState elevatorState;

    // get position targets for arm, hand, and elevator
    armPos = PoseConstants.poses[pose][0];
    handPos = PoseConstants.poses[pose][1];
    elevatorPos = PoseConstants.poses[pose][2];

    armState = m_ArmSubsystem.testArmPosition(armPos);
    handState = m_ArmSubsystem.testHandPosition(handPos);
    elevatorState = m_ElevatorSubsystem.testElevatorPosition(elevatorPos);

    return elevatorState != TestState.TARGET_ACHIEVED || armState != TestState.TARGET_ACHIEVED || handState != TestState.TARGET_ACHIEVED;
}

public void findAprilInit(double angle)
{
    double TA = LimelightHelpers.getTA("limelight");
    double TX = LimelightHelpers.getTX("limelight");

    // if seeing target
    if (TA != 0) {
        if (TX - angle < 0) {
            // target is on left
            System.out.println("Slide left slow " + TX);
            m_robotDrive.drive(0, RobotConstants.k_moveSpeed, 0, false);
        } else {
            // target is on right
            System.out.println("Slide right slow " + TX);
            m_robotDrive.drive(0, -RobotConstants.k_moveSpeed, 0, false);
        }
    } else {
        System.out.println("NOT SEEN");
        m_robotDrive.drive(0, 0, 0, false);
    }
}

public boolean findAprilFinished(double angle)
{
    double TA = LimelightHelpers.getTA("limelight");
    double TX = LimelightHelpers.getTX("limelight");

    if (m_sign * (TX - angle) < 0.0 || TA == 0.0 || Math.abs(TX - angle) <= 0.3) {
        m_sign = 0.0;
        System.out.println("Stop " + TX);
        m_robotDrive.drive(0, 0, 0, false);
        return true;
    }
    m_sign = TX - angle;

    return false;
}

public Command getfindAprilLeftCommand() {
    return new FunctionalCommand (() -> { findAprilInit(RobotConstants.k_leftAngle); },  // onInit
                                    () -> { },  // onExecute
                                    (interrupted) -> { m_robotDrive.drive(0, 0, 0, false); },  // onEnd
                                    () -> { return findAprilFinished(RobotConstants.k_leftAngle); },  // isFinished
                                    m_robotDrive);
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
    /*
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
    /*
    return new PathPlannerAuto("test");
    */
    /*
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_robotDrive.drive(0.1, 0, 0, false), m_robotDrive),
        new WaitCommand(3),
        new InstantCommand(() -> m_robotDrive.drive(0.1, 0, 0, false), m_robotDrive));
        */
  }
}
