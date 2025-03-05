// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TestPosition.TestState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IngestSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final IngestSubsystem m_IngestSubsystem = new IngestSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private int m_lastPov = -1;
  private boolean m_lastHandoffReady = false;

  private int ticks = 0;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

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
    boolean test = false;
    boolean thisHandoffReady;

    // if elevator calibration failed...
    if (m_ElevatorSubsystem.elevatorCalibrationFailed) {
        // don't take input from the controller
        return;
    }

    // the X (blue, left) and B (red, right) buttons control ingest
    if (m_driverController.getXButtonPressed()) {
        // start ingesting on X until either the beam breaks or the button is released
        m_IngestSubsystem.startIngesting();
    } else if (m_driverController.getXButtonReleased()) {
        // stop ingesting wehen the button is released
        m_IngestSubsystem.stopIngesting();
    } else if (m_driverController.getBButtonPressed()) {
        // reverse ingest on B for 1/10 second
        m_IngestSubsystem.reverseIngesting();
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
        m_IngestSubsystem.handoffIngesting();  // XXX move this -- bring back pivot for handoff
        m_ArmSubsystem.fingerGrab();
    } else if (m_driverController.getBackButtonReleased()) {
        // stop grabbing wehen the button is released
        m_ArmSubsystem.fingerStop();
    } else if (m_driverController.getStartButtonPressed()) {
        // reverse ingest on "start" for 1/10 second
        m_ArmSubsystem.fingerRelease();
    }

    if (m_driverController.getLeftStickButtonPressed()) {
        RobotGoto(4);
    }

    if (m_driverController.getRightStickButtonPressed()) {
        RobotGoto(5);
    }

    // the POV control selects preset poses 1 (north), 2 (east), 3 (south), 4 (west)
    int pov = m_driverController.getPOV();
    if (pov != m_lastPov) {
        if (pov%90 == 0) {
            int pose = pov/90;
            RobotGoto(pose);
        }
        m_lastPov = pov;
    }

    thisHandoffReady = m_IngestSubsystem.getIngestHandoffReady();
    if (thisHandoffReady && ! m_lastHandoffReady) {
        // we just ingested coral; goto handoff position
        RobotGoto(5);
        test = true;
    }
    m_lastHandoffReady = thisHandoffReady;

    if (test) {
        test = RobotTestMoving(5);
    }
}

public void RobotGoto(int pose)
{
    double armPos;
    double handPos;
    double elevatorPos;
    int armDelay = 0;
    int elevatorDelay = 0;

    // get position targets for arm, hand, and elevator
    armPos = PoseConstants.poses[pose][0];
    handPos = PoseConstants.poses[pose][1];
    elevatorPos = PoseConstants.poses[pose][2];

    // test the elevator target direction
    TestState state = m_ElevatorSubsystem.testElevatorPosition(elevatorPos);
    if (state == TestState.GOING_UP) {
        // going up -- move elevator first!
        armDelay = 2;
    } else if (state == TestState.GOING_DOWN) {
        // going down -- move arm/hand first!
        elevatorDelay = 2;
    }

    // cancel any previous targets
    m_ArmSubsystem.armHold();
    m_ElevatorSubsystem.elevatorHold();

    // set new targets giving elevator or arm/hand a chance to move first
    ParallelCommandGroup commands = new ParallelCommandGroup(
        new SequentialCommandGroup(new WaitCommand(armDelay),
                                   new InstantCommand(() -> { m_ArmSubsystem.armGoto(armPos);
                                                              m_ArmSubsystem.handGoto(handPos); }, m_ArmSubsystem)),

        new SequentialCommandGroup(new WaitCommand(elevatorDelay),
                                   new InstantCommand(() -> m_ElevatorSubsystem.elevatorGoto(elevatorPos), m_ElevatorSubsystem))
    );
    commands.schedule();
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

    if (ticks++%10==0) System.out.println("ELEVATOR: " + elevatorState +
                                          " ARM: " + armState +
                                          " HAND:"  + handState);

    return elevatorState != TestState.TARGET_ACHIEVED || armState != TestState.TARGET_ACHIEVED || handState != TestState.TARGET_ACHIEVED;
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
  public Command getAutonomousCommand() {
    return null;
  }
}
