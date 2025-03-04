// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PoseConstants;
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

  private int lastPov = -1;

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
    if (m_ElevatorSubsystem.elevatorCalibrationFailed) {
        return;
    }

    if (m_driverController.getXButtonPressed()) {
        m_IngestSubsystem.startIngesting();
    } else if (m_driverController.getXButtonReleased()){
        m_IngestSubsystem.stopIngesting();
    } else if (m_driverController.getBButtonPressed()) {
        m_IngestSubsystem.reverseIngesting();
    }

    if (m_driverController.getLeftBumperButton()) {
        m_ElevatorSubsystem.elevatorUp();
    } else if (m_driverController.getLeftTriggerAxis() > .1) {
        m_ElevatorSubsystem.elevatorDown();
    } else {
        m_ElevatorSubsystem.elevatorStop();
    }

    if (m_driverController.getRightBumperButton()) {
        m_ArmSubsystem.armUp();
    } else if (m_driverController.getRightTriggerAxis() > .1) {
        m_ArmSubsystem.armDown();
    } else {
        m_ArmSubsystem.armStop();
    }

    if (m_driverController.getYButton()) {
        m_ArmSubsystem.handUp();
    } else if (m_driverController.getAButton()) {
        m_ArmSubsystem.handDown();
    } else {
        m_ArmSubsystem.handStop();
    }

    if (m_driverController.getBackButton()) {
        m_IngestSubsystem.handoffIngesting();  // XXX move this bring back for handoff
        m_ArmSubsystem.fingerGrab();
    } else if (m_driverController.getBackButtonReleased()) {
        m_ArmSubsystem.fingerStop();
    } else if (m_driverController.getStartButtonPressed()) {
        m_ArmSubsystem.fingerRelease();
    }

    int pov = m_driverController.getPOV();
    do {
        if (pov != lastPov) {
            int pose;
            double armPos;
            double handPos;
            double elevatorPos;
            int armDelay = 0;
            int elevatorDelay = 0;

            if (pov%90 != 0) {
                break;
            }
            pose = pov/90;

            armPos = PoseConstants.poses[pose][0];
            handPos = PoseConstants.poses[pose][1];
            elevatorPos = PoseConstants.poses[pose][2];

            if (m_ElevatorSubsystem.willElevatorGoUp(elevatorPos)) {
                // going up -- move elevator first!
                armDelay = 2;
            } else {
                // going down -- move arm first!
                elevatorDelay = 2;
            }

            m_ArmSubsystem.armHold();
            m_ElevatorSubsystem.elevatorHold();
            ParallelCommandGroup commands = new ParallelCommandGroup(
                new SequentialCommandGroup(new WaitCommand(armDelay),
                                        new InstantCommand(() -> { m_ArmSubsystem.armGoto(armPos);
                                                                    m_ArmSubsystem.handGoto(handPos); }, m_ArmSubsystem)),

                new SequentialCommandGroup(new WaitCommand(elevatorDelay),
                                        new InstantCommand(() -> m_ElevatorSubsystem.elevatorGoto(elevatorPos), m_ElevatorSubsystem))
            );
            commands.schedule();

            lastPov = pov;
        }
    } while (false);
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
