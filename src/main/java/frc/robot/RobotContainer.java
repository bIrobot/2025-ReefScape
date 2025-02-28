// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IngestSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

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

  private int ticks = 0;

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
        m_ArmSubsystem.fingerGrab();
    } else if (m_driverController.getBackButtonReleased()) {
        m_ArmSubsystem.fingerStop();
    } else if (m_driverController.getStartButtonPressed()) {
        m_ArmSubsystem.fingerRelease();
    }

    int pov = m_driverController.getPOV();
    if (pov != lastPov) {
        if (pov == 0) {
            m_ArmSubsystem.armGoto(ArmState.LEVEL1);
            m_ElevatorSubsystem.elevatorGoto(ElevatorState.LEVEL1);
        } else if (pov == 90) {
            m_ArmSubsystem.armGoto(ArmState.LEVEL2);
            m_ElevatorSubsystem.elevatorGoto(ElevatorState.LEVEL2);
        } else if (pov == 180) {
            m_ArmSubsystem.armGoto(ArmState.LEVEL3);
            m_ElevatorSubsystem.elevatorGoto(ElevatorState.LEVEL3);
        } else if (pov == 270) {
            m_ArmSubsystem.armGoto(ArmState.LEVEL4);
            m_ElevatorSubsystem.elevatorGoto(ElevatorState.LEVEL4);
        }
        lastPov = pov;
    }
}

public void testRunning()
{
  m_ArmSubsystem.armCoast();
  m_ElevatorSubsystem.elevatorCoast();
  m_IngestSubsystem.ingestCoast();
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
