// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class TestPosition {
    public enum TestState {
      GOING_UP,
      TARGET_ACHIEVED,
      GOING_DOWN
    };
  }

  public static final class PoseConstants {
    public static final double[][] poses = {
      // armPos: bottom = 0.43; top = 0.05 -- note top is smaller number than bottom!
      // handPos: bottom = 0.30; top = 0.60
      // elevatorPos: bottom = 0.00; top = 4.00
      //
      // [0]:armPos, [1]:handPos, [2]:elevatorPos
      { 0.064, 0.201, 0.000 },  // [0]:pose 1
      { 0.216, 0.579, 0.985 },  // [1]:pose 2
      { 0.274, 0.633, 3.180 },  // [2]:pose 3
      { 0.054, 0.302, 4.055 },  // [3]:pose 4
      { 0.040, 0.420, 0.000 },  // [4]:pose 5 -- drive  left stick
      { 0.044, 0.377, 0.000 }  // [5]:pose 6 -- ingest  X
    };
  };

  // this should not be here!
  public static final double Round(double x)
  {
    return Math.round(x * 1000.0) / 1000.0;
  }

  public static final class ArmConstants {
    public static final double kArmUpSpeed = 0.30;
    public static final double kArmDownSpeed = 0.25;

    public static final double kArmLevelSafe = 0.11;
    public static final double kArmLevelBottom = 0.43;  // soft limit
    public static final double kArmLevelTop = 0.04;  // soft limit NOT 0

    public static final double kArmTestClose = 0.01;

    public static final double kSwivelSpeed = 0.1;

    public static final double kHandUpSpeed = 0.7;
    public static final double kHandDownSpeed = 0.5;

    public static final double kHandLevelSafe = 0.45;
    public static final double kHandLevelBottom = 0.20;  // soft limit NOT 0
    public static final double kHandLevelTop = 0.70;  // soft limit

    public static final double kHandTestClose = 0.01;

    public static final double kFingerGrabSpeed = 1.0;
    public static final double kFingerReleaseSpeed = 0.5;

    public static final double k_reverseNsec = 200000000;
  }

  public static final class ElevatorConstants {
    public static final double kElevatorUpSpeed = 1.00;
    public static final double kElevatorDownSpeed = 0.75;

    public static final double kElevatorLevelSafe = 3.00;
    public static final double kElevatorLevelBottom = 0.00;  // soft limit (0 allowed)
    public static final double kElevatorLevelTop = 4.20;  // soft limit

    public static final double kElevatorTestClose = 0.05;
  }
  
  public static final class RobotConstants {
    public static final double k_raiseNsec = 1000000000.0;

    public static final double k_moveSpeed = 0.02;

    public static final double k_leftAngle = -5;  // degrees from apriltag view to left reef post
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.375);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.625);
    // Distance between front and back wheels on robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
