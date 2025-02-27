package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IngestConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_ArmMotorLeft;
    private final SparkMax m_ArmMotorRight;
    private SparkAbsoluteEncoder m_ArmEncoder;

    private final SparkMax m_handMotor;
    private SparkAbsoluteEncoder m_handEncoder;

    private final SparkMax m_fingerMotor;

    private ArmState m_currentArmState = ArmState.STOP;
    private double m_ArmTime = 0;
    private HandState m_currentHandState = HandState.STOP;
    private double m_HandTime = 0;
    private FingerState m_currentFingerState = FingerState.STOP;
    private double m_FingerTime = 0;

    private int ticks = 0;

    public enum ArmState {  // XXX -- should be private!
        STOP,
        UP,
        DOWN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4
    };
  
    private enum HandState {
        STOP,
        UP,
        DOWN
        // XXX -- real state targets, not motions
    };

    private enum FingerState {
        STOP,
        GRAB,
        RELEASE
    };

    public ArmSubsystem()
    {
        m_ArmMotorLeft = new SparkMax(11, MotorType.kBrushless);  // XXX Constants
        m_ArmEncoder = m_ArmMotorLeft.getAbsoluteEncoder();

        m_ArmMotorRight = new SparkMax(12, MotorType.kBrushless);  // XXX Constants

        m_handMotor = new SparkMax(13, MotorType.kBrushless);  // XXX Constants
        m_handEncoder = m_handMotor.getAbsoluteEncoder();

        m_fingerMotor = new SparkMax(14, MotorType.kBrushless);  // XXX Constants

        SparkMaxConfig configArmLeft = new SparkMaxConfig();
        configArmLeft.idleMode(IdleMode.kBrake);
        m_ArmMotorLeft.configure(configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configArmRight = new SparkMaxConfig();
        configArmRight.idleMode(IdleMode.kBrake);
        m_ArmMotorRight.configure(configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configHand = new SparkMaxConfig();
        configHand.idleMode(IdleMode.kBrake);
        m_handMotor.configure(configHand, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // N.B. I don't believe we can use a PID controller because we drive Left/Right motors based on a single encoder
    }

    public double getArmEncoderFraction()
    {
        double value = m_ArmEncoder.getPosition();
        if (value > ArmConstants.k_armAngleWrapFraction) {
            return 0;
        }
        if (value > ArmConstants.k_armAngleMaxFraction) {
            return ArmConstants.k_armAngleMaxFraction;
        }
        return value;
    }

    public void armStop()
    {
        if (m_currentArmState == ArmState.UP || m_currentArmState == ArmState.DOWN) {
            m_currentArmState = ArmState.STOP;
        }
    }

    public void armUp()
    {
        m_currentArmState = ArmState.UP;
    }

    public void armDown()
    {
        m_currentArmState = ArmState.DOWN;
    }

    public void armGoto(ArmState level)  // XXX -- horrible api to have illegal enum values!
    {
        m_currentArmState = level;
    }

    public void handStop()
    {
        m_currentHandState = HandState.STOP;
    }

    public void handUp()
    {
        m_currentHandState = HandState.UP;
    }

    public void handDown()
    {
        m_currentHandState = HandState.DOWN;
    }

    public void fingerStop()
    {
        m_currentFingerState = FingerState.STOP;
    }

    public void fingerGrab()
    {
        m_currentFingerState = FingerState.GRAB;
    }

    public void fingerRelease()
    {
        m_FingerTime = System.nanoTime();
        m_currentFingerState = FingerState.RELEASE;

    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("ARM: Arm Encoder: " + m_ArmEncoder.getPosition() +
                                              " Hand Encoder: " + m_handEncoder.getPosition());

        setArmMotorToTarget();
        setHandMotorToTarget();
        setFingerMotorToTarget();
    }

    private void setArmMotorToTarget() {
        switch (m_currentArmState){
            case STOP:
                m_ArmMotorLeft.set(0.0);
                m_ArmMotorRight.set(0.0);
                break;
            case UP:
                m_ArmMotorLeft.set(ArmConstants.kArmUpSpeed);
                m_ArmMotorRight.set(-ArmConstants.kArmUpSpeed);
                break;
            case DOWN:
                m_ArmMotorLeft.set(-ArmConstants.kArmDownSpeed);
                m_ArmMotorRight.set(ArmConstants.kArmDownSpeed);
                break;
            case LEVEL1:
                setMotorsLevel(ArmConstants.kArmLevel1);
                break;
            case LEVEL2:
                setMotorsLevel(ArmConstants.kArmLevel2);
                break;
            case LEVEL3:
                setMotorsLevel(ArmConstants.kArmLevel3);
                break;
            case LEVEL4:
                setMotorsLevel(ArmConstants.kArmLevel4);
                break;
            default:
                assert(false);
                break;
        }
    }

    // poor man's PID controller
    private void setMotorsLevel(double pos) {
        double sign;
        double diff;

        sign = pos-getArmEncoderFraction();
        diff = Math.abs(sign);

        if (diff < 0.02) {
            if (pos < 0.1) {
                m_ArmMotorLeft.set(0.0);
                m_ArmMotorRight.set(0.0);
            } else {
                // resist gravity
                m_ArmMotorLeft.set(ArmConstants.kArmUpSpeed/4);
                m_ArmMotorRight.set(-ArmConstants.kArmUpSpeed/4);
            }
    } else if (diff < 0.2) {
            if (sign > 0) {
                // down slow
                m_ArmMotorLeft.set(-ArmConstants.kArmDownSpeed/2);
                m_ArmMotorRight.set(ArmConstants.kArmDownSpeed/2);
            } else {
                // up slow
                m_ArmMotorLeft.set(ArmConstants.kArmUpSpeed/2);
                m_ArmMotorRight.set(-ArmConstants.kArmUpSpeed/2);
            }
        } else {
            if (sign > 0) {
                // down
                m_ArmMotorLeft.set(-ArmConstants.kArmDownSpeed);
                m_ArmMotorRight.set(ArmConstants.kArmDownSpeed);
            } else {
                // up
                m_ArmMotorLeft.set(ArmConstants.kArmUpSpeed);
                m_ArmMotorRight.set(-ArmConstants.kArmUpSpeed);
            }
        }
    }

    private void setHandMotorToTarget() {
        switch (m_currentHandState){
            case STOP:
                m_handMotor.set(0.0);
                break;
            case UP:
                m_handMotor.set(-ArmConstants.kHandUpSpeed);
                break;
            case DOWN:
                m_handMotor.set(ArmConstants.kHandDownSpeed);
                break;
            default:
                assert(false);
                break;
        }
    }

    private void setFingerMotorToTarget() {
        switch (m_currentFingerState){
            case STOP:
                m_fingerMotor.set(0.0);
                break;
            case GRAB:
                m_fingerMotor.set(ArmConstants.kFingerGrabSpeed);
                break;
            case RELEASE:
                if (System.nanoTime() - m_FingerTime > ArmConstants.k_reverseNsec) {
                    m_currentFingerState = FingerState.STOP;
                } else {
                    m_fingerMotor.set(-ArmConstants.kFingerReleaseSpeed);
                }
                break;
            default:
                assert(false);
                break;
        }
    }
}
