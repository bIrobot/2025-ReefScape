package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IngestConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_armMotorLeft;
    private final SparkMax m_armMotorRight;
    private SparkAbsoluteEncoder m_armEncoder;

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

    private enum ArmState {
        STOP,
        UP,
        DOWN
        // XXX -- real state targets, not motions
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
        m_armMotorLeft = new SparkMax(11, MotorType.kBrushless);  // XXX Constants
        m_armEncoder = m_armMotorLeft.getAbsoluteEncoder();

        m_armMotorRight = new SparkMax(12, MotorType.kBrushless);  // XXX Constants

        m_handMotor = new SparkMax(13, MotorType.kBrushless);  // XXX Constants
        m_handEncoder = m_handMotor.getAbsoluteEncoder();

        m_fingerMotor = new SparkMax(14, MotorType.kBrushless);  // XXX Constants

        SparkMaxConfig configArmLeft = new SparkMaxConfig();
        configArmLeft.idleMode(IdleMode.kBrake);
        m_armMotorLeft.configure(configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configArmRight = new SparkMaxConfig();
        configArmRight.idleMode(IdleMode.kBrake);
        m_armMotorRight.configure(configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configHand = new SparkMaxConfig();
        configHand.idleMode(IdleMode.kBrake);
        m_handMotor.configure(configHand, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getArmEncoderFraction()
    {
        double value = m_armEncoder.getPosition();
        if (value > ArmConstants.k_armAngleWrapFraction) {
            return 0;
        }
        if (value > ArmConstants.k_armAngleMaxFraction) {
            return ArmConstants.k_armAngleMaxFraction;
        }
        return value;
    }

    private double getArmTargetFraction()
    {
        assert(false);
        return 0.0;
    }

    public void armStop()
    {
        m_currentArmState = ArmState.STOP;
    }

    public void armUp()
    {
        m_currentArmState = ArmState.UP;
    }

    public void armDown()
    {
        m_currentArmState = ArmState.DOWN;
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

    public void fingerGrab()
    {
        m_currentFingerState = FingerState.GRAB;
    }

    public void fingerStop()
    {
        m_currentFingerState = FingerState.STOP;
    }

    public void fingerRelease()
    {
        m_FingerTime = System.nanoTime();
        m_currentFingerState = FingerState.RELEASE;

    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("ARM: Arm Encoder: " + m_armEncoder.getPosition() +
                                              " Hand Encoder: " + m_handEncoder.getPosition());

        setArmMotorToTarget();
        setHandMotorToTarget();
        setFingerMotorToTarget();
    }

    private void setArmMotorToTarget() {
        switch (m_currentArmState){
            case STOP:
                m_armMotorLeft.set(0.0);
                m_armMotorRight.set(0.0);
                break;
            case UP:
                m_armMotorLeft.set(ArmConstants.kArmUpSpeed);
                m_armMotorRight.set(-ArmConstants.kArmUpSpeed);
                break;
            case DOWN:
                m_armMotorLeft.set(-ArmConstants.kArmDownSpeed);
                m_armMotorRight.set(ArmConstants.kArmDownSpeed);
                break;
            default:
                assert(false);
                break;
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
