package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// XXX - rename hand to wrist

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_ArmMotorLeft;
    private SparkAbsoluteEncoder m_ArmEncoder;
    private SparkClosedLoopController m_ArmController;

    private final SparkMax m_ArmMotorRight;

    private final SparkMax m_handMotor;
    private SparkAbsoluteEncoder m_handEncoder;
    private SparkClosedLoopController m_HandController;

    private final SparkMax m_fingerMotor;

    private ArmState m_currentArmState = ArmState.STOP;
    private HandState m_currentHandState = HandState.STOP;
    private FingerState m_currentFingerState = FingerState.STOP;
    private double m_FingerTime = 0;

    // slot 0 for position control
    private static final double k_armMotorP = 4.0;
    private static final double k_armMotorI = 0.0;
    private static final double k_armMotorD = 4.0;

    // slot 1 for velocity control
    private static final double k_armMotorP1 = 1.0;
    private static final double k_armMotorI1 = 0.0;
    private static final double k_armMotorD1 = 8.0;

    // slot 0 for position control
    private static final double k_handMotorP = 4.0;
    private static final double k_handMotorI = 0.0;
    private static final double k_handMotorD = 2.0;

    // slot 1 for velocity control
    private static final double k_handMotorP1 = 1.0;
    private static final double k_handMotorI1 = 0.0;
    private static final double k_handMotorD1 = 2.0;

    private int ticks = 0;

    public enum ArmState {  // XXX -- should be private!
        STOP,
        UP,
        DOWN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4,
        TOP
    };
  
    public enum HandState {
        STOP,
        UP,
        DOWN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4,
        STRAIGHT
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
        m_ArmController = m_ArmMotorLeft.getClosedLoopController();

        m_ArmMotorRight = new SparkMax(12, MotorType.kBrushless);  // XXX Constants

        m_handMotor = new SparkMax(13, MotorType.kBrushless);  // XXX Constants
        m_handEncoder = m_handMotor.getAbsoluteEncoder();
        m_HandController = m_handMotor.getClosedLoopController();

        m_fingerMotor = new SparkMax(14, MotorType.kBrushless);  // XXX Constants

        // left motor has encoder and controller
        SparkMaxConfig configArmLeft = new SparkMaxConfig();
        configArmLeft.inverted(true);
        configArmLeft.closedLoop.pid(k_armMotorP, k_armMotorI, k_armMotorD)
                                .pid(k_armMotorP1, k_armMotorI1, k_armMotorD1, ClosedLoopSlot.kSlot1)
                                .positionWrappingEnabled(true)
                                .positionWrappingInputRange(0, 1)
                                .outputRange(-ArmConstants.kArmUpSpeed, ArmConstants.kArmDownSpeed)
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        configArmLeft.idleMode(IdleMode.kBrake);
        m_ArmMotorLeft.configure(configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // right motor follows left motor with inverted direction
        SparkMaxConfig configArmRight = new SparkMaxConfig();
        configArmRight.inverted(true);
        configArmRight.idleMode(IdleMode.kBrake)
                      .follow(11, true);  // XXX Constants
        m_ArmMotorRight.configure(configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // initial arm position
        m_ArmController.setReference(m_ArmEncoder.getPosition(), ControlType.kPosition);

        SparkMaxConfig configHand = new SparkMaxConfig();
        configHand.inverted(true);
        configHand.closedLoop.pid(k_handMotorP, k_handMotorI, k_handMotorD)
                             .pid(k_handMotorP1, k_handMotorI1, k_handMotorD1, ClosedLoopSlot.kSlot1)
                             .positionWrappingEnabled(true)
                             .positionWrappingInputRange(0, 1)
                             .outputRange(-ArmConstants.kHandUpSpeed, ArmConstants.kHandDownSpeed)
                             .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        configHand.idleMode(IdleMode.kBrake);
        m_handMotor.configure(configHand, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // initial hand position
        m_HandController.setReference(m_handEncoder.getPosition(), ControlType.kPosition);
    }

    public void armCoast()
    {
        SparkMaxConfig configArmLeft = new SparkMaxConfig();
        configArmLeft.idleMode(IdleMode.kCoast);
        m_ArmMotorLeft.configure(configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configArmRight = new SparkMaxConfig();
        configArmRight.idleMode(IdleMode.kCoast);
        m_ArmMotorRight.configure(configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configHand = new SparkMaxConfig();
        configHand.idleMode(IdleMode.kBrake);
        m_handMotor.configure(configHand, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void armStop()
    {
        if (m_currentArmState == ArmState.UP || m_currentArmState == ArmState.DOWN) {
            m_currentArmState = ArmState.STOP;
            m_ArmController.setReference(m_ArmEncoder.getPosition(), ControlType.kPosition);
        }
    }

    public void armUp()
    {
        m_currentArmState = ArmState.UP;
        m_ArmController.setReference(-ArmConstants.kArmUpSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void armDown()
    {
        m_currentArmState = ArmState.DOWN;
        m_ArmController.setReference(ArmConstants.kArmDownSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void armGoto(ArmState level)  // XXX -- horrible api to have illegal enum values!
    {
        m_currentArmState = level;
        switch (level) {
            case LEVEL1:
                m_ArmController.setReference(ArmConstants.kArmLevel1, ControlType.kPosition);
                break;
            case LEVEL2:
                m_ArmController.setReference(ArmConstants.kArmLevel2, ControlType.kPosition);
                break;
            case LEVEL3:
                m_ArmController.setReference(ArmConstants.kArmLevel3, ControlType.kPosition);
                break;
            case LEVEL4:
                m_ArmController.setReference(ArmConstants.kArmLevel4, ControlType.kPosition);
                break;
            case TOP:
                m_ArmController.setReference(ArmConstants.kArmLevelTop, ControlType.kPosition);
                break;
            default:
                assert(false);
                break;
        }
    }

    public void handStop()
    {
        if (m_currentHandState == HandState.UP || m_currentHandState == HandState.DOWN) {
            m_currentHandState = HandState.STOP;
            m_HandController.setReference(m_handEncoder.getPosition(), ControlType.kPosition);
        }
    }

    public void handUp()
    {
        m_currentHandState = HandState.UP;
        m_HandController.setReference(ArmConstants.kHandUpSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void handDown()
    {
        m_currentHandState = HandState.DOWN;
        m_HandController.setReference(-ArmConstants.kHandDownSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void handGoto(HandState level)  // XXX -- horrible api to have illegal enum values!
    {
        m_currentHandState = level;
        switch (level) {
            case LEVEL1:
                m_HandController.setReference(ArmConstants.kHandLevel1, ControlType.kPosition);
                break;
            case LEVEL2:
                m_HandController.setReference(ArmConstants.kHandLevel2, ControlType.kPosition);
                break;
            case LEVEL3:
                m_HandController.setReference(ArmConstants.kHandLevel3, ControlType.kPosition);
                break;
            case LEVEL4:
                m_HandController.setReference(ArmConstants.kHandLevel4, ControlType.kPosition);
                break;
            case STRAIGHT:
                m_HandController.setReference(ArmConstants.kHandLevelStraight, ControlType.kPosition);
                break;
            default:
                assert(false);
                break;
        }
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

        if (DriverStation.isTest()) {
            m_ArmController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
            m_HandController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
            m_fingerMotor.set(0);
            armCoast();
            return;
        }

        if (m_handEncoder.getPosition() < ArmConstants.kHandLevelBottom || m_handEncoder.getPosition() > ArmConstants.kHandLevelTop) {
            m_HandController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        setFingerMotorToTarget();
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
