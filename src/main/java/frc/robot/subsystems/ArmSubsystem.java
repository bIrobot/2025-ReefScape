package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TestPosition.TestState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// XXX - rename hand to wrist

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_ArmMotorLeft;
    private final SparkMaxConfig m_configArmLeft = new SparkMaxConfig();
    private SparkAbsoluteEncoder m_ArmEncoder;  // calibrate 0 with green block removed and arm up all the way
    private SparkClosedLoopController m_ArmController;

    private final SparkMax m_ArmMotorRight;
    private final SparkMaxConfig m_configArmRight = new SparkMaxConfig();

    private final SparkMax m_swivelMotor;
    private final SparkMaxConfig m_configSwivel = new SparkMaxConfig();
    private SparkAbsoluteEncoder m_swivelEncoder;  // calibrate 0 with swivel horizontal
    private SparkClosedLoopController m_swivelController;

    private final SparkMax m_handMotor;
    private final SparkMaxConfig m_configHand = new SparkMaxConfig();
    private SparkAbsoluteEncoder m_handEncoder;  // calibrate 0 with black block removed and hand down all the way
    private SparkClosedLoopController m_HandController;

    private final SparkMax m_fingerMotor;

    private ArmState m_currentArmState = ArmState.STOP;
    private HandState m_currentHandState = HandState.STOP;
    private FingerState m_currentFingerState = FingerState.STOP;
    private double m_FingerTime = 0;

    // slot 0 for position control
    private static final double k_armMotorP = 5.0;
    private static final double k_armMotorI = 0.0;
    private static final double k_armMotorD = 10.0;

    // slot 0 for position control
    private static final double k_swivelMotorP = 4.0;
    private static final double k_swivelMotorI = 0.0;
    private static final double k_swivelMotorD = 2.0;

    // slot 0 for position control
    private static final double k_handMotorP = 4.0;
    private static final double k_handMotorI = 0.0;
    private static final double k_handMotorD = 2.0;

    private int ticks = 0;

    private enum ArmState {
        STOP,
        UP,
        DOWN,
        GOTO  // state in controller
    };
  
    private enum HandState {
        STOP,
        UP,
        DOWN,
        GOTO  // state in controller
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

        m_swivelMotor = new SparkMax(16, MotorType.kBrushless);  // XXX Constants
        m_swivelEncoder = m_swivelMotor.getAbsoluteEncoder();
        m_swivelController = m_swivelMotor.getClosedLoopController();

        m_handMotor = new SparkMax(13, MotorType.kBrushless);  // XXX Constants
        m_handEncoder = m_handMotor.getAbsoluteEncoder();
        m_HandController = m_handMotor.getClosedLoopController();

        m_fingerMotor = new SparkMax(14, MotorType.kBrushless);  // XXX Constants

        // left motor has encoder and controller
        m_configArmLeft.inverted(true);
        m_configArmLeft.closedLoop.pid(k_armMotorP, k_armMotorI, k_armMotorD)
                                  .positionWrappingEnabled(false)
                                  .outputRange(-ArmConstants.kArmUpSpeed, ArmConstants.kArmDownSpeed)
                                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_ArmMotorLeft.configure(m_configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // right motor follows left motor with inverted direction
        m_configArmRight.inverted(true)
                        .follow(11, true);  // XXX Constants
        m_ArmMotorRight.configure(m_configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // swivel motor has encoder and controller
        //m_configSwivel.inverted(true);
        m_configSwivel.closedLoop.pid(k_swivelMotorP, k_swivelMotorI, k_swivelMotorD)
                               .positionWrappingEnabled(false)
                               .outputRange(-ArmConstants.kSwivelSpeed, ArmConstants.kSwivelSpeed)
                               .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_swivelMotor.configure(m_configSwivel, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // hand motor has encoder and controller
        m_configHand.inverted(true);
        m_configHand.closedLoop.pid(k_handMotorP, k_handMotorI, k_handMotorD)
                               .positionWrappingEnabled(false)
                               .outputRange(-ArmConstants.kHandUpSpeed, ArmConstants.kHandDownSpeed)
                               .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_handMotor.configure(m_configHand, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // hold the current arm position
    public void armHold()
    {
        m_ArmController.setReference(getArmPosition(), ControlType.kPosition);
    }

    public void armStop()
    {
        if (m_currentArmState == ArmState.UP || m_currentArmState == ArmState.DOWN) {
            m_currentArmState = ArmState.STOP;
            armHold();
        }
    }

    public void armUp()
    {
        if (m_currentArmState != ArmState.UP && getArmPosition() >= ArmConstants.kArmLevelTop) {
            m_currentArmState = ArmState.UP;
            m_ArmController.setReference(ArmConstants.kArmLevelTop, ControlType.kPosition);
        }
    }

    public void armDown()
    {
        if (m_currentArmState != ArmState.DOWN && getArmPosition() <= ArmConstants.kArmLevelBottom) {
            m_currentArmState = ArmState.DOWN;
            m_ArmController.setReference(ArmConstants.kArmLevelBottom, ControlType.kPosition);
        }
    }

    public void armGoto(double armPos)
    {
        if (! (armPos >= ArmConstants.kArmLevelTop && armPos <= ArmConstants.kArmLevelBottom)) {
            armPos = ArmConstants.kArmLevelSafe;
            System.out.println("BAD POSITION IGNORED: armGoto(): " + armPos);
        }
        m_currentArmState = ArmState.GOTO;
        m_ArmController.setReference(armPos, ControlType.kPosition);
    }

    public void swivelZero()
    {
        m_swivelController.setReference(0, ControlType.kPosition);
    }

    public void swivelPlus()
    {
        m_swivelController.setReference(0.25, ControlType.kPosition);
    }

    public void swivelMinus()
    {
        m_swivelController.setReference(-0.25, ControlType.kPosition);
    }

    public void handHold()
    {
        m_HandController.setReference(getHandPosition(), ControlType.kPosition);
    }

    public void handStop()
    {
        if (m_currentHandState == HandState.UP || m_currentHandState == HandState.DOWN) {
            m_currentHandState = HandState.STOP;
            handHold();
        }
    }

    public void handUp()
    {
        if (m_currentHandState != HandState.UP && getHandPosition() < ArmConstants.kHandLevelTop) {
            m_currentHandState = HandState.UP;
            m_HandController.setReference(ArmConstants.kHandLevelTop, ControlType.kPosition);
        }
    }

    public void handDown()
    {
        if (m_currentHandState != HandState.DOWN && getHandPosition() >= ArmConstants.kHandLevelBottom) {
            m_currentHandState = HandState.DOWN;
            m_HandController.setReference(ArmConstants.kHandLevelBottom, ControlType.kPosition);
        }
    }

    public void handGoto(double handPos)
    {
        if (! (handPos >= ArmConstants.kHandLevelBottom && handPos <= ArmConstants.kHandLevelTop)) {
            handPos = ArmConstants.kHandLevelSafe;
            System.out.println("BAD POSITION IGNORED: handGoto(): " + handPos);
        }
        m_currentHandState = HandState.GOTO;
        m_HandController.setReference(handPos, ControlType.kPosition);
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
        if (ticks++%100==0) System.out.println("ARM: Arm Encoder: " + m_ArmEncoder.getPosition() +
                                              " Hand Encoder: " + m_handEncoder.getPosition());

        // arm limit safety
        if (getArmPosition() > ArmConstants.kArmLevelBottom && m_currentArmState == ArmState.DOWN) {
            armHold();
        }
        if (getArmPosition() < ArmConstants.kArmLevelTop && m_currentArmState == ArmState.UP) {
            armHold();
        }

        // hand limit safety
        if (getHandPosition() < ArmConstants.kHandLevelBottom && m_currentHandState == HandState.DOWN) {
            handHold();
        }
        if (getHandPosition() > ArmConstants.kHandLevelTop && m_currentHandState == HandState.UP) {
            handHold();
        }

        // seek motor targets
        setFingerMotorToTarget();
    }

    // return the direction of travel to the target
    public TestState testArmPosition(double armPos)
    {
        double diff = armPos - getArmPosition();
        if (Math.abs(diff) <= ArmConstants.kArmTestClose) {
            return TestState.TARGET_ACHIEVED;
        } else if (diff > 0) {
            return TestState.GOING_DOWN;
        }
        return TestState.GOING_UP;
    }

    // return the direction of travel to the target
    public TestState testHandPosition(double handPos)
    {
        double diff = handPos - getHandPosition();
        if (Math.abs(diff) <= ArmConstants.kHandTestClose) {
            return TestState.TARGET_ACHIEVED;
        } else if (diff > 0) {
            return TestState.GOING_UP;
        }
        return TestState.GOING_DOWN;
    }

    private void setFingerMotorToTarget() {
        switch (m_currentFingerState) {
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

    private double getArmPosition()
    {
        double armPos = m_ArmEncoder.getPosition();
        if (Math.round(armPos*100.0)/100.0 == 0.0) {
            // 0 is illegal value; make it safe
            armPos = ArmConstants.kArmLevelSafe;
        }
        return armPos;
    }

    private double getHandPosition()
    {
        double handPos = m_handEncoder.getPosition();
        if (Math.round(handPos*100.0)/100.0 == 0.0) {
            // 0 is illegal value; make it safe
            handPos = ArmConstants.kHandLevelSafe;
        }
        return handPos;
    }
}
