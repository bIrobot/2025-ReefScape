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
    private final SparkMaxConfig m_configArmLeft = new SparkMaxConfig();
    private SparkAbsoluteEncoder m_ArmEncoder;
    private SparkClosedLoopController m_ArmController;

    private final SparkMax m_ArmMotorRight;
    private final SparkMaxConfig m_configArmRight = new SparkMaxConfig();

    private final SparkMax m_handMotor;
    private final SparkMaxConfig m_configHand = new SparkMaxConfig();
    private SparkAbsoluteEncoder m_handEncoder;
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
    private boolean m_coast = false;

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

        m_handMotor = new SparkMax(13, MotorType.kBrushless);  // XXX Constants
        m_handEncoder = m_handMotor.getAbsoluteEncoder();
        m_HandController = m_handMotor.getClosedLoopController();

        m_fingerMotor = new SparkMax(14, MotorType.kBrushless);  // XXX Constants

        // left motor has encoder and controller
        m_configArmLeft.inverted(true);
        m_configArmLeft.closedLoop.pid(k_armMotorP, k_armMotorI, k_armMotorD)
                                  .pid(k_armMotorP1, k_armMotorI1, k_armMotorD1, ClosedLoopSlot.kSlot1)
                                  .positionWrappingEnabled(false)
                                  .outputRange(-ArmConstants.kArmUpSpeed, ArmConstants.kArmDownSpeed)
                                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_configArmLeft.idleMode(IdleMode.kBrake);
        m_ArmMotorLeft.configure(m_configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // right motor follows left motor with inverted direction
        m_configArmRight.inverted(true);
        m_configArmRight.idleMode(IdleMode.kBrake)
                        .follow(11, true);  // XXX Constants
        m_ArmMotorRight.configure(m_configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // initial arm position
        armHold();

        // hand motor has encoder and controller
        m_configHand.inverted(true);
        m_configHand.closedLoop.pid(k_handMotorP, k_handMotorI, k_handMotorD)
                               .pid(k_handMotorP1, k_handMotorI1, k_handMotorD1, ClosedLoopSlot.kSlot1)
                               .positionWrappingEnabled(false)
                               .outputRange(-ArmConstants.kHandUpSpeed, ArmConstants.kHandDownSpeed)
                               .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_configHand.idleMode(IdleMode.kBrake);
        m_handMotor.configure(m_configHand, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // initial hand position
        handHold();
    }

    // set the arm motors to coast or brake mode
    public void armCoast(boolean coast)
    {
        m_configArmLeft.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        m_ArmMotorLeft.configure(m_configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_configArmRight.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        m_ArmMotorRight.configure(m_configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_configHand.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
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
            m_ArmController.setReference(-ArmConstants.kArmUpSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }
    }

    public void armDown()
    {
        if (m_currentArmState != ArmState.DOWN && getArmPosition() <= ArmConstants.kArmLevelBottom) {
            m_currentArmState = ArmState.DOWN;
            m_ArmController.setReference(ArmConstants.kArmDownSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
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
            m_HandController.setReference(ArmConstants.kHandUpSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }
    }

    public void handDown()
    {
        if (m_currentHandState != HandState.DOWN && getHandPosition() >= ArmConstants.kHandLevelBottom) {
            m_currentHandState = HandState.DOWN;
            m_HandController.setReference(-ArmConstants.kHandDownSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
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
        if (ticks++%50==0) System.out.println("ARM: Arm Encoder: " + m_ArmEncoder.getPosition() +
                                              " Hand Encoder: " + m_handEncoder.getPosition());

        if (DriverStation.isTest()) {
            if (DriverStation.isEnabled()) {
                if (ticks++%50==0) System.out.println("ARM STOP/COAST");
                if (! m_coast) {
                    m_ArmController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
                    m_HandController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
                    m_fingerMotor.set(0);
                    armCoast(true);
                    m_coast = true;
                }
            }
            return;
        } else if (m_coast) {
            armCoast(false);
            m_coast = false;
        }

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

    private double getArmPosition()
    {
        double armPos = m_ArmEncoder.getPosition();
        if (armPos == 0) {
            armPos = ArmConstants.kArmLevelSafe;
        }
        return armPos;
    }

    private double getHandPosition()
    {
        double handPos = m_handEncoder.getPosition();
        if (handPos == 0.0) {
            handPos = ArmConstants.kHandLevelSafe;
        }
        return handPos;
    }
}
