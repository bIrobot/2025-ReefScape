package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IngestConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IngestSubsystem extends SubsystemBase{
    private final SparkMax m_ingestMotorLeft;
    private final SparkMax m_ingestMotorRight;

    private SparkMax m_PivotMotor;
    private SparkAbsoluteEncoder m_PivotEncoder;
    private SparkClosedLoopController m_PivotController;
    private SparkMaxConfig m_configPivot = new SparkMaxConfig();
    private IngestState m_currentIngestState = IngestState.STOP;
    private double m_IngestTime = 0;
    private double m_PulseTime = 0;

    private static final double k_pivotMotorP = 2.0;
    private static final double k_pivotMotorI = 0.0;
    private static final double k_pivotMotorD = 0.2;

    private final DigitalInput m_beamNotBroken = new DigitalInput(0);

    private int m_ticks = 0;

    private enum IngestState {
        STOP,
        SAFE,
        HANDOFF,
        FORWARD,
        REVERSE,
        PULSE1,  // forward
        PULSE2,  // pause
        PULSE3  // reverse
    };

    public IngestSubsystem()
    {
        m_ingestMotorLeft = new SparkMax(17, MotorType.kBrushless);  // XXX Constants
        m_ingestMotorRight = new SparkMax(15, MotorType.kBrushless);  // XXX Constants

        m_PivotMotor = new SparkMax(16, MotorType.kBrushless);  // XXX Constants
        m_PivotEncoder = m_PivotMotor.getAbsoluteEncoder();
        m_PivotController = m_PivotMotor.getClosedLoopController();

        m_configPivot.closedLoop.pid(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD)
                              .positionWrappingEnabled(false)
                              .outputRange(-0.2, 0.2)
                              .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_PivotMotor.configure(m_configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // return true if the ingest has captured a coral game piece
    public boolean getIngestHasCoral()
    {
        return ! m_beamNotBroken.get();
    }

    public void ingestHold()
    {
        m_PivotController.setReference(getPivotPosition(), ControlType.kPosition);
    }

    public void startIngesting()
    {
        if (! getIngestHasCoral()) {
            m_currentIngestState = IngestState.FORWARD;
            m_PivotController.setReference(IngestConstants.k_pivotAngleGroundFraction, ControlType.kPosition);
        } else {
            m_PulseTime = System.nanoTime();
            m_currentIngestState = IngestState.PULSE1;
        }
    }

    public void stopIngesting()
    {
        m_currentIngestState = IngestState.STOP;
        if (getIngestHasCoral()) {
            m_PivotController.setReference(IngestConstants.k_pivotAngleSafeFraction, ControlType.kPosition);
        } else {
            m_PivotController.setReference(IngestConstants.k_pivotAngleVerticalFraction, ControlType.kPosition);
        }
    }

    public void reverseIngesting()
    {
        m_IngestTime = System.nanoTime();
        m_currentIngestState = IngestState.REVERSE;
    }

    public void handoffIngesting()
    {
        m_currentIngestState = IngestState.HANDOFF;
        m_PivotController.setReference(IngestConstants.k_pivotAngleVerticalFraction, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        if (m_ticks++%50==0) System.out.println("INGEST: Position: " + m_PivotEncoder.getPosition() +
                                              " hasCoral: " + getIngestHasCoral());

        // seek motor targets
        setIngestMotorToTarget();
    }

    private void setIngestMotorToTarget() {
        switch (m_currentIngestState) {
            case FORWARD:
                if (getIngestHasCoral()) {
                    stopIngesting();
                } else {
                    m_ingestMotorLeft.set(IngestConstants.k_intakeSpeed);
                    m_ingestMotorRight.set(-IngestConstants.k_intakeSpeed-0.05);
                }
                break;
            case REVERSE:
                if (System.nanoTime() - m_IngestTime > IngestConstants.k_reverseNsec) {
                    m_currentIngestState = IngestState.STOP;
                } else {
                    m_ingestMotorLeft.set(-IngestConstants.k_ejectSpeed);
                    m_ingestMotorRight.set(IngestConstants.k_ejectSpeed);
                }
                break;
            case PULSE1:
                if (System.nanoTime() - m_PulseTime > IngestConstants.k_pulse1Nsec) {
                    m_PulseTime = System.nanoTime();
                    m_currentIngestState = IngestState.PULSE2;
                } else {
                    m_ingestMotorLeft.set(IngestConstants.k_intakeSpeed);
                    m_ingestMotorRight.set(-IngestConstants.k_intakeSpeed);
                }
                break;
            case PULSE2:
                if (System.nanoTime() - m_PulseTime > IngestConstants.k_pulse2Nsec) {
                    m_PulseTime = System.nanoTime();
                    m_currentIngestState = IngestState.PULSE3;
                } else {
                    m_ingestMotorLeft.set(0);
                    m_ingestMotorRight.set(0);
                }
                break;
            case PULSE3:
                if (System.nanoTime() - m_PulseTime > IngestConstants.k_pulse3Nsec) {
                    m_currentIngestState = IngestState.STOP;
                } else {
                    m_ingestMotorLeft.set(-IngestConstants.k_ejectSpeed);
                    m_ingestMotorRight.set(IngestConstants.k_ejectSpeed);
                }
                break;
            case STOP:
                m_ingestMotorLeft.set(0.0);
                m_ingestMotorRight.set(0.0);
                break;
            default:
                assert(false);
                break;
        }
    }

    private double getPivotPosition()
    {
        double pos = m_PivotEncoder.getPosition();
        if (Math.round(pos*100.0)/100.0 == 0.0) {
            // 0 is illegal value; make it safe
            pos = IngestConstants.k_pivotAngleSafeFraction;
        }
        return pos;
    }
}
