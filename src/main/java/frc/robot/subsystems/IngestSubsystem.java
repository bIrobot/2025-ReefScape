package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IngestConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IngestSubsystem extends SubsystemBase{
    private final SparkMax m_ingestMotorLeft;
    private final SparkMax m_ingestMotorRight;

    private SparkMax m_PivotMotor;
    private SparkAbsoluteEncoder m_PivotEncoder;
    private SparkClosedLoopController m_PivotController;

    private IngestState m_currentIngestState = IngestState.STOP;
    private double m_IngestTime = 0;

    private static final double k_pivotMotorP = 2.0;
    private static final double k_pivotMotorI = 0.0;
    private static final double k_pivotMotorD = 0.2;

    private final DigitalInput m_beamNotBroken = new DigitalInput(0);

    private int ticks = 0;

    private enum IngestState {
        STOP,
        SAFE,
        FORWARD,
        REVERSE
    };

    public IngestSubsystem()
    {
        m_ingestMotorLeft = new SparkMax(17, MotorType.kBrushless);  // XXX Constants
        m_ingestMotorRight = new SparkMax(15, MotorType.kBrushless);  // XXX Constants

        m_PivotMotor = new SparkMax(16, MotorType.kBrushless);  // XXX Constants
        m_PivotEncoder = m_PivotMotor.getAbsoluteEncoder();
        m_PivotController = m_PivotMotor.getClosedLoopController();

        SparkMaxConfig configPivot = new SparkMaxConfig();
        configPivot.closedLoop.pid(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD)
                              .positionWrappingEnabled(true)
                              .positionWrappingInputRange(0, 1)
                              .outputRange(-0.2, 0.2)
                              .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        configPivot.idleMode(IdleMode.kBrake);
        m_PivotMotor.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // initial pivot position
        m_PivotController.setReference(m_PivotEncoder.getPosition(), ControlType.kPosition);
    }

    public void ingestCoast()
    {
        SparkMaxConfig configPivot = new SparkMaxConfig();
        configPivot.idleMode(IdleMode.kCoast);
        m_PivotMotor.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public boolean getIngestHasCoral()
    {
        return ! m_beamNotBroken.get();
    }

    public void startIngesting()
    {
        if (! getIngestHasCoral()) {
            m_currentIngestState = IngestState.FORWARD;
            m_PivotController.setReference(IngestConstants.k_pivotAngleGroundFraction, ControlType.kPosition);
        }
    }

    public void stopIngesting()
    {
        m_currentIngestState = IngestState.STOP;
        m_PivotController.setReference(IngestConstants.k_pivotAngleSafeFraction, ControlType.kPosition);
    }

    public void reverseIngesting()
    {
        m_IngestTime = System.nanoTime();
        m_currentIngestState = IngestState.REVERSE;
    }

    public void safeIngesting()
    {
        m_currentIngestState = IngestState.SAFE;
        m_PivotController.setReference(IngestConstants.k_pivotAngleSafeFraction, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("INGEST: Position: " + m_PivotEncoder.getPosition() +
                                              " hasCoral: " + getIngestHasCoral());

        if (DriverStation.isTest() && DriverStation.isEnabled()) {
            if (ticks++%50==0) System.out.println("INGEST STOP/COAST");
            m_PivotController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
            m_ingestMotorLeft.set(0.0);
            m_ingestMotorRight.set(0.0);
            ingestCoast();
            return;
        }

        setIngestMotorToTarget();
    }

    private void setIngestMotorToTarget() {
        switch (m_currentIngestState){
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
            case STOP:
                m_ingestMotorLeft.set(0.0);
                m_ingestMotorRight.set(0.0);
                break;
            default:
                assert(false);
                break;
        }
    }
}
