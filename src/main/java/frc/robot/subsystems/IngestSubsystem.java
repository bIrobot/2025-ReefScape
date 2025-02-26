package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IngestConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IngestSubsystem extends SubsystemBase{
    private final SparkMax m_ingestMotorLeft;
    private final SparkMax m_ingestMotorRight;
    private SparkMax m_PivotMotor;
    private SparkAbsoluteEncoder m_PivotEncoder;

    private IngestState m_currentIngestState = IngestState.STOP;
    private double m_IngestTime = 0;
    private PivotState m_currentPivotState = PivotState.GROUND;  // STOW
    private double m_PivotTime = 0;

    private static final double k_pivotMotorP = 0.12;
    private static final double k_pivotMotorI = 0.0;
    private static final double k_pivotMotorD = 0.001;

    private int ticks = 0;

    private enum IngestState {
        STOP,
        FORWARD,
        REVERSE
    };

    private enum PivotState {
        GROUND,
        HANDOFF,
        STOW
    };
  
    public IngestSubsystem()
    {
        m_ingestMotorLeft = new SparkMax(17, MotorType.kBrushless);  // XXX Constants

        m_ingestMotorRight = new SparkMax(15, MotorType.kBrushless);  // XXX Constants

        m_PivotMotor = new SparkMax(16, MotorType.kBrushless);  // XXX Constants
        m_PivotEncoder = m_PivotMotor.getAbsoluteEncoder();

        SparkMaxConfig configPivot = new SparkMaxConfig();
        configPivot.closedLoop.p(k_pivotMotorP);
        configPivot.closedLoop.i(k_pivotMotorI);
        configPivot.closedLoop.d(k_pivotMotorD);
        configPivot.closedLoop.outputRange(-1.0, 1.0);
        configPivot.idleMode(IdleMode.kBrake);
        configPivot.encoder.positionConversionFactor(ArmConstants.kClimberGearRatio);
        configPivot.encoder.velocityConversionFactor(ArmConstants.kClimberGearRatio);

        m_PivotMotor.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getPivotEncoderFraction()
    {
        double value = m_PivotEncoder.getPosition();
        if (value > IngestConstants.k_pivotAngleWrapFraction) {
            return IngestConstants.k_pivotAngleGroundFraction;
        }
        if (value > IngestConstants.k_pivotAngleStowFraction) {
            return IngestConstants.k_pivotAngleStowFraction;
        }
        return value;
    }

    private double getPivotTargetFraction()
    {
        switch (m_currentPivotState) {
          case GROUND:
            return IngestConstants.k_pivotAngleGroundFraction;
          case HANDOFF:
            return IngestConstants.k_pivotAngleHandoffFraction;
          case STOW:
            return IngestConstants.k_pivotAngleStowFraction;
          default:
            assert(false);
            return 0.0;
        }
    }

    public boolean getIngestHasCoral()
    {
        // TODO
        return false;
    }

    public void startIngesting()
    {
        m_currentIngestState = IngestState.FORWARD;
        m_currentPivotState = PivotState.GROUND;
    }

    public void stopIngesting()
    {
        m_currentIngestState = IngestState.STOP;
        m_currentPivotState = PivotState.HANDOFF;
    }

    public void reverseIngesting()
    {
        m_currentIngestState = IngestState.REVERSE;
        m_IngestTime = System.nanoTime();
    }

    public boolean isPivotAtTarget() {
        return Math.abs(getPivotEncoderFraction() - getPivotTargetFraction()) < IngestConstants.k_pivotFractionResolution;
    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("INGEST: Position: " + getPivotEncoderFraction() +
                                              " Target: " + getPivotTargetFraction() +
                                              " atTarget: " + isPivotAtTarget());

        // XXX if we find coral, stop ingesting
        //stopIngesting();

        setIngestMotorToTarget();
        setPivotAngleToTarget();
    }

    private void setIngestMotorToTarget() {
        switch (m_currentIngestState){
            case FORWARD:
                m_ingestMotorLeft.set(IngestConstants.k_intakeSpeed);
                m_ingestMotorRight.set(-IngestConstants.k_intakeSpeed-0.05);
                break;
            case REVERSE:
                if (System.nanoTime() - m_IngestTime > IngestConstants.k_reverseNsec) {
                    m_currentIngestState = IngestState.STOP;
                } else {
                    m_ingestMotorLeft.set(IngestConstants.k_ejectSpeed);
                    m_ingestMotorRight.set(-IngestConstants.k_ejectSpeed);
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

    private void setPivotAngleToTarget() {
        if (isPivotAtTarget()) {
            m_PivotMotor.set(0.0);
            return;
        }

        if (getPivotEncoderFraction() < getPivotTargetFraction()) {
            m_PivotMotor.set(IngestConstants.k_pivotSpeed);
        } else {
            m_PivotMotor.set(-IngestConstants.k_pivotSpeed);
        }
    }
}
