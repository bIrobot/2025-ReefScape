package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_ElevatorMotorLeft;
    private final SparkMax m_ElevatorMotorRight;
    private SparkAbsoluteEncoder m_ElevatorEncoder;

    private ElevatorState m_currentElevatorState = ElevatorState.STOP;
    private double m_ElevatorTime = 0;

    private int ticks = 0;

    private boolean m_firstPos = true;
    private double m_lastPos = -1.0;  // illegal value
    private int m_revolutions = 0;

    public enum ElevatorState {  // XXX -- should be private!
        STOP,
        UP,
        DOWN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4
    };

    public ElevatorSubsystem()
    {
        m_ElevatorMotorLeft = new SparkMax(10, MotorType.kBrushless);  // XXX Constants
        m_ElevatorEncoder = m_ElevatorMotorLeft.getAbsoluteEncoder();

        m_ElevatorMotorRight = new SparkMax(9, MotorType.kBrushless);  // XXX Constants

        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kBrake);
        m_ElevatorMotorLeft.configure(configLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kBrake);
        m_ElevatorMotorRight.configure(configRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // N.B. I don't believe we can use a PID controller because we drive Left/Right motors based on a single encoder
    }

    public void elevatorCoast()
    {
        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kCoast);
        m_ElevatorMotorLeft.configure(configLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kCoast);
        m_ElevatorMotorRight.configure(configRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void elevatorStop()
    {
        if (m_currentElevatorState == ElevatorState.UP || m_currentElevatorState == ElevatorState.DOWN) {
            m_currentElevatorState = ElevatorState.STOP;
        }
    }

    public void elevatorUp()
    {
        m_currentElevatorState = ElevatorState.UP;
    }

    public void elevatorDown()
    {
        m_currentElevatorState = ElevatorState.DOWN;
    }

    public void elevatorGoto(ElevatorState level)  // XXX -- horrible api to have illegal enum values!
    {
        m_currentElevatorState = level;
    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("ELEVATOR: Encoder: " + getFullPosition());

        setElevatorMotorToTarget();
    }

    private void setElevatorMotorToTarget() {
        switch (m_currentElevatorState){
            case STOP:
                m_ElevatorMotorLeft.set(0.0);
                m_ElevatorMotorRight.set(0.0);
                break;
            case UP:
                m_ElevatorMotorLeft.set(ElevatorConstants.kElevatorUpSpeed);
                m_ElevatorMotorRight.set(-ElevatorConstants.kElevatorUpSpeed);
                break;
            case DOWN:
                m_ElevatorMotorLeft.set(-ElevatorConstants.kElevatorDownSpeed);
                m_ElevatorMotorRight.set(ElevatorConstants.kElevatorDownSpeed);
                break;
            case LEVEL1:
                setMotorsLevel(ElevatorConstants.kElevatorLevel1);
                break;
            case LEVEL2:
                setMotorsLevel(ElevatorConstants.kElevatorLevel2);
                break;
            case LEVEL3:
                setMotorsLevel(ElevatorConstants.kElevatorLevel3);
                break;
            case LEVEL4:
                setMotorsLevel(ElevatorConstants.kElevatorLevel4);
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

        sign = pos-getFullPosition();
        diff = Math.abs(sign);

        if (diff < 0.02) {
            m_ElevatorMotorLeft.set(0.0);
            m_ElevatorMotorRight.set(0.0);
        } else if (diff < 0.2) {
            if (sign < 0) {
                // down slow
                m_ElevatorMotorLeft.set(-ElevatorConstants.kElevatorDownSpeed/5);
                m_ElevatorMotorRight.set(ElevatorConstants.kElevatorDownSpeed/5);
            } else {
                // up slow
                m_ElevatorMotorLeft.set(ElevatorConstants.kElevatorUpSpeed/5);
                m_ElevatorMotorRight.set(-ElevatorConstants.kElevatorUpSpeed/5);
            }
        } else {
            if (sign < 0) {
                // down
                m_ElevatorMotorLeft.set(-ElevatorConstants.kElevatorDownSpeed);
                m_ElevatorMotorRight.set(ElevatorConstants.kElevatorDownSpeed);
            } else {
                // up
                m_ElevatorMotorLeft.set(ElevatorConstants.kElevatorUpSpeed);
                m_ElevatorMotorRight.set(-ElevatorConstants.kElevatorUpSpeed);
            }
        }
    }

    private double getFullPosition()
    {
        double pos = m_ElevatorEncoder.getPosition();
        if (m_firstPos) {
            if (pos > 0.5) {
                m_revolutions = -1;
            }
            m_firstPos = false;
        }
        if (m_lastPos != -1.0) {
            if (pos < m_lastPos-0.5) {
                m_revolutions++;
            }
            if (pos > m_lastPos+0.5) {
                m_revolutions--;
            }
        }
        m_lastPos = pos;
        return pos+m_revolutions;
    }
}
