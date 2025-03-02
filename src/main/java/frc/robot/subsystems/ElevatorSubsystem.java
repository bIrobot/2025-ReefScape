package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
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

    private final DigitalInput m_beamNotBroken = new DigitalInput(1);
    private final DigitalInput m_limitSwitch = new DigitalInput(2);

    private int ticks = 0;

    private boolean m_firstPos = true;
    private double m_lastPos = -1.0;  // illegal value
    private int m_revolutions = 0;

    public boolean elevatorCalibrationFailed = false;

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

        if (! elevatorRockBottom()) {
            LimelightHelpers.setLEDMode_ForceBlink("");
            elevatorCalibrationFailed = true;
        } else {
            LimelightHelpers.setLEDMode_ForceOn("");
        }

        // N.B. I don't believe we can use a PID controller because we manually add revolutions to our encoder?
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

    public boolean elevatorRockBottom()
    {
        return ! m_beamNotBroken.get();
    }

    public boolean elevatorRockTop()
    {
        return ! m_limitSwitch.get();
    }

    public void elevatorHold()
    {
        m_currentElevatorState = ElevatorState.STOP;
    }

    public void elevatorStop()
    {
        if (m_currentElevatorState == ElevatorState.UP || m_currentElevatorState == ElevatorState.DOWN) {
            elevatorHold();
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
        if (ticks++%50==0) System.out.println("ELEVATOR: Encoder: " + getFullPosition() +
                                               " rockBottom:" + elevatorRockBottom() +
                                               " rockTop:" + elevatorRockTop());

        if (DriverStation.isTest() && DriverStation.isEnabled()) {
            if (ticks++%50==0) System.out.println("ELEVATOR STOP/COAST");
            stopElevator();
            elevatorCoast();
            return;
        }

        // elevator safety
        if (getFullPosition() < ElevatorConstants.kElevatorLevelBottom && m_currentElevatorState == ElevatorState.DOWN) {
            m_ElevatorMotorLeft.set(0);
            m_ElevatorMotorRight.set(0);
        }
        if (getFullPosition() > ElevatorConstants.kElevatorLevelTop && m_currentElevatorState == ElevatorState.UP) {
            m_ElevatorMotorLeft.set(0);
            m_ElevatorMotorRight.set(0);
        }

        if (elevatorCalibrationFailed) {
            stopElevator();
            return;
        }

        setElevatorMotorToTarget();
    }

    private void stopElevator()
    {
        m_ElevatorMotorLeft.set(0);
        m_ElevatorMotorRight.set(0);
    }

    private void moveElevatorUp(boolean slow)
    {
        if (! elevatorRockTop() && getFullPosition() < ElevatorConstants.kElevatorLevelTop) {
            // up
            m_ElevatorMotorLeft.set(ElevatorConstants.kElevatorUpSpeed/(slow?5:1));
            m_ElevatorMotorRight.set(-ElevatorConstants.kElevatorUpSpeed/(slow?5:1));
        }
    }

    private void moveElevatorDown(boolean slow)
    {
        if (! elevatorRockBottom() && getFullPosition() > ElevatorConstants.kElevatorLevelBottom) {
            // down
            m_ElevatorMotorLeft.set(-ElevatorConstants.kElevatorDownSpeed/(slow?5:1));
            m_ElevatorMotorRight.set(ElevatorConstants.kElevatorDownSpeed/(slow?5:1));
        }
    }

    private double getElevatorMotorTarget(ElevatorState state)
    {
        switch (state) {
            case LEVEL1:
                return ElevatorConstants.kElevatorLevel1;
            case LEVEL2:
                return ElevatorConstants.kElevatorLevel2;
            case LEVEL3:
                return ElevatorConstants.kElevatorLevel3;
            case LEVEL4:
                return ElevatorConstants.kElevatorLevel4;
            default:
                assert(false);
                return ElevatorConstants.kElevatorLevelSafe;
        }
    }

    public boolean willElevatorGoUp(ElevatorState state)
    {
        return getElevatorMotorTarget(state) > getFullPosition();
    }

    private void setElevatorMotorToTarget() {
        if (elevatorCalibrationFailed) {
            stopElevator();
            return;
        }

        // if we are at bottom and still going down...
        if (elevatorRockBottom() && m_ElevatorMotorLeft.get() < 0) {
            stopElevator();
            System.out.println("ROCK BOTTOM!");
            return;
        }

        // if we are at top and still going up...
        if (elevatorRockTop() && m_ElevatorMotorLeft.get() > 0) {
            stopElevator();
            System.out.println("ROCK TOP!");
            return;
        }

        switch (m_currentElevatorState){
            case STOP:
                // hold position
                setMotorsLevel(getFullPosition());
                break;
            case UP:
                moveElevatorUp(false);
                break;
            case DOWN:
                moveElevatorDown(false);
                break;
            case LEVEL1:
            case LEVEL2:
            case LEVEL3:
            case LEVEL4:
                setMotorsLevel(getElevatorMotorTarget(m_currentElevatorState));
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

        sign = pos - getFullPosition();
        diff = Math.abs(sign);

        if (diff < 0.02) {
            // close enough
            m_ElevatorMotorLeft.set(0.0);
            m_ElevatorMotorRight.set(0.0);
        } else if (diff < 0.2) {
            if (sign < 0) {
                // down slow
                moveElevatorDown(true);
            } else {
                // up slow
                moveElevatorUp(true);
            }
        } else {
            if (sign < 0) {
                // down
                moveElevatorDown(false);
            } else if (! elevatorRockTop()) {
                // up
                moveElevatorUp(false);
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
