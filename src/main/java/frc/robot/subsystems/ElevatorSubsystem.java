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
    private final SparkMaxConfig m_configLeft = new SparkMaxConfig();
    private final SparkMax m_ElevatorMotorRight;
    private final SparkMaxConfig m_configRight = new SparkMaxConfig();
    private SparkAbsoluteEncoder m_ElevatorEncoder;

    private ElevatorState m_currentElevatorState = ElevatorState.STOP;
    private double m_currentElevatorGoto = ElevatorConstants.kElevatorLevelSafe;

    private final DigitalInput m_beamNotBroken = new DigitalInput(1);
    private final DigitalInput m_limitSwitch = new DigitalInput(2);

    private int ticks = 0;
    private boolean m_coast = false;

    private boolean m_firstPos = true;
    private double m_lastElevatorPos = -1.0;  // illegal value
    private int m_revolutions = 0;

    public boolean elevatorCalibrationFailed = false;

    private enum ElevatorState {
        STOP,
        UP,
        DOWN,
        GOTO  // state in m_currentElevatorGoto
    };

    public ElevatorSubsystem()
    {
        m_ElevatorMotorLeft = new SparkMax(10, MotorType.kBrushless);  // XXX Constants
        m_ElevatorEncoder = m_ElevatorMotorLeft.getAbsoluteEncoder();

        m_ElevatorMotorRight = new SparkMax(9, MotorType.kBrushless);  // XXX Constants

        m_configLeft.idleMode(IdleMode.kBrake);
        m_ElevatorMotorLeft.configure(m_configLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_configRight.idleMode(IdleMode.kBrake);
        m_ElevatorMotorRight.configure(m_configRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (! elevatorRockBottom()) {
            LimelightHelpers.setLEDMode_ForceBlink("");
            elevatorCalibrationFailed = true;
        } else {
            LimelightHelpers.setLEDMode_ForceOn("");
        }

        // N.B. I don't believe we can use a PID controller because we manually add revolutions to our encoder?
    }

    // set the elevator motors to coast or brake mode
    public void elevatorCoast(boolean coast)
    {
        m_configLeft.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        m_ElevatorMotorLeft.configure(m_configLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_configRight.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        m_ElevatorMotorRight.configure(m_configRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // return true if the elevator must not go lower!
    public boolean elevatorRockBottom()
    {
        return ! m_beamNotBroken.get();
    }

    // return true if the elvator must not go higher!
    public boolean elevatorRockTop()
    {
        return ! m_limitSwitch.get();
    }

    // hold the current elevator position
    public void elevatorHold()
    {
        m_currentElevatorGoto = getFullElevatorPosition();
        m_currentElevatorState = ElevatorState.GOTO;
    }

    public void elevatorStop()
    {
        if (m_currentElevatorState == ElevatorState.UP || m_currentElevatorState == ElevatorState.DOWN) {
            m_currentElevatorState = ElevatorState.STOP;
        }
    }

    public void elevatorUp()
    {
        if (m_currentElevatorState != ElevatorState.UP && getFullElevatorPosition() < ElevatorConstants.kElevatorLevelTop) {
            m_currentElevatorState = ElevatorState.UP;
        }
    }

    public void elevatorDown()
    {
        if (m_currentElevatorState != ElevatorState.DOWN && getFullElevatorPosition() > ElevatorConstants.kElevatorLevelBottom) {
            m_currentElevatorState = ElevatorState.DOWN;
        }
    }

    public void elevatorGoto(double elevatorPos)
    {
        if (! (elevatorPos >= ElevatorConstants.kElevatorLevelBottom && elevatorPos <= ElevatorConstants.kElevatorLevelTop)) {
            elevatorPos = ElevatorConstants.kElevatorLevelSafe;
            System.out.println("BAD POSITION IGNORED: alevatorGoto(): " + elevatorPos);
        }
        m_currentElevatorGoto = elevatorPos;
        m_currentElevatorState = ElevatorState.GOTO;
    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("ELEVATOR: Encoder: " + getFullElevatorPosition() +
                                               " rockBottom:" + elevatorRockBottom() +
                                               " rockTop:" + elevatorRockTop());

        if (DriverStation.isTest()) {
            if (DriverStation.isEnabled()) {
                if (ticks++%50==0) System.out.println("ELEVATOR STOP/COAST");
                if (! m_coast) {
                    stopElevatorMotors();
                    elevatorCoast(true);
                    m_coast = true;
                }
            }
            return;
        } else if (m_coast) {
            elevatorCoast(false);
            m_coast = false;
        }

        if (elevatorCalibrationFailed) {
            stopElevatorMotors();
            return;
        }

        // elevator safety
        if (getFullElevatorPosition() < ElevatorConstants.kElevatorLevelBottom && m_currentElevatorState == ElevatorState.DOWN) {
            stopElevatorMotors();
        }
        if (getFullElevatorPosition() > ElevatorConstants.kElevatorLevelTop && m_currentElevatorState == ElevatorState.UP) {
            stopElevatorMotors();
        }

        setElevatorMotorToTarget();
    }

    private void stopElevatorMotors()
    {
        m_ElevatorMotorLeft.set(0);
        m_ElevatorMotorRight.set(0);
    }

    private void moveElevatorMotorsUp(boolean slow)
    {
        if (! elevatorRockTop()) {
            // up
            m_ElevatorMotorLeft.set(ElevatorConstants.kElevatorUpSpeed/(slow?5:1));
            m_ElevatorMotorRight.set(-ElevatorConstants.kElevatorUpSpeed/(slow?5:1));
        }
    }

    private void moveElevatorMotorsDown(boolean slow)
    {
        if (! elevatorRockBottom()) {
            // down
            m_ElevatorMotorLeft.set(-ElevatorConstants.kElevatorDownSpeed/(slow?5:1));
            m_ElevatorMotorRight.set(ElevatorConstants.kElevatorDownSpeed/(slow?5:1));
        }
    }

    public boolean willElevatorGoUp(double elevatorPos)
    {
        return elevatorPos > getFullElevatorPosition();
    }

    private void setElevatorMotorToTarget() {
        if (elevatorCalibrationFailed) {
            stopElevatorMotors();
            return;
        }

        // if we are at bottom and still going down...
        if (elevatorRockBottom() && m_ElevatorMotorLeft.get() < 0) {
            stopElevatorMotors();
            System.out.println("ROCK BOTTOM!");
            return;
        }

        // if we are at top and still going up...
        if (elevatorRockTop() && m_ElevatorMotorLeft.get() > 0) {
            stopElevatorMotors();
            System.out.println("ROCK TOP!");
            return;
        }

        switch (m_currentElevatorState){
            case STOP:
                // hold position
                setElevatorMotorsLevel(getFullElevatorPosition());
                break;
            case UP:
                setElevatorMotorsLevel(ElevatorConstants.kElevatorLevelTop);
                break;
            case DOWN:
                setElevatorMotorsLevel(ElevatorConstants.kElevatorLevelBottom);
                break;
            case GOTO:
                setElevatorMotorsLevel(m_currentElevatorGoto);
                break;
            default:
                assert(false);
                break;
        }
    }

    // poor man's PID controller
    private void setElevatorMotorsLevel(double elevatorPos) {
        double sign;
        double diff;

        sign = elevatorPos - getFullElevatorPosition();
        diff = Math.abs(sign);

        if (diff <= 0.01) {
            // close enough
            m_ElevatorMotorLeft.set(0.0);
            m_ElevatorMotorRight.set(0.0);
        } else if (diff < 0.2) {
            if (sign < 0) {
                // down slow
                moveElevatorMotorsDown(true);
            } else {
                // up slow
                moveElevatorMotorsUp(true);
            }
        } else {
            if (sign < 0) {
                // down
                moveElevatorMotorsDown(false);
            } else if (! elevatorRockTop()) {
                // up
                moveElevatorMotorsUp(false);
            }
        }
    }

    private double getFullElevatorPosition()
    {
        double elevatorPos = m_ElevatorEncoder.getPosition();
        if (m_firstPos) {
            if (elevatorPos > 0.5) {
                m_revolutions = -1;
            }
            m_firstPos = false;
        }
        if (m_lastElevatorPos != -1.0) {
            if (elevatorPos < m_lastElevatorPos-0.5) {
                m_revolutions++;
            }
            if (elevatorPos > m_lastElevatorPos+0.5) {
                m_revolutions--;
            }
        }
        m_lastElevatorPos = elevatorPos;
        return elevatorPos + m_revolutions;
    }
}
