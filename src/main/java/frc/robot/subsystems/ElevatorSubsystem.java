package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

    private enum ElevatorState {
        STOP,
        UP,
        DOWN
        // XXX -- real state targets, not motions
        //BOTTOM,
        //HANDOFF,
        //LEVEL1,
        //LEVEL2,
        //LEVEL3,
        //LEVEL4
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
    }

    public void elevatorStop()
    {
        m_currentElevatorState = ElevatorState.STOP;
    }

    public void elevatorUp()
    {
        m_currentElevatorState = ElevatorState.UP;
    }

    public void elevatorDown()
    {
        m_currentElevatorState = ElevatorState.DOWN;
    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("ELEVATOR: Encoder: " + m_ElevatorEncoder.getPosition());

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
            default:
                assert(false);
                break;
        }
    }
}
