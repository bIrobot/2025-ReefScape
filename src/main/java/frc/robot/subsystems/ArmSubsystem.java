package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_armMotorLeft;
    private final SparkMax m_armMotorRight;
    private SparkAbsoluteEncoder m_armEncoder;

    private ArmState m_currentArmState = ArmState.LIMP;
    private double m_ArmTime = 0;
    private HandState m_currentHandState = HandState.LIMP;  // ???
    private double m_PivotTime = 0;

    private int ticks = 0;

    private enum HandState {
        LIMP,
        STRAIGHT,
        DOWN,
        UP
    };

    private enum ArmState {
        LIMP,
        DOWN,
        HANDOFF,
        STOW
    };
  
    public ArmSubsystem()
    {
        m_armMotorLeft = new SparkMax(11, MotorType.kBrushless);  // XXX Constants
        m_armEncoder = m_armMotorLeft.getAbsoluteEncoder();

        m_armMotorRight = new SparkMax(12, MotorType.kBrushless);  // XXX Constants

        SparkMaxConfig configArmLeft = new SparkMaxConfig();
        //configArmLeft.idleMode(IdleMode.kBrake);
        m_armMotorLeft.configure(configArmLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configArmRight = new SparkMaxConfig();
        //configArmRight.idleMode(IdleMode.kBrake);
        m_armMotorRight.configure(configArmRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        if (ticks++%50==0) System.out.println("ARM: Encoder: " + m_armEncoder.getPosition());
    }
}
