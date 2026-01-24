package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestSubsystem extends SubsystemBase {
    private final SparkMax m_motor;

    public TestSubsystem() {
        m_motor = new SparkMax(Constants.kTestMotorId, MotorType.kBrushless);
    }

    public void run(){
        m_motor.setVoltage(Constants.kTestMotorSpeed);
    }
    public void stop(){
        m_motor.setVoltage(0);
    }
}
