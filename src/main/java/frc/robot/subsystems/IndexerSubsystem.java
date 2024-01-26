package frc.robot.subsystems;
 
import com.revrobotics.CANSparkMax;
 import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
 import com.revrobotics.SparkRelativeEncoder;
 import com.revrobotics.CANSparkBase.ControlType;
 import com.revrobotics.CANSparkBase.IdleMode;
 import com.revrobotics.CANSparkLowLevel.MotorType;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase{
    CANSparkMax holder1;
    CANSparkMax holder2;
    CANSparkMax feeder1;
    CANSparkMax feeder2;
    boolean isHolding;
    public IndexerSubsystem(){
        CANSparkMax holder1 = new CANSparkMax(IndexerConstants.HOLDER_1_MOTOR, MotorType.kBrushless);
        CANSparkMax holder2 = new CANSparkMax(IndexerConstants.HOLDER_2_MOTOR, MotorType.kBrushless);
        CANSparkMax feeder1 = new CANSparkMax(IndexerConstants.FEEDER_1_MOTOR, MotorType.kBrushless);
        CANSparkMax feeder2 = new CANSparkMax(IndexerConstants.FEEDER_2_MOTOR, MotorType.kBrushless);
        //sensor sensorThingy = new ???;
        boolean isHolding = false;
        feeder2.follow(feeder1);
        feeder2.setInverted(true);
        holder2.follow(holder1);
        holder2.setInverted(true);
        holder1.setIdleMode(IdleMode.kBrake);
        holder2.setIdleMode(IdleMode.kBrake);
        feeder1.setIdleMode(IdleMode.kBrake);
        feeder2.setIdleMode(IdleMode.kBrake);
    }

    public double getInchesFromSensor() {
        return 0;
    }
    public void feederFeed(double generalSpeed){
        feeder1.set(generalSpeed);
    }
    public void feederOff(){
        feeder1.set(0);
    }
    public void holderRetrieve(double generalSpeed){
        holder1.set(generalSpeed);
    }
    public void holderOff(){
        holder1.set(0);
    }
    public void allOff(){
        feeder1.set(0);
        holder1.set(0);
    }
}
