package frc.robot.subsystems.Conv;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class ConvSubsystem extends SubsystemBase {
    private final CANSparkMax convMotor;
    private RelativeEncoder conv_encoder;
    
    public ConvSubsystem() {
        convMotor = new CANSparkMax(9, MotorType.kBrushless);
        convMotor.setSmartCurrentLimit(10);
        convMotor.restoreFactoryDefaults();
        convMotor.setIdleMode(IdleMode.kBrake);
        conv_encoder = convMotor.getEncoder();
    }
    
    public Command runConv(double speed){
        return startEnd(
            () -> convMotor.set(speed),
            () -> convMotor.stopMotor()
        );
    }
    public Command runConvIntake(){
        return runConv(0.5);
    }

    // Auto
    public Command autoRunConv(){
        return runConv(-0.3);
    }
    public Command stopConv(){
        return runOnce(
            () -> convMotor.stopMotor()
        );
    }
}
