package frc.robot.subsystems.ClimberMain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;


public class ClimberSubsystemR extends SubsystemBase {
    private final CANSparkMax climberR;

    public ClimberSubsystemR() {
        climberR = new CANSparkMax(18, MotorType.kBrushless);

        climberR.restoreFactoryDefaults();
        climberR.setIdleMode(IdleMode.kBrake);
        
    }

    public Command uhOhNoWorky(double speed){
        return run(() -> {
            climberR.set(speed);
        });
    }

    public Command uhOhNoWorkyStop() {
        return run(() -> {
            climberR.set(0);
        });
    }
    // Waiting for them to make it, so I can do the math for Encoder distance
}
