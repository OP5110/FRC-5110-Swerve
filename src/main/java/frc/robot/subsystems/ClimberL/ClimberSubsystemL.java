package frc.robot.subsystems.ClimberL;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;


public class ClimberSubsystemL extends SubsystemBase {
    private final CANSparkMax climberL;

    public ClimberSubsystemL() {
        climberL = new CANSparkMax(19, MotorType.kBrushless);

        climberL.restoreFactoryDefaults();
        climberL.setIdleMode(IdleMode.kBrake);
        
    }

    public Command uhOhNoWorky2(double speed){
        return run(() -> {
            climberL.set(speed);
        });
    }

    public Command uhOhNoWorky2Stop() {
        return run(() -> {
            climberL.set(0);
        });
    }
    // Waiting for them to make it, so I can do the math for Encoder distance
}
