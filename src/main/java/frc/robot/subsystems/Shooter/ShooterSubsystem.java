package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor;
    private final CANSparkMax shooterMotor2;
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(16, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(17, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kBrake);
    }

    public Command runShooter(double speed){
        return run(() -> {
            shooterMotor.set(speed);
            shooterMotor2.set(speed);
        });
    }

    // Auto
    public Command stopShooter(){
        return runOnce(() -> {
            shooterMotor.set(0);
            shooterMotor2.set(0);
        });
    }
    public Command autoShooterRun(){
        return startEnd(
            () -> {
                shooterMotor.set(1);
                shooterMotor2.set(1);
            },
            () -> {
                shooterMotor.stopMotor();
                shooterMotor2.stopMotor();
            }
        );
    }
}
