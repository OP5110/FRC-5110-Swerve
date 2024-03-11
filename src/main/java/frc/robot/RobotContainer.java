// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.ClimberL.ClimberSubsystemL;
import frc.robot.subsystems.ClimberMain.ClimberSubsystemR;
import frc.robot.subsystems.Conv.ConvSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;


public class RobotContainer
{
  private final ClimberSubsystemL m_climberL = new ClimberSubsystemL();
  private final ClimberSubsystemR m_climberR = new ClimberSubsystemR();
  private final ConvSubsystem m_conv = new ConvSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  XboxController driverXbox2 = new XboxController(1);
  XboxController driverXbox = new XboxController(0);


  private void configureBindings()
  {
    // Climber, DPad
    //Constants.operatorController.povUp().onTrue(m_climber.setHeight(ClimberSubsystem.ClimberState.EXTENDED.height));
    //Constants.operatorController.povDown().onTrue(m_climber.setHeight(ClimberSubsystem.ClimberState.RETRACTED.height));
    
    Constants.operatorController.rightBumper().whileTrue(m_climberR.uhOhNoWorky(-.75)).whileFalse(m_climberR.uhOhNoWorkyStop());
    Constants.operatorController.rightTrigger().whileTrue(m_climberR.uhOhNoWorky(.75)).whileFalse(m_climberR.uhOhNoWorkyStop());
    
    Constants.operatorController.leftBumper().whileTrue(m_climberL.uhOhNoWorky2(-.75)).whileFalse(m_climberL.uhOhNoWorky2Stop());
    Constants.operatorController.leftTrigger(.1).whileTrue(m_climberL.uhOhNoWorky2(.75)).whileFalse(m_climberL.uhOhNoWorky2Stop());


    // Conv, Bumpers and run when inake
    //Constants.operatorController.axisGreaterThan(5, 0.1).whileTrue(m_conv.runConvIntake()); // Looks wrong because of Intake stuff but we just running the Conv.
    //Constants.operatorController.axisLessThan(5, 0.1).whileTrue(m_conv.stopConv());
    Constants.operatorController.povUp().onTrue(m_conv.runConv(0.50))
      .onFalse(m_conv.runConv(0.03));
    Constants.operatorController.povDown().onTrue(m_conv.runConv(-.40))
      .onFalse(m_conv.runConv(0.03));
    


    /* Intake, left and right === Working on making these work
    following items are temp fix, unless I dont :p
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) > 0.1).whileTrue(m_intake.runLeftIntake(Constants.operatorController::getLeftY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) < 0.1).whileTrue(m_intake.runLeftIntake(Constants.operatorController::getLeftY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) == 0).whileTrue(m_intake.stopLeftIntake());
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) > 0.1).whileTrue(m_intake.runRightIntake(Constants.operatorController::getRightY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) < 0.1).whileTrue(m_intake.runRightIntake(Constants.operatorController::getRightY));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) == 0).whileTrue(m_intake.stopRightIntake());
     */


     
    Constants.operatorController.b().whileTrue(m_intake.autoRunIntake())
      .whileFalse(m_intake.stopIntake());
    Constants.operatorController.a().whileTrue(m_intake.autoRunIntakeRevers())
      .whileFalse(m_intake.stopIntake());
    
    // Shooter, LT & RT
    Constants.operatorController.x().whileTrue(m_shooter.runShooter(-1))
      .whileFalse(m_shooter.runShooter(0));


    // Default stuff remove eventually
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    //new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  
  public void configurePathPlanner() {
    // Conv
    NamedCommands.registerCommand("runConv", m_conv.autoRunConv());
    NamedCommands.registerCommand("stopConv", m_conv.stopConv());
    // Intake
    NamedCommands.registerCommand("runIntake", m_intake.autoRunIntake());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());
    // Shooter
    NamedCommands.registerCommand("runShooter", m_shooter.autoShooterRun());
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());

    drivebase.setupPathPlanner();
  }

   private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configurePathPlanner();
    PortForwarder.add(5800, "photonvision.local", 5800);

        // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    //double voltage = PDH.getVoltage();
    //SmartDashboard.putNumber("Voltage", voltage);

    configureBindings();
  }

  public void setDriveMode()
  {
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    drivebase.setDefaultCommand( !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle );
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public Command getAutonomousCommand()
  {
   // An example command will be run in autonomous
   
    
  // timer.reset();
  // timer.start();

    //if (timer.get() > 0 && timer.get() < 1 ) {
  //     m_shooter.runShooter(-1);
   //  }

//     if (timer.get() > 1 && timer.get() < 3 ) {
//       m_intake.autoRunIntake();
      
//     }
//     if (timer.get() > 3 && timer.get() < 4 ) {
//       m_shooter.stopShooter();
//       m_intake.stopIntake();
//     }


   return drivebase.getAutonomousCommand("POS3");

 }

}
