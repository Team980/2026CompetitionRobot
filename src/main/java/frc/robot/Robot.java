// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIOPigeon2;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    private Command autonomousCommand;
    public static Alliance alliance = DriverStation.Alliance.Red;

    public enum AimStates
    {
        HUBAIM,
        LEFTCORNER,
        RIGHTCORNER;
    }

    public AimStates aimStates = AimStates.HUBAIM;
    public static boolean hasAimedOnce = false;
    
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
         RobotContainer.hasSeededPose = false;
        System.out.println("Constants.currentMode: " + Constants.currentMode);  
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                RobotController.setBrownoutVoltage(6.1);
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
         // Start AdvantageKit logger
        Logger.start();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
    }

    @Override
    public void disabledPeriodic()
    {
        //   if(!DriverStation.isEnabled() && !RobotContainer.hasEnabled)
        // {
            // System.out.println("Robot Enabled");
            // RobotContainer.hasEnabled = true;
            
            RobotContainer.hasSeededPose = false;
       // }
    
       
    }

    
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // start at high priority for robot code to minimize latency
        Threads.setCurrentThreadPriority(true, 99);
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
      // System.out.println( m_robotContainer.intake.pivotMotor.getPosition().getValueAsDouble());
      // System.out.println(m_robotContainer.intake.pivotMotor.rotation);
     
        // Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        
        CommandScheduler.getInstance().run();
        // if (allianceOptional.isPresent()) {
        //     alliance = allianceOptional.get();
        // } 
        // return to default priority for non-robot code
        Threads.setCurrentThreadPriority(false, 10);
        Logger.recordOutput("Zeroed exterior poses", new Pose3d [] {new Pose3d()});
        Logger.recordOutput("ZeroedInteriorPoses", new Pose3d[] {new Pose3d()});
        Logger.recordOutput(
        "FinalCompoenentPoses",
        new Pose3d[] {
          new Pose3d(
              0, 0, Math.sin(Timer.getTimestamp()) * 0.37 - 0.07, new Rotation3d(0.0, 0.0, 0.0))
        });
        if(alliance != null)
        {
            m_robotContainer.driver.rightBumper().whileTrue(m_robotContainer.subsystemCommands.testAim());
        }
       
      //  SmartDashboard.putNumber("Distance to Hub (inches)", PrepareShotCommand.distanceHub);
        //System.out.println(RobotContainer.isInCenter);
        if(Alliance.Blue == alliance && m_robotContainer.swerve.getState().Pose.getX() - Landmark.HUB.get().getX() > 0)
        {
            RobotContainer.isInCenter = true;
        }
        else if(Alliance.Red == alliance && m_robotContainer.swerve.getState().Pose.getX() - Landmark.HUB.get().getX() < 0)
        {
            RobotContainer.isInCenter = true;
        }
        else if(Alliance.Blue == alliance && m_robotContainer.swerve.getState().Pose.getX() - Landmark.HUB.get().getX() < 0)
        {
            RobotContainer.isInCenter = false;
        }
        else if(Alliance.Red == alliance && m_robotContainer.swerve.getState().Pose.getX() - Landmark.HUB.get().getX() > 0)
        {
            RobotContainer.isInCenter = false;
        }

          if(RobotContainer.isInCenter)
        {
            m_robotContainer.driver.y().whileTrue(m_robotContainer.subsystemCommands.testAimRightCorner());
            m_robotContainer.driver.a().whileTrue(m_robotContainer.subsystemCommands.testAimLeftCorner());
            hasAimedOnce = false;
        }
        // else if((aimStates == AimStates.HUBAIM || !RobotContainer.isInCenter) && !hasAimedOnce)
        // {
        //     m_robotContainer.driver.rightBumper().whileTrue(m_robotContainer.subsystemCommands.testAim());
        //     hasAimedOnce = true;
        // }
        

    //   //  System.out.println(m_robotContainer.driver.y().getAsBoolean());
        if(m_robotContainer.driver.y().getAsBoolean() && aimStates != AimStates.RIGHTCORNER)
        {
            aimStates = AimStates.RIGHTCORNER;
            m_robotContainer.operator.rightTrigger().whileTrue(m_robotContainer.subsystemCommands.preShootRightCommand());
        }
        else if(m_robotContainer.driver.a().getAsBoolean() && aimStates != AimStates.LEFTCORNER)
        {
              aimStates = AimStates.LEFTCORNER;
            m_robotContainer.operator.rightTrigger().whileTrue(m_robotContainer.subsystemCommands.preShootLeftCommand());
        }
        else if(aimStates != AimStates.HUBAIM)
        {
            aimStates = AimStates.HUBAIM;
            m_robotContainer.operator.rightTrigger().whileTrue(m_robotContainer.subsystemCommands.preShootCommand());
        }

        // AdvantageScope via Advantagkit trying to log the motor positions in 3d field
        //Intakeangle
        // Pose2d robotPose =
        //     m_robotContainer.swerve.getState().Pose;
        // double intakeAngle = 
        //     m_robotContainer.intake.pivotMotor.getPosition().getValueAsDouble();
        // double climberPos =
        //     m_robotContainer.hanger.motor.getPosition().getValueAsDouble();
        // Logger.recordOutput(
        //     "Robot Mechanisms",  new Pose3d[] {

        // // Intake pivot
        // new Pose3d(
        //     robotPose.getX() + 0.25,
        //     robotPose.getY(),
        //     0.15,
        //     new Rotation3d(
        //         0,
        //         intakeAngle,
        //         robotPose.getRotation().getRadians()
        //     )),
        // new Pose3d(
        //     robotPose.getX(),
        //     robotPose.getY(),
        //     climberPos,
        //     new Rotation3d(
        //         0,
        //         0,
        //         robotPose.getRotation().getRadians()
        //     )
        // )
        // });
            
            // Shooter angle
      //  m_robotContainer.swerve.updateSimState(defaultPeriodSecs, defaultPeriodSecs);
        //Odometry.update(GyroIOPigeon2., )

        //System.out.println(m_robotContainer.swerve.getState().Pose);
       // System.out.println(m_robotContainer.driver.getRawAxis());
    //    var state = m_robotContainer.swerve.getState();

    //     var flTarget = state.ModuleTargets[0];
    //     var flActual = state.ModuleStates[0];

    //     SmartDashboard.putNumber("FL Target Speed",
    //         flTarget.speedMetersPerSecond);

    //     SmartDashboard.putNumber("FL Actual Speed",flActual.speedMetersPerSecond);
    //     SmartDashboard.putNumber("FL Speed Error", flTarget.speedMetersPerSecond - flActual.speedMetersPerSecond);
    //     var constrainTargetAngle = ((flTarget.angle.getDegrees() + 180) % 360 + 360) % 360 - 180;
    //     var constrainActualAngle = ((flActual.angle.getDegrees() + 180) % 360 + 360) % 360 - 180;
    //     SmartDashboard.putNumber("FL Target Angle",
    //         constrainTargetAngle);

    //     SmartDashboard.putNumber("FL Actual Angle", constrainActualAngle);

    //     SmartDashboard.putNumber("FL Angle Error", MathUtil.angleModulus(constrainTargetAngle - constrainActualAngle));

       //System.out.println(TunerConstants.FrontLeft.)
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
       autonomousCommand = m_robotContainer.getAutonomousCommand();

        //schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
       // CommandScheduler.getInstance().schedule(m_robotContainer.autoRoutines.getAutonomousCommand());
      // m_robotContainer.autoInit();
    }

    @Override
    public void autonomousPeriodic()
    {
        //System.out.println(m_r obotContainer.swerve.getState().Pose);
    }

    @Override
    public void teleopPeriodic() {
        // SignalLogger.writeValue("SwerveMotorVelocity",
        //  RotationsPerSecond.of(m_robotContainer.swerve.getModule(0).getSteerMotor().getVelocity().getValueAsDouble()));
        //  SignalLogger.writeValue("SwerveMotorPosition",
        //  RotationsPerSecond.of(m_robotContainer.swerve.getModule(0).getSteerMotor().getPosition().getValueAsDouble()));
        //   SignalLogger.writeValue("SwerveMotorVoltage",
        //  Volts.of(m_robotContainer.swerve.getModule(0).getSteerMotor().getMotorVoltage().getValueAsDouble()));
    }

    // @Override
    // public void teleopInit() {
    //     m_robotContainer.swerve.seedFieldCentric();
    // }
      /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        m_robotContainer.swerve.updateSimState(
            0.02,
            RobotController.getBatteryVoltage()
        );
    }
}
