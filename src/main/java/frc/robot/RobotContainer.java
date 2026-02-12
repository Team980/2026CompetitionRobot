// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PlotLandmarks;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // For landmark visualizaton
    private final PlotLandmarks plotter = new PlotLandmarks();
    private final SendableChooser<Command> autoChooser;
    // private final Swerve swerve = new Swerve();
    // private final Intake intake = new Intake();
    // private final Floor floor = new Floor();
    // private final Feeder feeder = new Feeder();
    // private final Shooter shooter = new Shooter();
    // private final Hood hood = new Hood();
    // private final Hanger hanger = new Hanger();
    private final Swerve swerve = new Swerve();
    private final Intake intake = null;
    private final Floor floor = null;
    private final Feeder feeder = null;
    private final Shooter shooter = null;
    private final Hood hood = null;
    private final Hanger hanger = null;
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    // private final AutoRoutines autoRoutines = new AutoRoutines(
    //     swerve,
    //     null,
    //     null,
    //     null,
    //     null,
    //     null,
    //     null,
    //     limelight
    // );

    
    // private final SubsystemCommands subsystemCommands = new SubsystemCommands(
    //     swerve,
    //     null,
    //     null,
    //     null,
    //     null,
    //     null,
    //     null,
    //     () -> -driver.getLeftY(),
    //     () -> -driver.getLeftX()
    // );
    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    );

    
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
   
    public static PathConstraints constraints =
      new PathConstraints(2.25, 2, Units.degreesToRadians(540), Units.degreesToRadians(720));
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        boolean isCompetition = true;
       autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> isCompetition
        // ? stream.filter(auto -> auto.getName().startsWith("")) : stream);
      //  SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
   }


    public void registerNamedCommands() {
        NamedCommands.registerCommand("Hub", 
        (AutoBuilder.pathfindToPose(Landmarks.hubPosition(), constraints,0)));
         
        NamedCommands.registerCommand("Tower", 
        (AutoBuilder.pathfindToPose(Landmarks.towerPosition(), constraints,0)));

        NamedCommands.registerCommand("Outpost", 
        (AutoBuilder.pathfindToPose(Landmarks.outpostPosition(), constraints,0)));
        
        NamedCommands.registerCommand("RightBump", 
        (AutoBuilder.pathfindToPose(Landmarks.rightBumpPosition(), constraints,0)));
        
        NamedCommands.registerCommand("LeftBump", 
        (AutoBuilder.pathfindToPose(Landmarks.leftBumpPosition(), constraints,0)));
        
        NamedCommands.registerCommand("RightTrench", 
        (AutoBuilder.pathfindToPose(Landmarks.rightTrenchPosition(), constraints,0)));
        
        NamedCommands.registerCommand("LeftTrench", 
        (AutoBuilder.pathfindToPose(Landmarks.leftTrenchPosition(), constraints,0)));
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());
        

        /*RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        driver.leftTrigger().whileTrue(intake.intakeCommand());
        driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));*/
        driver.rightTrigger().whileTrue(subsystemCommands.testAim());

       /* driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));*/
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.leftBumper().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }

    // private Command updateVisionCommand() {
    //     return limelight.run(() -> {
    //         final Pose2d currentRobotPose = swerve.getState().Pose;
    //         final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
    //         measurement.ifPresent(m -> {
    //             swerve.addVisionMeasurement(
    //                 m.poseEstimate.pose, 
    //                 m.poseEstimate.timestampSeconds,
    //                 m.standardDeviations
    //             );
    //         });
    //     })
    //     .ignoringDisable(true);
    // }
    boolean hasSeededPose = false;
    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            //if(button.onTrue(System.out.println(measurement)))
            // measurement.ifPresent(m -> {
            //     swerve.addVisionMeasurement(
            //         m.poseEstimate.pose, 
            //         m.poseEstimate.timestampSeconds,
            //         m.standardDeviations
            //     );
            System.out.println(measurement);
            measurement.ifPresent(m -> {
                if (!hasSeededPose) {
                    swerve.resetPose(m.poseEstimate.pose);
                    hasSeededPose = true;
                    System.out.println("SEEDED FIELD POSE");
                }
                System.out.println(m.poseEstimate.pose);
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
                
            // });
           // swerve.resetPose(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(50), Rotation2d.fromDegrees(180)));
            // measurement.ifPresent(m -> {
            //     swerve.resetPose(m.poseEstimate.pose);
            // });
        })
        .ignoringDisable(true);
    }
}
