// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AimAndDriveCommand;
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
import frc.robot.subsystems.OneShooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PlotLandmarks;
import frc.util.DriveInputSmoother;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static boolean isInTeleop = false;
    public static Pose2d lastPose;
    public static double restrictedSpeed = 0.4;
    public static double unrestrictedSpeed = 0.8;
    public static double speedFactor = unrestrictedSpeed;
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
    public final Swerve swerve = new Swerve();
    private final Intake intake = null;
    private final Floor floor = null;
    private final Feeder feeder = null;
    //private final Shooter shooter = null;
    private final OneShooter shooter = new OneShooter();
    private final Hood hood = null;
    private final Hanger hanger = null;
    private final Limelight limelight = new Limelight("limelight-pdp");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    public final CommandXboxController driver = new CommandXboxController(0);

    public static void restrictSpeed() {
        speedFactor = restrictedSpeed;
        DriveInputSmoother.joystickDeadband = 0.1*speedFactor;
    }
    public static void unrestrictSpeed() {
        speedFactor = unrestrictedSpeed;
        DriveInputSmoother.joystickDeadband = 0.1*speedFactor;
    }
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
        () -> -driver.getLeftY()*speedFactor,
        () -> -driver.getLeftX()*speedFactor // reduced the speed
    );

   public static PathConstraints constraints =
  new PathConstraints(0.85, 0.85, 0.85, 0.85);
   // public static PathConstraints constraints =
    //  new PathConstraints(2.25, 2, Units.degreesToRadians(540), Units.degreesToRadians(720));
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
       // boolean isCompetition = true;
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> isCompetition
        // ? stream.filter(auto -> auto.getName().startsWith("")) : stream);
        SmartDashboard.putData("AutoChooser", autoChooser);
        swerve.resetPose(Pose2d.kZero);
        lastPose = swerve.getState().Pose;
       // System.out.println(limelight);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
   }


    public void registerNamedCommands() {
        // doesnt work
        NamedCommands.registerCommand("FaceSingularTag", 
        (AutoBuilder.pathfindToPose(Swerve.isTargeting() ? LimelightHelpers.getTargetPose3d_RobotSpace("limelight-pdp").toPose2d() : 
        Landmark.OUTPOST.get(), constraints, 0)));

        // works
        NamedCommands.registerCommand("Start", 
        (AutoBuilder.pathfindToPose(Landmark.RIGHT_START.get(new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg)), constraints,0)));

        NamedCommands.registerCommand("Tower", 
        (AutoBuilder.pathfindToPose(Landmark.TOWER.get(new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 5 + 36), Inches.of(0), Rotation2d.k180deg)), constraints,0)));

        NamedCommands.registerCommand("Outpost", 
        (AutoBuilder.pathfindToPose(Landmark.OUTPOST.get(new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 10), Inches.of(0), Rotation2d.k180deg)), constraints,0)));

        NamedCommands.registerCommand("Depot", 
        (AutoBuilder.pathfindToPose(Landmark.DEPOT.get(new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg)), constraints,0)));
        
        NamedCommands.registerCommand("Hub", 
        (AutoBuilder.pathfindToPose(Landmark.HUB.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero)), constraints,0)));
         
        NamedCommands.registerCommand("RightBump", 
        (AutoBuilder.pathfindToPose(Landmark.RIGHT_BUMP.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints,0)));
        
        NamedCommands.registerCommand("LeftBump", 
        (AutoBuilder.pathfindToPose(Landmark.LEFT_BUMP.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints,0)));
        
        NamedCommands.registerCommand("RightTrench", 
        (AutoBuilder.pathfindToPose(Landmark.RIGHT_TRENCH.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints,0)));
        
        NamedCommands.registerCommand("LeftTrench", 
        (AutoBuilder.pathfindToPose(Landmark.LEFT_TRENCH.get(new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.fromDegrees(0))), constraints,0)));
        
        NamedCommands.registerCommand("AimToHub", 
        (subsystemCommands.testAim())
        );

        NamedCommands.registerCommand("Shoot", subsystemCommands.shootHalf());
        NamedCommands.registerCommand("StopShooter", subsystemCommands.stopShooter());
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
       // if(isInTeleop)
            limelight.setDefaultCommand(updateVisionCommand());
       /*  else
        {
            limelight.setDefaultCommand(slowUpdateVisionCommand());
        }*/
        

        // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
        //     .onTrue(intake.homingCommand())
        //     .onTrue(hanger.homingCommand());

        //driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
       //  driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        // driver.leftTrigger().whileTrue(intake.intakeCommand());
        // driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
       /// System.out.println("subsystemCommands:"  + subsystemCommands);

        driver.povUp().whileTrue(subsystemCommands.shootHalf());
        driver.povDown().whileTrue(subsystemCommands.stopShooter());
        driver.rightTrigger().whileTrue(subsystemCommands.testAim());

       /* driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));*/
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY()*speedFactor, 
            () -> -driver.getLeftX()*speedFactor, 
            () -> -driver.getRightX()*speedFactor
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.leftBumper().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
        driver.leftTrigger()
            .onTrue(Commands.runOnce(() -> restrictSpeed()))
            .onFalse(Commands.runOnce(() -> unrestrictSpeed()));
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
    final double maxDistanceChange = 1;
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
           // System.out.println(measurement);
           
            measurement.ifPresent(m -> {
                
                if (!hasSeededPose && InBoundsCheck(m.poseEstimate.pose)) {
                    swerve.resetPose(m.poseEstimate.pose);
                   // swerve.seedFieldCentric(m.poseEstimate.pose.getRotation());
                    hasSeededPose = true;
                  //  System.out.println("SEEDED FIELD POSE");
                }

               double distance = m.poseEstimate.pose.getTranslation().getDistance(swerve.getState().Pose.getTranslation());

                if (distance < maxDistanceChange && InBoundsCheck(m.poseEstimate.pose)) {
                    swerve.addVisionMeasurement(
                        m.poseEstimate.pose, 
                        m.poseEstimate.timestampSeconds,
                        m.standardDeviations
                    );
                }
               // }
                // swerve.resetRotation(m.poseEstimate.pose.getRotation());
            });
                
            // });
           // swerve.resetPose(new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(50), Rotation2d.fromDegrees(180)));
            // measurement.ifPresent(m -> {
            //     swerve.resetPose(m.poseEstimate.pose);
            // });
        })
        .ignoringDisable(true);
    }


    public boolean InBoundsCheck(Pose2d visionPose)
    {
        double fieldLength = Constants.FieldConstants.FIELD_WIDTH.in(Meter); 
        double fieldWidth = Constants.FieldConstants.FIELD_HEIGHT.in(Meter); 
        if (visionPose.getX() < 0 || visionPose.getX() > fieldLength || visionPose.getY() < 0 || visionPose.getY() > fieldWidth) 
        { 
            return false;
        }
        return true;
    }
    private final float maxError = 0.1f;
    
    private Command slowUpdateVisionCommand() {
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
           // System.out.println(measurement);
            measurement.ifPresent(m -> {
                if (!hasSeededPose) {
                    swerve.resetPose(m.poseEstimate.pose);
                    hasSeededPose = true;
                  //  System.out.println("SEEDED FIELD POSE");
                }
                //System.out.println(m.poseEstimate.pose);
                
                // Translation2d distance = lastPose.getTranslation().minus(m.poseEstimate.pose.getTranslation());//- m.poseEstimate.pose.getTranslation().getX();
                // if(Math.abs(distance.getX()) > maxError || Math.abs(distance.getY()) > maxError)
                // {
                    swerve.addVisionMeasurement(
                        m.poseEstimate.pose, 
                        m.poseEstimate.timestampSeconds,
                        m.standardDeviations
                    );
                    lastPose = m.poseEstimate.pose;
                //}
                // swerve.resetRotation(m.poseEstimate.pose.getRotation());
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
