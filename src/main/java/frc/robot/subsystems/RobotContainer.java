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

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.traits.HasTalonSignals;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Driving;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AimAndDriveToRightCommand;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PreparePredictShotCommand;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ModuleIOTalonFX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.OneShooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Floor.Speed;
import frc.robot.subsystems.GyroIOPigeon2;
import frc.robot.subsystems.PlotLandmarks;
import frc.util.DriveInputSmoother;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public static double restrictedSpeed = 0.5; //TODO: Remove Speed restriction
    public static double unrestrictedSpeed = 1;
    public static double restrictedRotation = 0.6;
    public static double unrestrictedRotation = 1;
    public static double speedFactor = unrestrictedSpeed;
    public static double rotationFactor = unrestrictedRotation;
    
    // For landmark visualizaton
    private final PlotLandmarks plotter = new PlotLandmarks();
    private final SendableChooser<Command> autoChooser;
    //private final Swerve swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Floor floor = new Floor();
    public final Feeder feeder = new Feeder();
    public final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    public final Hanger hanger = new Hanger();
    public final Swerve swerve = new Swerve(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
    // private final Intake intake = null;
    // private final Floor floor = null;
    // private final Feeder feeder = null;
    // //private final Shooter shooter = null;
    // private final OneShooter shooter = null;
    // private final Hood hood = null;
    // private final Hanger hanger = null;
    // private final Hanger leftHanger = null; //one is opposite
    private final Limelight limelight = new Limelight("limelight-pdp");

    public static boolean isInCenter = false;
    public static boolean faceRightSide = false;

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    public static void restrictSpeed() {
        speedFactor = restrictedSpeed;
        DriveInputSmoother.joystickDeadband = 0.1*speedFactor;
    }
    public static void unrestrictSpeed() {
        speedFactor = unrestrictedSpeed;
        DriveInputSmoother.joystickDeadband = 0.1*speedFactor;
    }

    public static void restrictRotation()
    {
        rotationFactor = restrictedRotation;
    }

    public static void unrestrictRotation()
    {
        rotationFactor = unrestrictedRotation;
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

    
    public final SubsystemCommands subsystemCommands = new SubsystemCommands(
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
        boolean isCompetition = true;
        registerNamedCommands();
      //  autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("COMP")) : stream);
        SmartDashboard.putData("AutoChooser", autoChooser);
        swerve.resetPose(Pose2d.kZero);
        lastPose = swerve.getState().Pose;
       // System.out.println(limelight);

       swerve.registerTelemetry(state -> {

            swerveTelemetry.telemeterize(state);

            plotter.updateRobotPose(lastPose);

            var flTarget = state.ModuleTargets[0];
            var flActual = state.ModuleStates[0];

            double targetAngle =
                MathUtil.inputModulus(flTarget.angle.getDegrees(), -180, 180);
            double actualAngle =
                MathUtil.inputModulus(flActual.angle.getDegrees(), -180, 180);

            SmartDashboard.putNumber("FL Target Speed",
                flTarget.speedMetersPerSecond);
            SmartDashboard.putNumber("FL Actual Speed",
                flActual.speedMetersPerSecond);

            SmartDashboard.putNumber("FL Target Angle",
                targetAngle);
            SmartDashboard.putNumber("FL Actual Angle",
                actualAngle);

            SmartDashboard.putNumber("FL Speed Error",
                flTarget.speedMetersPerSecond - flActual.speedMetersPerSecond);

            SmartDashboard.putNumber("FL Angle Error",
                targetAngle - actualAngle);
        });
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
   }


    public void registerNamedCommands() {
        // doesnt work
        // NamedCommands.registerCommand("FaceSingularTag", 
        // (AutoBuilder.pathfindToPose(Swerve.isTargeting() ? LimelightHelpers.getTargetPose3d_RobotSpace("limelight-pdp").toPose2d() : 
        // Landmark.OUTPOST.get(), constraints, 0)));

        // works
        NamedCommands.registerCommand("Start", 
        (AutoBuilder.pathfindToPose(Landmark.RIGHT_START.get(new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg)), constraints,0)));

        NamedCommands.registerCommand("Tower", 
        (AutoBuilder.pathfindToPose(Landmark.TOWER.get(new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 5 + 36), Inches.of(0), Rotation2d.k180deg)), constraints,0)));

        NamedCommands.registerCommand("Outpost", 
        (AutoBuilder.pathfindToPose(
            Landmark.OUTPOST.get(new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 1), Inches.of(0), Rotation2d.k180deg)), constraints,0)));
                                                                                                //Measured to be 24, 17.5 + 5 (previously 10)
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

        NamedCommands.registerCommand("DeployIntake", 
            (intake.GoOut())
        );

        NamedCommands.registerCommand("StartIntake", 
            (intake.intakeCommand())
        );

        NamedCommands.registerCommand("IntakeIn", 
            (intake.ReturnIn())
        );

       
        NamedCommands.registerCommand("StopIntake", 
            (intake.stopRollers())
        );

        NamedCommands.registerCommand("Agitate", (intake.agitateCommand()));

        Command parallelFeed = new ParallelCommandGroup(feeder.feedCommand(), floor.feedCommand());
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        NamedCommands.registerCommand("PreShoot", prepareShotCommand);
        NamedCommands.registerCommand("Shoot", parallelFeed);
         Command parallelStop = new ParallelCommandGroup(feeder.stopFeed(), floor.stopFloor());
        NamedCommands.registerCommand("StopFeeds", parallelStop);
        NamedCommands.registerCommand("StopShooter", shooter.stopShootCommand());
        NamedCommands.registerCommand("ClimberUp", hanger.hangarUpCommand());
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
        // driver.povRight()//actaully down

        //     .whileTrue(hanger.manualUpCommand()).whileFalse(hanger.stopCommand());
        // driver.povLeft() //actually up
        //     .whileTrue(hanger.manualDownCommand()).whileFalse(hanger.stopCommand());
        //  driver.povUp()
        //      .whileTrue(hanger.hangarUpCommand()).whileFalse(hanger.stopCommand());
        // operator.a()
        //     .whileTrue(hanger.hangarDownCommand());

        operator.leftTrigger().whileTrue(intake.agitateCommand());

        operator.b()
            .onTrue(intake.GoOut());
        operator.x()
            .onTrue(intake.ReturnIn());
         Command zeroCommand = intake.runOnce(() -> intake.resetDeployEncoder());
         operator.povDown()
             .onTrue(zeroCommand);
        Command stopCommand = intake.runOnce(() -> intake.stopMotor());
          operator.povUp()
            .onTrue(stopCommand);

        Command speedCommand = floor.runOnce(() -> floor.set(Speed.FEED.rpm()));
        driver.povDown().onTrue(hanger.hangarDownCommand());
        //operator.leftTrigger().whileTrue(floor.feedCommand());


    
      //  operator.rightBumper().whileTrue(shooter.dashboardSpinUpCommand()).whileFalse(shooter.stopShootCommand());
        Command parallel = new ParallelCommandGroup(feeder.feedCommand(), floor.feedCommand());
       // operator.rightTrigger().whileTrue(feeder.feedCommand()).whileTrue(floor.feedCommand());
        //whileTrue(shooter.dashboardSpinUpCommand()).whileFalse(shooter.stopShootCommand());
       // Command feedStop = floor.runOnce(() -> floor.set(Speed.STOP));
       //  driver.a().whileTrue(feedStop);
          //  .onTrue(hanger.homingCommand());

      //  driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
       // driver.rightBumper().whileTrue(subsystemCommands.shootManually());
    //   driver.rightBumper().whileTrue(subsystemCommands.shootHalf()).whileFalse(subsystemCommands.stopShooter());
     //    driver.leftTrigger().whileTrue(intake.intakeCommandNoWheels());
     //positive should go outwards
          operator.povRight().whileTrue(intake.percentMoveCommand(0.05)).whileFalse(intake.percentMoveCommand(0.0));
         operator.povLeft().whileTrue(intake.percentMoveCommand(-0.05)).whileFalse(intake.percentMoveCommand(0.0));
       /// System.out.println("subsystemCommands:"  + subsystemCommands);
       
        // driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // /*
        // * Joystick Y = quasistatic forward
        // * Joystick A = quasistatic reverse
        // * Joystick B = dynamic forward
        // * Joystick X = dyanmic reverse
        // */
        // driver.y().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driver.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driver.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driver.x().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // driver.povUp().whileTrue(subsystemCommands.shootHalf());
        // driver.povDown().whileTrue(subsystemCommands.stopShooter());
        // driver.y().onTrue(hood.positionCommand(0.5));
        // driver.a().onTrue(hood.positionCommand(0.25));
        // driver.rightBumper().whileTrue(subsystemCommands.testAim());

        // driver.povRight().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        // driver.povLeft().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    // public Command setRightCorner(boolean isRight)
    // {
    //     if(isRight)
    //     {
    //         return Commands.runOnce(() -> plotter.setLandmarkPose(Landmark.RIGHT_CORNER.get()));
    //     }
    //     else
    //     {
    //         return Commands.runOnce(() -> plotter.setLandmarkPose(Landmark.LEFT_CORNER.get()));
    //     }
    // }
    
    public void seedAgain()
    {
        hasSeededPose = true;
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
        //    (Alliance.Red != Robot.alliance) ? 
            () -> -driver.getLeftY()*speedFactor 
            // : 
            // () -> driver.getLeftY()*speedFactor
            , 
        //    (Alliance.Red != Robot.alliance) ? 
            () -> -driver.getLeftX()*speedFactor
            // :
            // () -> driver.getLeftX()*speedFactor
            , 
            //(Alliance.Red != Robot.alliance) ? 
            () -> -driver.getRightX()*rotationFactor
             //: () -> (-driver.getRightX() + Math.PI) * rotationFactor
        );
        

        if(isInCenter)
        {
            driver.y().whileTrue(subsystemCommands.testAimRightCorner());
            driver.a().whileTrue(subsystemCommands.testAimLeftCorner());
        }

        


       // Commands.runOnce(() -> swerve.SetControl())
        swerve.setDefaultCommand(manualDriveCommand);
       driver.leftBumper().whileTrue(Commands.run(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg.div(2))));
       driver.b().whileTrue(Commands.run(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg.div(2))));
        // driver.povUp().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        // driver.povRight().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        // driver.povLeft().onTrue(Commands.runOnce(()-> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        // driver.povDown().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));

         driver.x().onTrue(Commands.runOnce(() -> seedAgain()));
        driver.leftTrigger()
            .whileTrue(Commands.runOnce(() -> restrictSpeed())).whileFalse(Commands.runOnce(() -> unrestrictSpeed()));
        driver.rightTrigger()
            .whileTrue(Commands.runOnce(() -> restrictRotation())).whileFalse(Commands.runOnce(() -> unrestrictRotation()));
        operator.rightTrigger().whileTrue(subsystemCommands.preShootCommand());
        Command parallel = new ParallelCommandGroup(feeder.feedCommand(), floor.feedCommand());
        Command parallelStop = new ParallelCommandGroup(feeder.stopFeed(), floor.stopFloor());
        operator.rightBumper().whileTrue(parallel).whileFalse(parallelStop);
        if(Robot.alliance != null)
            driver.rightBumper().whileTrue(subsystemCommands.testAim());
        operator.leftBumper().whileTrue(intake.intakeCommand()).whileFalse(intake.stopRollers());

        //driver.povUp().onTrue(updateVisionCommand());
            //.onFalse(Commands.runOnce(() -> unrestrictSpeed()));
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
    public static boolean hasSeededPose = false;
    final double maxDistanceChange = 0.2;
    public static boolean hasEnabled = false;
   // public static int count = 0;
   // int waitCounts = 100;
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
                
                // if (!hasSeededPose && InBoundsCheck(m.poseEstimate.pose)){ //&& InBoundsCheck(m.poseEstimate.pose)) {
                // if (count < waitCounts) {
                if(DriverStation.isDisabled())
                {
                    //TODO: Check if this also works for the red side
                    Rotation2d rotation = (Robot.alliance == Alliance.Red) ? new Rotation2d(180) : new Rotation2d(0);
                    Pose2d driverPose = new Pose2d(m.poseEstimate.pose.getTranslation(), rotation);
                    swerve.resetPose(driverPose);
                    // swerve.resetPose(m.poseEstimate.pose);
                    // if (Robot.alliance == Alliance.Red) {
                    //      swerve.seedFieldCentric(Rotation2d.fromDegrees(180));
                    //     //swerve.resetRotation(Rotation2d.fromDegrees(180));
                    // } else {
                    //      swerve.seedFieldCentric(Rotation2d.fromDegrees(0));
                    //     //swerve.resetRotation(Rotation2d.fromDegrees(0));
                    // }

                  
                   swerve.seedFieldCentric(m.poseEstimate.pose.getRotation());
                    hasSeededPose = true;
                  //  System.out.println("SEEDED FIELD POSE");
                }

               double distance = m.poseEstimate.pose.getTranslation().getDistance(swerve.getState().Pose.getTranslation());

                if //(distance < maxDistanceChange && 
                    (InBoundsCheck(m.poseEstimate.pose)) {
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
    
   
}
