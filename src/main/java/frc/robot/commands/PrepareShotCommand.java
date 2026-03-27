package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmark;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
//import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.OneShooter;
import frc.robot.subsystems.Swerve;

public class PrepareShotCommand extends Command {
    public static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    public static boolean hasRun = false;

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800 + 250, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275 + 300, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650 + 350, 0.48));
    }


    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;
    public static double distanceHub = 0.0;
   // public static boolean isReadyToShoot;
    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;

        //TODO: Uncomment all hood code later when ready
        addRequirements(shooter, hood);
      //  addRequirements(shooter);
      // SmartDashboard.putNumber("Distance to Hub (inches)", distanceHub);
      SmartDashboard.putData(this);
    }


    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    // public boolean isReadyToShoot() {
    //     return shooter.isVelocityWithinTolerance();
    // }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = // Alliance.Red == Robot.alliance ? 
        Landmark.HUB.get().getTranslation()
        // :  Landmark.HUBRED.get().getTranslation()
        ;
       // SmartDashboard.putNumber("Distance From Hub", Meters.of(robotPosition.getDistance(hubPosition)).baseUnitMagnitude());
        return Meters.of(robotPosition.getDistance(hubPosition));
    }


     
    boolean hasHitFirstBatch = false;
  
    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setRPM(shot.shooterRPM);
       
       
        hood.setPosition(shot.hoodPosition);
        
        //SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
        distanceHub = distanceToHub.in(Inches);
        
    }



    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     // if(!hasRun)
    //     // {
    //          builder.addDoubleProperty("Distance to Hub (inches)", () -> distanceHub, (distanceHub) -> getDistanceToHub());
            
    //     //}
       
    // }

    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
