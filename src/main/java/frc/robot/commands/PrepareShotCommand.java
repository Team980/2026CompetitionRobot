package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmark;
import frc.robot.LimelightHelpers;
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

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }

    private final OneShooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Swerve swerve;

    public PrepareShotCommand(OneShooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier, Swerve swerve) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        this.swerve = swerve;

        //TODO: Uncomment all hood code later when ready
        //addRequirements(shooter, hood);
        addRequirements(shooter);
    }

    // public boolean isReadyToShoot() {
    //     return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    // }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmark.HUB.get().getTranslation();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

     private Distance getDistanceToHubPredicted() {
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        // Compute flight time for parabolic shot
        double flightTime = PrepareShotCommand.getFlightTimeParabolic(
            getDistanceToHub().in(Meters),
            distanceToShotMap.get(getDistanceToHub()).shooterRPM,
            0.05,               // shooter wheel radius (m)
            Math.toRadians(45), // launch angle
            0.8,                // shooter height (m)
            Inch.of(72).in(Meters)                // hub height (m)
        );
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation().plus
        (new Translation2d(speeds.vxMetersPerSecond * flightTime
        , speeds.vyMetersPerSecond * flightTime));

        final Translation2d hubPosition = Landmark.HUB.get().getTranslation();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    /**
     * Calculates the flight time of a ball shot in an arc.
     *
     * @param distanceMeters Horizontal distance to target (m)
     * @param shooterRPM Shooter wheel speed (RPM)
     * @param wheelRadiusMeters Radius of shooter wheel (m)
     * @param launchAngleRadians Shooter launch angle relative to horizontal (rad)
     * @param shooterHeightMeters Shooter exit height (m)
     * @param targetHeightMeters Target height (m)
     * @return Flight time in seconds
     */
    public static double getFlightTimeParabolic(
            double distanceMeters,
            double shooterRPM,
            double wheelRadiusMeters,
            double launchAngleRadians,
            double shooterHeightMeters,
            double targetHeightMeters) 
        {
        // Convert RPM to linear ball speed (v = ω * r)
        double v0 = shooterRPM * 2 * Math.PI * wheelRadiusMeters / 60.0; // m/s

        // Horizontal and vertical components
        double vx = v0 * Math.cos(launchAngleRadians);
        double vy = v0 * Math.sin(launchAngleRadians);

        // Gravity
        double g = 9.81;

        // Solve quadratic for flight time based on vertical motion:
        // 0.5 * g * t^2 - vy * t + (targetHeight - shooterHeight) = 0
        double a = 0.5 * g;
        double b = -vy;
        double c = targetHeightMeters - shooterHeightMeters;

        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) {
            // No real solution — target too high or too far
            return distanceMeters / vx; // fallback to horizontal-only approximation
        }

        double t1 = (-b + Math.sqrt(discriminant)) / (2*a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2*a);

        // Choose positive, realistic flight time
        double tFlight = Math.max(t1, t2);
        if (tFlight < 0.01) tFlight = distanceMeters / vx; // fallback if negative

        return tFlight;
    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        //shooter.setRPM(shot.shooterRPM);
        shooter.setPercentOutput(0.5);
        //hood.setPosition(shot.hoodPosition);

        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

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
