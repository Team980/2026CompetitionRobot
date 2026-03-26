package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
//import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;
import frc.robot.*;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    // private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6.0);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

   
    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = //Alliance.Red == Robot.alliance ? 
        Landmark.HUB.get(Robot.alliance).getTranslation()
      //  :  Landmark.HUBRED.get().getTranslation()
       ;//add alliance if not working right now
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();

        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    //  private Rotation2d getDirectionToHubPredicted() {

    //     double distanceMeters = swerve.getState().Pose.getTranslation().getDistance(Landmark.HUB.get().getTranslation());

    //     PrepareShotCommand.Shot shot = PrepareShotCommand.distanceToShotMap.get(Meters.of(distanceMeters));

    //     // Compute flight time for parabolic shot
    //     double flightTime = PrepareShotCommand.getFlightTimeParabolic(
    //         distanceMeters,
    //         shot.shooterRPM,
    //         0.05,               // shooter wheel radius (m)
    //         Math.toRadians(45), // launch angle
    //         0.8,                // shooter height (m)
    //         Inch.of(72).in(Meters)                // hub height (m)
    //     );

    //     Translation2d predictedPosition = getPredictedRobotPosition(flightTime);

    //     Translation2d hubPosition = Landmark.HUB.get().getTranslation();
    //     Rotation2d hubDirection = hubPosition.minus(predictedPosition).getAngle();


    //     double yawLead = swerve.getChassisSpeeds().omegaRadiansPerSecond * flightTime;
    //     hubDirection = hubDirection.plus(new Rotation2d(yawLead));

    //     return hubDirection.rotateBy(swerve.getOperatorForwardDirection());
    // }

    // private Translation2d getPredictedRobotPosition(double time) {
    //     var pose = swerve.getState().Pose;
    //     var speeds = swerve.getChassisSpeeds();

    //     return pose.getTranslation().plus(
    //         new Translation2d(
    //             speeds.vxMetersPerSecond * time,
    //             speeds.vyMetersPerSecond * time
    //         )
    //     );
    // }


    private Rotation2d getDirectionToTower() {
        final Translation2d towerPos = Landmark.TOWER.get().getTranslation();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = towerPos.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = 
            hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }


    public Rotation2d getTargetAngle()
    {
        return getDirectionToHub();
    }
    public double targetAngle = 0;
    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        Rotation2d targetDirection = getDirectionToHub();
        targetAngle = targetDirection.getDegrees();
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                .withVelocityY(Driving.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub())
        );
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


     @Override
    public void initSendable(SendableBuilder builder) {
        // if(!hasRun)
        // {
             builder.addDoubleProperty("Hub X get", 
             () ->Landmark.HUB.get().getTranslation().getX(), (null));

             builder.addDoubleProperty("Hub Y", 
             () -> Landmark.HUB.get().getTranslation().getY(), (null));

             builder.addDoubleProperty("Hub Y", 
             () -> targetAngle, (targetAngle) -> getDirectionToHub().getDegrees());

             builder.addBooleanProperty("IsBlue", () -> Robot.alliance == Alliance.Blue, (null));
        //}
       
    }

}
