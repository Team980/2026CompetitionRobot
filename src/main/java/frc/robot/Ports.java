package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.robot.generated.TunerConstants;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("CANmeloAnthony");

    // Talon FX IDs
    public static final int kIntakePivot = 50; // motor let pivot right
    public static final int kIntakeRollers = 51; // motor on slamtake
    public static final int kFloor = 52; // left conveyor belt floor
    public static final int kFeeder = 53; // left preshoot motors
    public static final int kShooterLeft = 54;
    public static final int kShooterMiddle = 55;
    public static final int kShooterRight = 56;
    public static final int kHanger = 58; //right motor on hangar
   // public static final int kHangerRight = 59;

    // PWM Ports
    public static final int kHoodLeftServo = 0;
    public static final int kHoodRightServo = 1;
}
