package frc.robot.subsystems.Algae;

public final class AlgaeConstants {

    public static final int kAlgaeCanId = 22;

    public static final int algaeCurrentLimit = 20;
    public static final double kAlgaeEjectSpeed = 1;
    public static final double kAlgaeIntakeSpeed = 1;

    // TEMPORARY VALUES
    public static double kAlgaeMinOutput = -0.1;
    public static double kAlgaeMaxOutput = 0.1;

    // TEMPORARY VALUES
    public static boolean kAlgaeInverted = true;
    // TEMPORARY VALUES
    public static int kAlgaePositionConversionFactor = 1000;
    public static int kAlgaeVelocityConversionFactor = 1000;
    // TEMPORARY VALUES
    public static double kAlgaeP = .001;
    public static double kAlgaeI = 0.0;
    public static double kAlgaeD = 0.0;
    public static double kAlgaeFF = 0.0;

    // Beam Break Sensor
    public static int kIRPort = 4;

    // Current Limit for Algae Stall
    public static double kAlgaeCurrentLimit = 15;
    public static double kAlgaeCurrentDebouncerTime = .2;

}
