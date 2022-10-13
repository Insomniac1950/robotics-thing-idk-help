package frc.robot;

public class RobotMap {
    public static final int LEFT_FRONT_DRIVE_PORT = 3;
    public static final int RIGHT_FRONT_DRIVE_PORT = 4;
    public static final int LEFT_BACK_DRIVE_PORT = 1;
    public static final int RIGHT_BACK_DRIVE_PORT = 2;
    public static WPI_TalonFX leftFrontDriveMotor = new WPI_TalonFx(LEFT_FRONT_DRIVE_PORT);
    public static WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFx(RIGHT_FRONT_DRIVE_PORT);
    public static WPI_TalonFX leftBackDriveMotor = new WPI_TalonFx(LEFT_BACK_DRIVE_PORT);
    public static WPI_TalonFX rightBackDriveMotor = new WPI_TalonFx(RIGHT_BACK_DRIVE_PORT);
}
