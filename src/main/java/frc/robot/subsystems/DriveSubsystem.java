package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    public static Joystick xBoxController = new Joystick(0);

    private static final TalonSRX Right_Drive_Motor_1 = new TalonSRX(Constants.Right_Drive_1);
    private static final TalonSRX Right_Drive_Motor_2 = new TalonSRX(Constants.Right_Drive_2);
    private static final TalonSRX Left_Drive_Motor_1 = new TalonSRX(Constants.Left_Drive_1);
    private static final TalonSRX Left_Drive_Motor_2 = new TalonSRX(Constants.Left_Drive_2);

    private static double RightYAxis;
    public static boolean tankDrive = false;
    public static int tankDriveInvert = 1;

    public static double tx;
    public static double ty;
    public static double ta;
    public static boolean auto = false;

    public static void driveInit() {
        Right_Drive_Motor_2.follow(Right_Drive_Motor_1);
        Left_Drive_Motor_2.follow(Left_Drive_Motor_1);

        // Right_Drive_Motor_1.setInverted(false);
        // Left_Drive_Motor_1.setInverted(false);

        Right_Drive_Motor_2.setInverted(InvertType.OpposeMaster);
        // Left drive only works with followmaster
        Left_Drive_Motor_2.setInverted(InvertType.FollowMaster);
    }

    public DriveSubsystem() {
    }

    public CommandBase stop() {
        return run(
                () -> {
                    Left_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                    Right_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                });
    }

    public CommandBase xBoxButtonA() {
        return run(
                () -> {
                    tankDrive = true;
                });
    }

    public CommandBase xBoxButtonX() {
        return run(
                () -> {
                    tankDrive = false;
                });
    }

    public CommandBase xBoxButtonY() {
        return run(
                () -> {
                    auto = true;
                });
    }

    public CommandBase xBoxButtonB() {
        return run(
                () -> {
                    auto = false;
                });
    }

    public static void drivePeriodic() {
        double xBoxLeftYAxis = xBoxController.getRawAxis(1);
        double xBoxLeftXAxis = xBoxController.getRawAxis(0);
        double RightYAxis = xBoxController.getRawAxis(5);

        double LeftYAxis = xBoxLeftYAxis;
        double LeftXAxis = xBoxLeftXAxis;

        // SmartDashboard.putNumber("note (tx)", tx);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(1);

        if (auto == true) {

            if (tx != 0) {
                if (Math.abs(tx) > 10 - ty / 2) {
                    if (tx > 10 - ty / 2) {
                        Left_Drive_Motor_1.set(ControlMode.PercentOutput, 0.2);
                        Right_Drive_Motor_1.set(ControlMode.PercentOutput, 0.2);
                    } else if (tx < 10 - ty / 2) {
                        Left_Drive_Motor_1.set(ControlMode.PercentOutput, -0.2);
                        Right_Drive_Motor_1.set(ControlMode.PercentOutput, -0.2);
                    }
                } else {
                    Left_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                    Right_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                }
            } else {
                Left_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                Right_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
            }
        } else {

            if (tankDrive == false) {

                if (Constants.Deadzone_Factor <= Math.abs(LeftYAxis)
                        && Constants.Deadzone_Factor >= Math.abs(LeftXAxis)) {
                    // Driving Straight
                    Left_Drive_Motor_1.set(ControlMode.PercentOutput, LeftYAxis * 0.8);
                    Right_Drive_Motor_1.set(ControlMode.PercentOutput, LeftYAxis * 0.8);

                } else if (Constants.Deadzone_Factor >= Math.abs(LeftYAxis)
                        && Constants.Deadzone_Factor <= Math.abs(LeftXAxis)) {
                    // Turning In Place
                    Left_Drive_Motor_1.set(ControlMode.PercentOutput, LeftXAxis);
                    Right_Drive_Motor_1.set(ControlMode.PercentOutput, -LeftXAxis);

                } else {

                    if (Constants.Deadzone_Factor >= Math.abs(LeftYAxis)) {
                        // Disable Auto-Drive
                        Left_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                        Right_Drive_Motor_1.set(ControlMode.PercentOutput, 0);

                    } else {
                        // Turning While Driving
                        Left_Drive_Motor_1.set(ControlMode.PercentOutput, (LeftYAxis) -
                                (-LeftXAxis));
                        Right_Drive_Motor_1.set(ControlMode.PercentOutput, (LeftYAxis) -
                                (LeftXAxis));
                    }
                }
            } else if (tankDrive == true && Constants.Deadzone_Factor <= Math.abs(LeftYAxis)
                    || tankDrive == true && Constants.Deadzone_Factor <= Math.abs(RightYAxis)) {

                if (Constants.Deadzone_Factor <= Math.abs(LeftYAxis)
                        || Constants.Deadzone_Factor <= Math.abs(RightYAxis)) {
                    Left_Drive_Motor_1.set(ControlMode.PercentOutput, LeftYAxis);
                    Right_Drive_Motor_1.set(ControlMode.PercentOutput, RightYAxis);
                }
            } else if (tankDrive == true) {
                Left_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
                Right_Drive_Motor_1.set(ControlMode.PercentOutput, 0);
            }
        }
    }
}
