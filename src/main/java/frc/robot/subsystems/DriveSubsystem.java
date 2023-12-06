package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class DriveSubsystem {
    public static Joystick xBoxController = new Joystick(0);

    private static final TalonSRX Right_Drive_Motor_1 = new TalonSRX(Constants.Right_Drive_1);
    private static final TalonSRX Right_Drive_Motor_2 = new TalonSRX(Constants.Right_Drive_2);
    private static final TalonSRX Left_Drive_Motor_1 = new TalonSRX(Constants.Left_Drive_1);
    private static final TalonSRX Left_Drive_Motor_2 = new TalonSRX(Constants.Left_Drive_2);

    public static void driveInit() {
        Right_Drive_Motor_2.follow(Right_Drive_Motor_1);
        Left_Drive_Motor_2.follow(Left_Drive_Motor_1);

        Right_Drive_Motor_1.setInverted(false);
        Left_Drive_Motor_1.setInverted(false);

        Right_Drive_Motor_2.setInverted(InvertType.OpposeMaster);
        // Left drive only works with followmaster
        Left_Drive_Motor_2.setInverted(InvertType.FollowMaster);
    }

    public static void drivePeriodic() {
        double xBoxLeftYAxis = xBoxController.getRawAxis(1);
        double xBoxLeftXAxis = xBoxController.getRawAxis(0);

        double LeftYAxis = -xBoxLeftYAxis;
        double LeftXAxis = -xBoxLeftXAxis;

        if (Constants.Deadzone_Factor >= Math.abs(xBoxLeftYAxis)) {
            LeftYAxis = 0.0;
        }

        if (Constants.Deadzone_Factor >= Math.abs(xBoxLeftXAxis)) {
            LeftXAxis = 0.0;
        }

        if (Constants.Deadzone_Factor <= Math.abs(xBoxLeftYAxis) && Constants.Deadzone_Factor >= Math.abs(xBoxLeftXAxis)) {
            Left_Drive_Motor_1.set(ControlMode.PercentOutput, LeftYAxis * 0.8);
            Right_Drive_Motor_1.set(ControlMode.PercentOutput, LeftYAxis * 0.8);
        } else if (Constants.Deadzone_Factor >= Math.abs(xBoxLeftYAxis) && Constants.Deadzone_Factor <= Math.abs(xBoxLeftXAxis)) {
            Left_Drive_Motor_1.set(ControlMode.PercentOutput, -LeftXAxis * 0.8);
            Right_Drive_Motor_1.set(ControlMode.PercentOutput, LeftXAxis * 0.8);
        } else {
            Left_Drive_Motor_1.set(ControlMode.PercentOutput, (LeftYAxis * 0.8) - (LeftXAxis * 0.8));
            Right_Drive_Motor_1.set(ControlMode.PercentOutput, (LeftYAxis * 0.8) - (-LeftXAxis * 0.8));
        }
        
    }
}
