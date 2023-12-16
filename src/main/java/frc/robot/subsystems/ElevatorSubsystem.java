package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
    public static Joystick xBoxController = new Joystick(0);

    public static DigitalInput bottomStop = new DigitalInput(0);
    public static DigitalInput topStop = new DigitalInput(1);

    private static final TalonSRX Elevator_Motor_1 = new TalonSRX(Constants.Elevator_1);
    private static final TalonSRX Elevator_Motor_2 = new TalonSRX(Constants.Elevator_2);

    static int button = 0;

    static double RightTrigger;
    static double LeftTrigger;

    public static boolean tankDrive = false;
    public static int tankDriveInvert = 1;

    private static double trigger = 0;

    public static void elevatorInit() {
        Elevator_Motor_2.follow(Elevator_Motor_1);

        Elevator_Motor_1.setInverted(false);
        Elevator_Motor_2.setInverted(InvertType.OpposeMaster);
    }

    public CommandBase rightBumper() {
        return run(
                () -> {
                    if (Math.abs(trigger) < Constants.Deadzone_Factor) {
                        Elevator_Motor_1.set(ControlMode.PercentOutput, 1);
                    } else if (Math.abs(trigger) > Constants.Deadzone_Factor) {
                        Elevator_Motor_1.set(ControlMode.PercentOutput, trigger);
                    }
                });
    }

    public CommandBase leftBumper() {
        return run(
                () -> {
                    if (Math.abs(trigger) < Constants.Deadzone_Factor) {
                        Elevator_Motor_1.set(ControlMode.PercentOutput, -1);
                    } else if (Math.abs(trigger) > Constants.Deadzone_Factor) {
                        Elevator_Motor_1.set(ControlMode.PercentOutput, trigger);
                    }
                });
    }

    public CommandBase xBoxButtonB() {
        return run(
                () -> {
                    if (Math.abs(trigger) < Constants.Deadzone_Factor) {
                        Elevator_Motor_1.set(ControlMode.PercentOutput, 0);
                    } else if (Math.abs(trigger) > Constants.Deadzone_Factor) {
                        Elevator_Motor_1.set(ControlMode.PercentOutput, trigger);
                    }
                });
    }

    public static void elevatorPeriodic() {

        boolean bottomStop_Boolean = bottomStop.get();
        boolean topStop_Boolean = topStop.get();
        double LeftTrigger = xBoxController.getRawAxis(2);
        double RightTrigger = xBoxController.getRawAxis(3);
        trigger = RightTrigger - LeftTrigger;

        Elevator_Motor_1.set(ControlMode.PercentOutput, trigger);
    }
}
