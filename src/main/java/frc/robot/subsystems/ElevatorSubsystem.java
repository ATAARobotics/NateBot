// Encoder Distance Between Top & Bottom: 15205
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
    public static Joystick xBoxController = new Joystick(0);

    // public static DigitalInput bottomStop = new DigitalInput(0);
    public static DigitalInput topStop = new DigitalInput(1);

    private static final TalonSRX Elevator_Motor_1 = new TalonSRX(Constants.Elevator_1);
    private static final TalonSRX Elevator_Motor_2 = new TalonSRX(Constants.Elevator_2);

    public Counter Encoder;

    public double RightTrigger;
    public double LeftTrigger;

    public static boolean tankDrive = false;
    public static int tankDriveInvert = 1;

    public double trigger;
    public double ElevatorDistance = 0;

    public ElevatorSubsystem() {
        Encoder = new Counter(0);
        Encoder.setDistancePerPulse(100);
        Elevator_Motor_2.follow(Elevator_Motor_1);

        Elevator_Motor_1.setInverted(false);
        Elevator_Motor_2.setInverted(InvertType.OpposeMaster);

        Elevator_Motor_1.set(ControlMode.PercentOutput, -1);
    }

    public void periodic() {

        // boolean bottomStop_Boolean = bottomStop.get();
        // boolean topStop_Boolean = topStop.get();
        LeftTrigger = xBoxController.getRawAxis(2);
        RightTrigger = xBoxController.getRawAxis(3);
        trigger = RightTrigger - LeftTrigger;

        if (Math.abs(trigger) >= Constants.Deadzone_Factor) {
            Elevator_Motor_1.set(ControlMode.PercentOutput, -trigger);
        } else {
            Elevator_Motor_1.set(ControlMode.PercentOutput, 0);
        }

        // Elevator Distance Up & Down

        String ElevatorDirection;

        if (trigger >= Constants.Deadzone_Factor) {
            ElevatorDirection = "Up";
        } else if (trigger <= -Constants.Deadzone_Factor) {
            ElevatorDirection = "Down";
        } else {
            ElevatorDirection = "Stopped";
        }

        SmartDashboard.putString("ElevatorDirection", ElevatorDirection);
        SmartDashboard.putBoolean("ElevatorStopped", Encoder.getStopped());
        SmartDashboard.putNumber("Encoder", Encoder.get());
    }

    public CommandBase rightBumper() {
        return run(
                () -> {
                    Elevator_Motor_1.set(ControlMode.PercentOutput, 1);
                });
    }

    public CommandBase leftBumper() {
        return run(
                () -> {
                    Elevator_Motor_1.set(ControlMode.PercentOutput, -1);
                    ;
                });
    }

    public CommandBase xBoxButtonB() {
        return run(
                () -> {
                    trigger = 0;
                });
    }

}
