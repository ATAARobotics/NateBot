package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem {
    public static Joystick xBoxController = new Joystick(0);

    public static DigitalInput bottomStop = new DigitalInput(0);
    public static DigitalInput topStop = new DigitalInput(1);

    private static final TalonSRX Elevator_Motor_1 = new TalonSRX(Constants.Elevator_1);
    private static final TalonSRX Elevator_Motor_2 = new TalonSRX(Constants.Elevator_2);

    public static void elevatorInit() {
        Elevator_Motor_2.follow(Elevator_Motor_1);

        Elevator_Motor_1.setInverted(false);
        Elevator_Motor_2.setInverted(InvertType.OpposeMaster);
    }

    public static void elevatorPeriodic() {
        boolean bottomStop_Boolean = bottomStop.get();
        boolean topStop_Boolean = topStop.get();

        double xBoxLeftTrigger = xBoxController.getRawAxis(2);
        double xBoxRightTrigger = xBoxController.getRawAxis(3);

        double xBoxTrigger = 0.0;

        if (Constants.Deadzone_Factor <= Math.abs(xBoxRightTrigger - 0)) {
        xBoxTrigger = xBoxRightTrigger;
        }
        if (Constants.Deadzone_Factor <= Math.abs(xBoxLeftTrigger - 0)) {
        xBoxTrigger = -xBoxLeftTrigger;

        }
        
        if (topStop_Boolean == false && bottomStop_Boolean == false) {
            Elevator_Motor_1.set(ControlMode.PercentOutput, xBoxTrigger * 1.5);
        }
    }
}
