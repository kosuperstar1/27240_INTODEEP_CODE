package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Tele extends OpMode {
    // up here define motors
    public DcMotor UpperLeftDrive = null;
    public DcMotor UpperRightDrive = null;
    public DcMotor LowerLeftDrive = null;
    public DcMotor LowerRightDrive = null;

    public static final double Servo_1 =  1 ;
    public static final double Arm_Speed = 0.8 ;
    public static final double Arm_GoingUp_Power    =  0.50 ;
    public static final double Arm_GoingDown_Power  = -0.50 ;
    // Before code runs when initialized
    @Override
    public void init() {

        // hardware map motors here
        LowerRightDrive  = hardwareMap.get(DcMotor.class, "Lower_right_drive");
        UpperRightDrive = hardwareMap.get(DcMotor.class, "Upper_right_drive");
        LowerLeftDrive  = hardwareMap.get(DcMotor.class, "Lower_left_drive");
        UpperLeftDrive = hardwareMap.get(DcMotor.class, "Upper_left_drive");
        UpperLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        LowerLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    // code runs
    @Override
    public void loop() {

        double forwardPower = gamepad1.left_stick_y;
        double strafePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;


        UpperLeftDrive.setPower(forwardPower+strafePower+turnPower);
        UpperRightDrive.setPower(forwardPower-strafePower-turnPower);
        LowerLeftDrive.setPower(forwardPower-strafePower+turnPower);
        LowerRightDrive.setPower(forwardPower+strafePower-turnPower);

    }
}
