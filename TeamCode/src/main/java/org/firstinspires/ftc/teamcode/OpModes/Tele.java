package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Tele extends OpMode {
    // up here defines motors
    public DcMotor UpperLeftDrive = null;
    public DcMotor UpperRightDrive = null;
    public DcMotor LowerLeftDrive = null;
    public DcMotor LowerRightDrive = null;
    public DcMotor ArmAngleControl = null;
    public DcMotor ArmExtension = null;
    public CRServo Intake = null;

    //Sets speeds to the servo, arm, and motors
    public static final double Arm_Speed = 0.1;
    public static final double Arm_GoingUp_Power = 0.1;
    public static final double Arm_GoingDown_Power = -0.1;
    public static final double Motor_Speed = 0.5;
    public static final double Arm_Extension_Speed = 0.25;
    public static final double Servo_Speed = 0.5;
    // Before code runs when initialized
    @Override
    public void init() {

        // hardware map motors and servos here
        LowerRightDrive = hardwareMap.get(DcMotor.class, "Lower_right_drive");
        UpperRightDrive = hardwareMap.get(DcMotor.class, "Upper_right_drive");
        LowerLeftDrive = hardwareMap.get(DcMotor.class, "Lower_left_drive");
        UpperLeftDrive = hardwareMap.get(DcMotor.class, "Upper_left_drive");
        ArmAngleControl = hardwareMap.get(DcMotor.class, "ArmAngle");
        ArmExtension = hardwareMap.get(DcMotor.class, "ArmExtension");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        UpperLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        LowerLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        ArmAngleControl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmAngleControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmAngleControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // code runs
    @Override
    public void loop(){
        double forwardPower = gamepad1.left_stick_y;
        double strafePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;
        int extendArmState = 6;
        telemetry.addData("Does this read 6? =>", extendArmState);
        //Enabling the controller to move the arm up and down
        if (extendArmState > 5) {
            if (gamepad2.right_stick_y > 0.2){
                double armmovingangle = 0.4;
                ArmAngleControl.setPower(armmovingangle);
            }else if (gamepad2.right_stick_y < -0.2 ) {
                double armmovingangle = -0.4;
                ArmAngleControl.setPower(armmovingangle);
            }  else{
                int armmovingangle = 0;
                ArmAngleControl.setPower(armmovingangle);
            }
            //Code for extending arm
            if(gamepad2.left_stick_y > 0.9){
                double armextension = 1;
                ArmExtension.setPower(armextension);
            } else if (gamepad2.left_stick_y < -0.1) {
                double armextension = -1;
                ArmExtension.setPower(armextension);
            }else {
                int armextesion = 0;
                ArmExtension.setPower(armextesion);
            }
            //Intake code
            if (gamepad2.right_trigger > 0.1) {
                telemetry.addLine("intake");
                int servo = -1;
                Intake.setPower(servo);
            } else if (gamepad2.left_trigger > 0.1) {
                int servo = 5;
                Intake.setPower(servo);
            }
            else {
                int servo = 0;
                Intake.setPower(servo);
            }
        }
        //DO NOT TOUCH THIS CODE BELOW ME!!
        /*if (!extendArmState) {

            if (gamepad2.a) {
                ArmAngleControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmAngleControl.setTargetPosition(scoringPostition);
            }
            if (gamepad2.b) {
                ArmAngleControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmAngleControl.setTargetPosition(backToNormal);

            }
            if (gamepad2.x) {
                ArmExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmExtension.setTargetPosition(alsoAScoringPostion);
            }
            if (gamepad2.x) {
                ArmExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmExtension.setTargetPosition(alsoBackToNormal);
            }
            if (gamepad2.dpad_down) {
                ArmExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmExtension.setTargetPosition(endGameTime);
            }
            if (gamepad2.dpad_up) {
                ArmAngleControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmAngleControl.setTargetPosition(timeToEndThis);
            }

            if (gamepad2.right_trigger > 0.4) {
                int servo = 1;
                Intake.setPower(servo);
            } else if (gamepad2.left_trigger > .4) {
                int servo = -1;
                Intake.setPower(servo);
            }
            else {
                int servo = 0;
                Intake.setPower(servo);
            }
        }
        if (gamepad2.right_trigger > 0.4) {
            int servo = 1;
            Intake.setPower(servo);
        } else if (gamepad2.left_trigger > .4) {
            int servo = -1;
            Intake.setPower(servo);
        }
        else {
            int servo = 0;
            Intake.setPower(servo);
        }*/


        //Allowing the motors to drive/strafe/turn
        UpperLeftDrive.setPower(forwardPower - strafePower + turnPower);
        UpperRightDrive.setPower(forwardPower + strafePower - turnPower);
        LowerLeftDrive.setPower(forwardPower + strafePower + turnPower);
        LowerRightDrive.setPower(forwardPower - strafePower - turnPower);


    }
}
