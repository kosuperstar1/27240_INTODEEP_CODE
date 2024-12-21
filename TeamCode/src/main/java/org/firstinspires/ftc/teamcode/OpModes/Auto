package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Start of the Autonomous
@Autonomous
public class Auto extends LinearOpMode {
    public DcMotor UpperRightDrive = null;
    public DcMotor UpperLeftDrive = null;
    public DcMotor LowerRightDrive = null;
    public DcMotor LowerLeftDrive = null;
    public ElapsedTime timer = null;
    @Override
    public void runOpMode(){
        // hardware map of all of  the motors and servos
        LowerRightDrive = hardwareMap.get(DcMotor.class, "Lower_right_drive");
        UpperRightDrive = hardwareMap.get(DcMotor.class, "Upper_right_drive");
        LowerLeftDrive = hardwareMap.get(DcMotor.class, "Lower_left_drive");
        UpperLeftDrive = hardwareMap.get(DcMotor.class, "Upper_left_drive");
        UpperLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        LowerLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //waiting for the start
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        //if the opMode is working, make the robot perform autonomous
        if(opModeIsActive()){
            timer.reset();
            //The actual auto code
            while (timer.seconds() < 10.0) {
                //strafe right
                UpperRightDrive.setPower(-1);
                UpperLeftDrive.setPower(-1);
                LowerLeftDrive.setPower(1);
                LowerRightDrive.setPower(1);
            }

        }
    }
}
