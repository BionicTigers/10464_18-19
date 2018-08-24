package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.CruiseControlTest;

@TeleOp(name="Position", group ="summerProjects")
//This class is designed to do some testing with the boch IMU chip onboard the REV
public class PositionTracking extends LinearOpMode {
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        //Bellow is an overly complex way of going forward a bit :)
        int rotations = randomWithRange(3,7);
        CruiseControlTest test = new CruiseControlTest();
        test.setCruiseControl(50,rotations);

    }
   public int randomWithRange(int min, int max)
    {
        int range = (max - min) + 1;
        return (int)(Math.random() * range) + min;
    }



}

