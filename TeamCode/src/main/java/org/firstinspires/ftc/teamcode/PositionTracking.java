package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.CruiseControlTest;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name="POS", group ="summerProjects")
//This class is designed to do some testing with the boch IMU chip onboard the REV
public class PositionTracking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Bellow is an overly complex way of going forward a bit :)
        int rotations = randomWithRange(3,7);
        CruiseControlTest test = new CruiseControlTest();
        test.setCruiseControl(50,rotations);
        //Next we will try and use the imu chip to guess how far we went.
        //**note** needs testing on venux or mercury not board
        telemetry.addData("hi",rotations);
        telemetry.update();
    }
   public int randomWithRange(int min, int max)
    {
        int range = (max - min) + 1;
        return (int)(Math.random() * range) + min;
    }



}

