package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name= "Red Front", group="Red" )
public class Vex_autonomous extends LinearOpMode {

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor launcher;
    public DcMotor transport;
    public DcMotor collector;

   public void runOpMode(){

       motorFrontRight = hardwareMap.dcMotor.get("frontRight");
       motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
       motorBackRight = hardwareMap.dcMotor.get("backRight");
       motorBackLeft = hardwareMap.dcMotor.get("backLeft");
       launcher = hardwareMap.dcMotor.get("launcher");
       transport = hardwareMap.dcMotor.get("transport"); //
       collector = hardwareMap.dcMotor.get("collector");





   }

}
