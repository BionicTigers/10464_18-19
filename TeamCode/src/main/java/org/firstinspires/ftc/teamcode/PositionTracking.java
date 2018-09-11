package org.firstinspires.ftc.teamcode;
import android.support.annotation.NonNull;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.*;


import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//This is a modification of the random thing tried to replace their proprietary methods with actual stuff but idk how well that worked
@Autonomous(name="Test: REV Expansion Hub", group="hi")

public class PositionTracking extends LinearOpMode
{





    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode()
    {
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = imu.startAccelerationIntegration(Position, Velocity, int);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getPosition();
        imu.initialize(parameters);


        while (true ){
            telemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                   imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
                   imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                   imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("Velocity", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getVelocity().xVeloc,
                    imu.getVelocity().yVeloc,
                    imu.getVelocity().zVeloc);
            telemetry.addData("LinearAccel", "Dist: x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getLinearAcceleration().xAccel,
                    imu.getLinearAcceleration().yAccel,
                    imu.getLinearAcceleration().zAccel);
            telemetry.update();
        }


    }

}



