package org.firstinspires.ftc.teamcode;
import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//This is a modification of the random thing tried to replace their proprietary methods with actual stuff but idk how well that worked
@TeleOp(name="Test: REV Expansion Hub", group="3543TestSamples")
//@Disabled
public class PositionTracking extends OpMode
{
    private BNO055IMU imu;
    private float hsvValues[] = {0.0f, 0.0f, 0.0f};

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void init()
    {
        //
        // Initializing sensors on or connected to the REV hub.
        //

        imu = new BNO055IMU() {
            @Override
            public boolean initialize(@NonNull Parameters parameters) {
                return false;
            }

            @NonNull
            @Override
            public Parameters getParameters() {
                return null;
            }

            @Override
            public void close() {

            }

            @Override
            public Orientation getAngularOrientation() {
                return null;
            }

            @Override
            public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
                return null;
            }

            @Override
            public Acceleration getOverallAcceleration() {
                return null;
            }

            @Override
            public AngularVelocity getAngularVelocity() {
                return null;
            }

            @Override
            public Acceleration getLinearAcceleration() {
                return null;
            }

            @Override
            public Acceleration getGravity() {
                return null;
            }

            @Override
            public Temperature getTemperature() {
                return null;
            }

            @Override
            public MagneticFlux getMagneticFieldStrength() {
                return null;
            }

            @Override
            public Quaternion getQuaternionOrientation() {
                return null;
            }

            @Override
            public Position getPosition() {
                return null;
            }

            @Override
            public Velocity getVelocity() {
                return null;
            }

            @Override
            public Acceleration getAcceleration() {
                return null;
            }

            @Override
            public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {

            }

            @Override
            public void stopAccelerationIntegration() {

            }

            @Override
            public SystemStatus getSystemStatus() {
                return null;
            }

            @Override
            public SystemError getSystemError() {
                return null;
            }

            @Override
            public CalibrationStatus getCalibrationStatus() {
                return null;
            }

            @Override
            public boolean isSystemCalibrated() {
                return false;
            }

            @Override
            public boolean isGyroCalibrated() {
                return false;
            }

            @Override
            public boolean isAccelerometerCalibrated() {
                return false;
            }

            @Override
            public boolean isMagnetometerCalibrated() {
                return false;
            }

            @Override
            public CalibrationData readCalibrationData() {
                return null;
            }

            @Override
            public void writeCalibrationData(CalibrationData data) {

            }

            @Override
            public byte read8(Register register) {
                return 0;
            }

            @Override
            public byte[] read(Register register, int cb) {
                return new byte[0];
            }

            @Override
            public void write8(Register register, int bVal) {

            }

            @Override
            public void write(Register register, byte[] data) {

            }
        };


    }   //initRobot


    public void loop()
    {
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


    }   //runPeriodic

}   //class FtcTestRevHub
