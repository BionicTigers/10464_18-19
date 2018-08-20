package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Vuforia Distance Tracker", group ="summerProjects")
public class VuforiaDistance extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables flag_finder = this.vuforia.loadTrackablesFromAsset("flag_finder");
        VuforiaTrackable trendy  = flag_finder.get(0);
        trendy.setName("Trendy");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.add(flag_finder.get(0));

        telemetry.addData(">", "Press Play to see if this bad boy works...");
        telemetry.update();
        waitForStart();

       //activate vuforia
        flag_finder.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix testLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                String pose = "NULL";
                String distance = "NULL";
                String launchVelocity = "NULL";
                String cameraCenter = "NULL";
                String cameraRotationAlias = "NULL";
                if(testLocation != null) {
                    pose = format(testLocation);
                    distance = "" + getDistance(testLocation);
                    launchVelocity = "" + launchVelocity(getDistance(testLocation),30);
                    cameraCenter = "" + cameraRotation(testLocation);
                    cameraRotationAlias = "" + (cameraRotation(testLocation) < 0 ? "TURNING RIGHT" : "TURINING LEFT");
                }
                telemetry.addData("POSE", pose);
                telemetry.addData("DISTANCE", distance);
                telemetry.addData("VELOCITY", launchVelocity);
                telemetry.addData("CAMERA_MOTOR_POWER", cameraCenter);
                telemetry.addData("CAMERA_SERVO_ALIAS", cameraRotationAlias);


            }
            telemetry.update();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    /**
     * A utility that takes an OPENGLMatrix transform and determines the distance to the origin (0,0,0).
     */
    float getDistance(OpenGLMatrix transformatoinMatrix) {
        float x = transformatoinMatrix.getTranslation().get(0);
        float y = transformatoinMatrix.getTranslation().get(1);
        float z = transformatoinMatrix.getTranslation().get(2);
        return (float)Math.sqrt(Math.pow(Math.sqrt(Math.pow(x,2)+Math.pow(y,2)),2)+Math.pow(z,2)); //use pythagorean formula twice to find distance
    }

    /**
     *
     * @param distance - distance to the target, in meters
     * @param angle - angle the object is launched at, in degrees
     * @return velocity of launch, in meters per second (assuming object is level with camera)
     */
    float launchVelocity(float distance, float angle) {
        return (float)Math.sqrt((9.8*distance)/Math.sin(Math.toRadians(angle)));
    }

    float cameraRotation(OpenGLMatrix transformationMatrix) {
        float y = transformationMatrix.getTranslation().get(1);
        return 0.1f * y;
    }

}