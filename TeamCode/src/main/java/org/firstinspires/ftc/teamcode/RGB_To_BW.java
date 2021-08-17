package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="RGB To BW", group="linearOpMode")
public class RGB_To_BW extends AutonomousPrime2020 {
    Mat grey = new Mat();

    @Override
    public void runOpMode() {
        mapObjects();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDevice();
        camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);

        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
                return grey;
            }
        });

        waitForStart();

        if (opModeIsActive()) {

        }
    }
}