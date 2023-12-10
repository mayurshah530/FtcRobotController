package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessorV0;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;

@Autonomous()
public class FirstVisionOpmode extends OpMode {
    private FirstVisionProcessorV0 visionProcessor;
    private VisionPortal visionPortal;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        visionProcessor = new FirstVisionProcessorV0();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);

//        setManualExposure(20, 60);
    }

    public void start(){
        visionPortal.stopStreaming();
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getTelemetry());
    }

    /*
 Manually set the camera gain and exposure.
 This can only be called AFTER calling initAprilTag(), and only works for Webcams;
*/
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        try {

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);
        }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

}
