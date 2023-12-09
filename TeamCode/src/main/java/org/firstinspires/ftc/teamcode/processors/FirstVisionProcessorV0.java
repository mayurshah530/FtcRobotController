package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.ScoringElementLocation;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class FirstVisionProcessorV0 implements VisionProcessor {
    ScoringElementLocation selection = ScoringElementLocation.UNKNOWN;

    // Define lower and upper HSV values for the red and blue color ranges
    public Scalar lowerRed = new Scalar(0, 100, 100);
    public Scalar upperRed = new Scalar(10, 255, 255);
    public Scalar lowerBlue = new Scalar(100, 100, 100);
    public Scalar upperBlue = new Scalar(140, 255, 255);

    private Mat hsvImage;
    private Mat mask;


    int width = 640;
    int height = 480;

    // Divide the mask into three vertical regions
    Rect leftRegion = new Rect(0, 0, width / 5, height);
    Rect centerRegion = new Rect(width / 5, 0, 3 * (width / 5), height);
    Rect rightRegion = new Rect(4 * (width / 5), 0, width / 5, height);

    String telemetry = "";

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        telemetry = "";
        // Convert the image to HSV format
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Create a mask to extract red-colored pixels
//        Mat redMask = new Mat();
        Core.inRange(hsvImage, lowerBlue, upperBlue, frame);

//        Scalar lowerRed2 = new Scalar(150, 100, 100);
//        Scalar upperRed2 = new Scalar(180, 255, 255);
//        // Create a mask to extract red-colored pixels
//        Mat redMask2 = new Mat();
//        Core.inRange(hsvImage, lowerRed2, upperRed2, redMask2);
//        Core.add(redMask, redMask2, redMask);

        // Get the dimensions of the image
        int height = frame.rows();
        int width = frame.cols();


        Mat leftMask = new Mat(frame, leftRegion);
        Mat centerMask = new Mat(frame, centerRegion);
        Mat rightMask = new Mat(frame, rightRegion);

        // Count the red pixels in each region
        int leftCount = Core.countNonZero(leftMask);
        int centerCount = Core.countNonZero(centerMask);
        int rightCount = Core.countNonZero(rightMask);

        telemetry += "leftCount = " + leftCount;
        telemetry += " centerCount =" + centerCount;
        telemetry += " rightCount = " + rightCount;

        if (leftCount > centerCount && leftCount > rightCount){
            selection =  ScoringElementLocation.LEFT;
        } else if (centerCount > leftCount && centerCount > rightCount){
            selection =  ScoringElementLocation.CENTER;
        } else {
            selection = ScoringElementLocation.RIGHT;
        }
        telemetry += " Selected " + selection.toString();

//        frame = redMask.clone();
//        redMask.CopyTo(frame);
//        Core.bitwise_and(frame, frame, frame, redMask);

        return telemetry;
    }



    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);
        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(leftRegion, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(centerRegion, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rightRegion, scaleBmpPxToCanvasPx);
        canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
        canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
        canvas.drawRect(drawRectangleRight, nonSelectedPaint);

    }

    public ScoringElementLocation getSelection() { return selection;
    }

    public void SetAlliance(Alliance alliance){
        _alliance = alliance;
    }

    public String getTelemetry(){ return telemetry;}
    private Alliance _alliance = Alliance.RED;


    private android.graphics.Rect makeGraphicsRect(Rect rect, float  scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx); int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom); 
    }

}
