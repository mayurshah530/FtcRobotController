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

public class FirstVisionProcessor implements VisionProcessor {
    ScoringElementLocation selection = ScoringElementLocation.UNKNOWN;

    // Define lower and upper HSV values for the red and blue color ranges
    public Scalar lowerRed = new Scalar(0, 100, 100);
    public Scalar upperRed = new Scalar(10, 255, 255);
    public Scalar lowerBlue = new Scalar(100, 100, 100);
    public Scalar upperBlue = new Scalar(140, 255, 255);

    private Mat hsvImage;
    private Mat mask;

    // Divide the mask into three vertical regions
    private Rect leftRegion;
    private Rect centerRegion;
    private Rect rightRegion;

    String telemetry = "";

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        hsvImage = new Mat(height, width, CvType.CV_8UC3);
        mask = new Mat(height, width, CvType.CV_8UC1);

        // Divide the mask into three vertical regions
        leftRegion = new Rect(0, 0, width / 5, height);
        centerRegion = new Rect(width / 5, 0, 3 * (width / 5), height);
        rightRegion = new Rect(4 * (width / 5), 0, width / 5, height);
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Scalar lowerHsv = lowerRed;
        Scalar upperHsv = upperRed;
        switch (_alliance){
            case UNKNOWN:
                return ScoringElementLocation.UNKNOWN;
            case RED:
                lowerHsv = lowerRed;
                upperHsv = upperRed;
                break;
            case BLUE:
                lowerHsv = lowerBlue;
                upperHsv = upperBlue;
                break;
        }

        // Convert the image to HSV format
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Create a mask to extract red-colored pixels
        Core.inRange(hsvImage, lowerHsv, upperHsv, mask);

        Mat leftMask = new Mat(mask, leftRegion);
        Mat centerMask = new Mat(mask, centerRegion);
        Mat rightMask = new Mat(mask, rightRegion);

        // Count the red pixels in each region
        int leftCount = Core.countNonZero(leftMask);
        int centerCount = Core.countNonZero(centerMask);
        int rightCount = Core.countNonZero(rightMask);

        if (leftCount > centerCount && leftCount > rightCount){
            selection =  ScoringElementLocation.LEFT;
        } else if (centerCount > leftCount && centerCount > rightCount){
            selection =  ScoringElementLocation.CENTER;
        } else {
            selection = ScoringElementLocation.RIGHT;
        }

        telemetry += "leftCount = " + leftCount;
        telemetry += " centerCount =" + centerCount;
        telemetry += " rightCount = " + rightCount;
        telemetry += " Selected " + selection.toString();

        return selection;
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


    private android.graphics.Rect makeGraphicsRect(Rect rect, float  scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx); int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    public ScoringElementLocation getSelection() { return selection;
    }

    public void SetAlliance(Alliance alliance){
        _alliance = alliance;
    }

    public String getTelemetry(){ return telemetry;}
    private Alliance _alliance = Alliance.RED;

}
