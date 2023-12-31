package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Propecessor implements VisionProcessor {

    private final Telemetry telemetry;
    Rect left;
    Rect center;
    public boolean isRed = true;
    public double detectThreshold = 50;
    public double unsavoryChannelFactor = 1;

    public enum Spike {
        LEFT, CENTER, RIGHT
    }

    public Propecessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        left = new Rect(50, 40, 150, 160);
        center = new Rect(420, 5, 105, 112);
    }

    private double processSide(Mat subframe) {
        List<Mat> channels = new ArrayList<>();
        Core.split(subframe, channels);
        // put coi at first position
        if (!isRed) {
            channels.add(0, channels.get(2));
        }
        channels.get(1).convertTo(channels.get(1), -1, unsavoryChannelFactor);
        channels.get(2).convertTo(channels.get(2), -1, unsavoryChannelFactor);
        Core.subtract(channels.get(0), channels.get(1), channels.get(0));
        Core.subtract(channels.get(0), channels.get(2), channels.get(0));
        return Core.mean(channels.get(0)).val[0];
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double avgLeft = processSide(frame.submat(left).clone());
        double avgCenter = processSide(frame.submat(center).clone());
        telemetry.addData("left", avgLeft);
        telemetry.addData("center", avgCenter);
        Spike select;
        if (avgLeft > detectThreshold) {
            if (avgCenter > detectThreshold) {
                telemetry.addLine("BAD BAD: BOTH SELECTS OVER THRESHOLD, ASSUME CENTER");
                select = Spike.CENTER;
            } else {
                select = Spike.LEFT;
            }
        } else {
            select = Spike.RIGHT;
        }
        telemetry.addData("spike", select);
        telemetry.update();
        return select;
    }

    private Rect scaleRect(Rect r, float f) {
        return new Rect((int) (r.x * f), (int) (r.y * f), (int) (r.width * f), (int) (r.height * f));
    }

    private android.graphics.Rect convertRects(Rect rect) {
        return new android.graphics.Rect(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(10);
        canvas.drawRect(convertRects(scaleRect(left, scaleBmpPxToCanvasPx)), paint);
        canvas.drawRect(convertRects(scaleRect(center, scaleBmpPxToCanvasPx)), paint);
    }
}
