package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

public class TeamPropProcessor implements VisionProcessor {
    public final int RECT_WIDTH = 60;
    public final int RECT_HEIGHT = 60;
    
    public int alliance = -1;       // -1: none, 0: blue, 1: red
    public int aBase = -1;
    public int propPos = -1;
    public double satMax = 0;

    public Rect[] rectList = null;
    Mat subMat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        rectList = new Rect[6];
        rectList[0] = new Rect(160, 220, RECT_WIDTH, RECT_HEIGHT); // blue left
        rectList[1] = new Rect(365, 220, RECT_WIDTH, RECT_HEIGHT); // blue center
        rectList[2] = new Rect(580, 220, RECT_WIDTH, RECT_HEIGHT); // blue right
        rectList[3] = new Rect( 10, 230, RECT_WIDTH, RECT_HEIGHT); // red left;
        rectList[4] = new Rect(230, 230, RECT_WIDTH, RECT_HEIGHT); // red center;
        rectList[5] = new Rect(420, 230, RECT_WIDTH, RECT_HEIGHT); // red right;
        alliance = -1;
        aBase = -1;
        propPos = -1;
    }
    
    public void setAlliance(int a, int b) {
        alliance = a;
        aBase = b;
        //propPos = -1;
    }
    
    public int getPropPos() { return propPos; }
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (aBase < 0) return this;
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        satMax = 0;
        for (int i = 0; i < 3; i++) {
            double s = getAverageSaturation(hsvMat, rectList[i+aBase]);
            if (s > satMax) { satMax = s; propPos = i+aBase; }
        }
        return this;
    }
    
    public double getAverageSaturation(Mat input, Rect rect) {
        subMat = input.submat(rect);
        Scalar color = Core.mean(subMat);
        return color.val[1];
    }
    
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmp) {
        int left = Math.round(rect.x * scaleBmp);
        int top = Math.round(rect.y * scaleBmp);
        int right = left + Math.round(rect.width * scaleBmp);
        int bottom = top + Math.round(rect.height * scaleBmp);
        return new android.graphics.Rect(left, top, right, bottom);
    }
    
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onScreenHeight,
                            float scaleBmp, float scaleCanvas, Object userContext) {
        if (aBase < 0) return;
        Paint paint = new Paint();
        int baseColor = (aBase == 0) ? Color.BLUE : Color.RED;
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvas * 4);
        for (int i = 0; i < 3; i++) {
            android.graphics.Rect dRect = makeGraphicsRect(rectList[i+aBase], scaleBmp);
            paint.setColor( (i+aBase) == propPos ? Color.GREEN : baseColor );
            canvas.drawRect(dRect, paint);
        }
    }                            
}
