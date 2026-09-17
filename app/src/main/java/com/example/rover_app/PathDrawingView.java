package com.example.rover_app;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;

/**
 * Custom interactive touch canvas for drawing a rover navigation path.
 * Converts freehand touch gestures into Arduino differential-drive commands
 * formatted as: F:<time>,R:<time>,F:<time>,L:<time>,S
 */
public class PathDrawingView extends View {

    private final Paint pathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint endPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Cached drawing dimensions for 60fps rendering without repeated dpToPx conversions
    private float cachedGridSize = 0f;
    private float cachedMarkerRadius = 0f;
    private float cachedLabelOffset = 0f;
    private float cachedMinMoveDist = 0f;

    private final Path drawPath = new Path();
    private final List<PointF> rawPoints = new ArrayList<>();

    private PathListener listener;

    public interface PathListener {
        void onPathDrawn(String generatedCommand);
        void onPathCleared();
    }

    public PathDrawingView(Context context) {
        super(context);
        init();
    }

    public PathDrawingView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public PathDrawingView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        // Pre-compute density dimensions
        cachedGridSize = dpToPx(32);
        cachedMarkerRadius = dpToPx(10);
        cachedLabelOffset = cachedMarkerRadius + dpToPx(4);
        cachedMinMoveDist = dpToPx(8);

        // Path paint (Electric Tech Blue)
        pathPaint.setColor(Color.parseColor("#2563EB"));
        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setStrokeWidth(dpToPx(4.5f));
        pathPaint.setStrokeCap(Paint.Cap.ROUND);
        pathPaint.setStrokeJoin(Paint.Join.ROUND);

        // Grid lines (Crisp Blueprint Slate Grid)
        gridPaint.setColor(Color.parseColor("#CBD5E1"));
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(dpToPx(1f));
        gridPaint.setPathEffect(new DashPathEffect(new float[]{dpToPx(4), dpToPx(4)}, 0));

        // Start point (Emerald Green)
        startPaint.setColor(Color.parseColor("#10B981"));
        startPaint.setStyle(Paint.Style.FILL);

        // End point (Crimson Red)
        endPaint.setColor(Color.parseColor("#EF4444"));
        endPaint.setStyle(Paint.Style.FILL);

        // Text labels (Deep Slate)
        textPaint.setColor(Color.parseColor("#0F172A"));
        textPaint.setTextSize(dpToPx(11f));
        textPaint.setFakeBoldText(true);
        textPaint.setTextAlign(Paint.Align.CENTER);
    }

    public void setPathListener(PathListener listener) {
        this.listener = listener;
    }

    public void clearPath() {
        rawPoints.clear();
        drawPath.reset();
        invalidate();
        if (listener != null) {
            listener.onPathCleared();
        }
    }

    public boolean hasPath() {
        return rawPoints.size() >= 2;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();

        // Draw background grid lines using cached step
        for (float x = cachedGridSize; x < width; x += cachedGridSize) {
            canvas.drawLine(x, 0, x, height, gridPaint);
        }
        for (float y = cachedGridSize; y < height; y += cachedGridSize) {
            canvas.drawLine(0, y, width, y, gridPaint);
        }

        // Draw user drawn path
        canvas.drawPath(drawPath, pathPaint);

        // Draw start and end markers
        if (!rawPoints.isEmpty()) {
            PointF start = rawPoints.get(0);
            canvas.drawCircle(start.x, start.y, cachedMarkerRadius, startPaint);
            canvas.drawText("START", start.x, start.y - cachedLabelOffset, textPaint);

            if (rawPoints.size() > 1) {
                PointF end = rawPoints.get(rawPoints.size() - 1);
                canvas.drawCircle(end.x, end.y, cachedMarkerRadius, endPaint);
                canvas.drawText("END", end.x, end.y - cachedLabelOffset, textPaint);
            }
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float x = event.getX();
        float y = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                rawPoints.clear();
                drawPath.reset();
                drawPath.moveTo(x, y);
                rawPoints.add(new PointF(x, y));
                invalidate();
                return true;

            case MotionEvent.ACTION_MOVE:
                if (!rawPoints.isEmpty()) {
                    PointF last = rawPoints.get(rawPoints.size() - 1);
                    float dist = (float) Math.hypot(x - last.x, y - last.y);
                    if (dist >= cachedMinMoveDist) {
                        drawPath.lineTo(x, y);
                        rawPoints.add(new PointF(x, y));
                        invalidate();
                    }
                }
                return true;

            case MotionEvent.ACTION_UP:
                if (!rawPoints.isEmpty()) {
                    drawPath.lineTo(x, y);
                    rawPoints.add(new PointF(x, y));
                    invalidate();
                    if (listener != null) {
                        listener.onPathDrawn(generateRoverCommand());
                    }
                }
                return true;
        }

        return super.onTouchEvent(event);
    }

    /**
     * Converts the drawn 2D pixel trajectory into Arduino rover movement commands:
     * Sequential steps: F:<ms>, R:<ms>, L:<ms>, ... ending with S.
     */
    public String generateRoverCommand() {
        if (rawPoints.size() < 2) {
            return "";
        }

        // 1. Resample points with step distance of ~45dp
        float stepDist = dpToPx(45);
        List<PointF> simplified = new ArrayList<>();
        simplified.add(rawPoints.get(0));

        PointF prev = rawPoints.get(0);
        for (int i = 1; i < rawPoints.size(); i++) {
            PointF cur = rawPoints.get(i);
            float d = (float) Math.hypot(cur.x - prev.x, cur.y - prev.y);
            if (d >= stepDist || i == rawPoints.size() - 1) {
                simplified.add(cur);
                prev = cur;
            }
        }

        if (simplified.size() < 2) {
            simplified.add(rawPoints.get(rawPoints.size() - 1));
        }

        // Helper step class
        class Step {
            char dir;
            long duration;
            Step(char dir, long duration) { this.dir = dir; this.duration = duration; }
        }
        List<Step> stepList = new ArrayList<>();

        // 2. Initial Forward Movement towards first sampled point
        PointF p0 = simplified.get(0);
        PointF p1 = simplified.get(1);

        double currentHeading = Math.toDegrees(Math.atan2(p1.y - p0.y, p1.x - p0.x));
        float d0 = (float) Math.hypot(p1.x - p0.x, p1.y - p0.y);
        long t0 = calculateForwardTime(d0);
        stepList.add(new Step('F', t0));

        // 3. Process subsequent segments: Turn if needed, then Move Forward
        for (int i = 1; i < simplified.size() - 1; i++) {
            PointF from = simplified.get(i);
            PointF to = simplified.get(i + 1);

            double targetHeading = Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
            double deltaAngle = targetHeading - currentHeading;

            // Normalize deltaAngle to [-180, +180]
            while (deltaAngle > 180) deltaAngle -= 360;
            while (deltaAngle < -180) deltaAngle += 360;

            // Significant turn threshold: 22 degrees
            if (Math.abs(deltaAngle) >= 22) {
                long turnMs = Math.round(Math.abs(deltaAngle) * (600.0 / 90.0));
                turnMs = Math.max(250, Math.min(1200, (turnMs / 50) * 50));
                stepList.add(new Step(deltaAngle > 0 ? 'R' : 'L', turnMs));
                currentHeading = targetHeading;
            }

            float segmentDist = (float) Math.hypot(to.x - from.x, to.y - from.y);
            if (segmentDist >= dpToPx(12)) {
                long forwardMs = calculateForwardTime(segmentDist);
                // Combine consecutive forward segments
                if (!stepList.isEmpty() && stepList.get(stepList.size() - 1).dir == 'F') {
                    stepList.get(stepList.size() - 1).duration += forwardMs;
                } else {
                    stepList.add(new Step('F', forwardMs));
                }
            }
        }

        // Build formatted string
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < stepList.size(); i++) {
            Step s = stepList.get(i);
            if (i > 0) sb.append(",");
            sb.append(s.dir).append(":").append(s.duration);
        }

        // 4. Always terminate path with Stop command (S)
        if (sb.length() > 0) {
            sb.append(",S");
        } else {
            sb.append("S");
        }

        return sb.toString();
    }

    private long calculateForwardTime(float pixelDist) {
        // ~150px corresponds to ~800ms of forward motion at normal speed
        float msPerPixel = 5.5f;
        long time = Math.round(pixelDist * msPerPixel);
        // Clamp between 400ms and 2500ms, round to nearest 50ms
        time = Math.max(400, Math.min(2500, (time / 50) * 50));
        return time;
    }

    private float dpToPx(float dp) {
        return dp * getResources().getDisplayMetrics().density;
    }
}
