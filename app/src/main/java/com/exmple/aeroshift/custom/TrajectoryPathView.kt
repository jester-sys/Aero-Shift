package com.exmple.aeroshift.custom

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import androidx.core.content.ContextCompat
import com.exmple.aeroshift.R

class TrajectoryPathView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
):View(context,attrs,defStyleAttr) {

    private var pathColor: Int
    private var pathStrokeWidth: Float

    private val pathPaint: Paint
    private val path = Path()


    var startX = 0f
        private set
    var startY = 0f
        private set
    var endX = 0f
        private set
    var endY =0f
        private set

    init {
        // Read custom attributes if provided
        context.theme.obtainStyledAttributes(
            attrs,
            R.styleable.TrajectoryPathView,
            0, 0
        ).apply {
            try {
                pathColor = getColor(
                    R.styleable.TrajectoryPathView_pathColor,
                    ContextCompat.getColor(context, R.color.cyan_400_alpha)
                )
                pathStrokeWidth = getDimension(
                    R.styleable.TrajectoryPathView_pathStrokeWidth,
                    4f
                )
            }finally {

                recycle()
            }
        }

        pathPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth  = pathStrokeWidth
            color  =pathColor
            pathEffect = DashPathEffect(floatArrayOf(10f,20f),0f)
        }

    }


    fun setTrajectory(fromX: Float, fromY: Float, toX: Float, toY: Float) {
        startX = fromX
        startY = fromY
        endX = toX
        endY = toY
        updatePath()
        invalidate()
    }


    fun clearTrajectory() {
        path.reset()
        invalidate()
    }

    private fun updatePath() {
        path.reset()

        if (startX == 0f && startY == 0f && endX == 0f && endY == 0f) {
            return
        }

        val controlX = (startX + endX) / 2 + (endX - startX) * 02f
        val controlY = (startY + endY) / 2

        path.moveTo(startX, startY)
        path.quadTo(controlX, controlY, endX, endY)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawPath(path, pathPaint)
    }

    fun setPathColor(color: Int){
        pathColor = color
        pathPaint.color = color
        invalidate()
    }
    fun setPathStrokeWidth(width: Float) {
        pathStrokeWidth = width
        pathPaint.strokeWidth = width
        invalidate()
    }
    @SuppressLint("Recycle")
    fun animatePathDrawing(durationMs: Long, onComplete:(()-> Unit)? = null){
        val pathMeasure = PathMeasure(path,false)
        val pathLength = pathMeasure.length

        val animator = android.animation.ValueAnimator.ofFloat(0f,pathLength).apply {
            duration = durationMs
            interpolator = android.view.animation.AccelerateDecelerateInterpolator()
            addUpdateListener { animation ->
                val distance = animation.animatedValue as Float
                val effect = DashPathEffect(
                    floatArrayOf(pathLength,pathLength),
                    distance
                )
                pathPaint.pathEffect = effect
                invalidate()
        }
            addListener(object : android.animation.AnimatorListenerAdapter() {
                override fun onAnimationEnd(animation: android.animation.Animator) {

                    pathPaint.pathEffect = DashPathEffect(floatArrayOf(20f, 15f), 0f)
                    invalidate()
                    onComplete?.invoke()
                }
            })
        }
        animator.start()
    }

}
