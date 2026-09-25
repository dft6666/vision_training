#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include<vector>
#include<cmath>
#include<algorithm>
#include<limits>
#include<fstream>
using namespace cv;
using namespace std;
struct AngleModelParams
{
    double A;
    double b;
    double Omega;
    double phi;
    double theta0;

    double finalCost;
    bool success;
};
struct AngleResidual
{
    AngleResidual(
        double t,
        double thetaObserved)
        : t_(t),
          thetaObserved_(thetaObserved)
    {
    }

    template <typename T>
    bool operator()(
        const T* const params,
        T* residual) const
    {
        const T& A =
            params[0];

        const T& d =
            params[1];

        const T& Omega =
            params[2];

        const T& phi =
            params[3];

        const T& theta0 =
            params[4];
        const T b =
            A + d;

        const T t =
            T(t_);
        const T thetaPredicted =
            theta0
            + b * t
            + (A / Omega)
              * (
                    ceres::cos(phi)
                    -
                    ceres::cos(
                        Omega * t + phi
                    )
                );
        residual[0] =
            T(thetaObserved_)
            - thetaPredicted;

        return true;
    }

    double t_;
    double thetaObserved_;
};
double wrapToPi(double angle)
{
    while (angle >= CV_PI)
    {
        angle -= 2.0 * CV_PI;
    }

    while (angle < -CV_PI)
    {
        angle += 2.0 * CV_PI;
    }

    return angle;
}
bool detectCyanCenter(const Mat& frame,Point2d& center,Mat& mask){
	Mat hsv;
	cvtColor(frame,hsv,COLOR_BGR2HSV);
	inRange(hsv,Scalar(80,100,100),Scalar(100,255,255),mask);
	 Mat kernel = getStructuringElement(
        MORPH_ELLIPSE,
        Size(5, 5)
    );
	 morphologyEx(
        mask,
        mask,
        MORPH_OPEN,
        kernel
    );
vector<vector<Point>> contours;

findContours(
    mask,
    contours,
    RETR_EXTERNAL,
    CHAIN_APPROX_SIMPLE
);
double bestArea = 0.0;
int bestIndex = -1;

for (int i = 0; i < contours.size(); ++i)
{
    double area = contourArea(contours[i]);

    if (area > bestArea)
    {
        bestArea = area;
        bestIndex = i;
    }
}
if (bestIndex == -1 || bestArea < 50.0)
{
    return false;
}
Moments m = moments(contours[bestIndex]);

    if (m.m00 == 0.0)
    {
        return false;
    }

    center.x = m.m10 / m.m00;
    center.y = m.m01 / m.m00;

	return true;
}
double computeWrappedAngle(const Point2d& rotationCenter,const Point2d& targetCenter){
double dx =
        targetCenter.x
        - rotationCenter.x;

    double dy =
        rotationCenter.y
        - targetCenter.y;

    double theta =
        atan2(dy, dx);

    return theta;
}
double unwrapAngle(
    double thetaWrapped,
    bool& hasPreviousAngle,
    double& previousWrapped,
    double& previousUnwrapped)
{
    double thetaUnwrapped;
    if (!hasPreviousAngle)
    {
        thetaUnwrapped =
            thetaWrapped;

        hasPreviousAngle = true;
    }
    else
    {
        double delta =
            thetaWrapped
            - previousWrapped;
        if (delta > CV_PI)
        {
            delta -= 2.0 * CV_PI;
        }
        if (delta < -CV_PI)
        {
            delta += 2.0 * CV_PI;
        }

        thetaUnwrapped =
            previousUnwrapped
            + delta;
    }
    previousWrapped =
        thetaWrapped;
    previousUnwrapped =
        thetaUnwrapped;
    return thetaUnwrapped;
}
void drawTrackingOverlay(
    Mat& frame,
    const Point2d& rotationCenter,
    const Point2d& targetCenter,
    double theta,
    int frameIndex)
{
    circle(
        frame,
        rotationCenter,
        6,
        Scalar(255, 0, 255),
        -1
    );
    circle(
        frame,
        targetCenter,
        6,
        Scalar(0, 0, 255),
        -1
    );
    line(
        frame,
        rotationCenter,
        targetCenter,
        Scalar(0, 255, 0),
        2
    );
    string angleText =
        "theta = "
        + to_string(theta)
        + " rad";
    putText(
        frame,
        angleText,
        Point(30, 40),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        Scalar(0, 255, 0),
        2
    );
    string frameText =
        "frame = "
        + to_string(frameIndex);
    putText(
        frame,
        frameText,
        Point(30, 75),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        Scalar(0, 255, 0),
        2
    );
}
AngleModelParams fitAngleModel(
    const vector<double>& times,
    const vector<double>& angles)
{
    AngleModelParams result{};

    result.success = false;

    if (times.size() < 10 ||
        times.size() != angles.size())
    {
        cerr
            << "Not enough valid angle data."
            << endl;

        return result;
    }
    double totalTime =
        times.back() - times.front();

    double totalAngle =
        angles.back() - angles.front();

    double b0 =
        totalAngle / totalTime;


    if (b0 <= 0.0)
    {
        cerr
            << "Initial b is not positive."
            << endl;

        return result;
}
    double A0 =
        0.2 * b0;

    double d0 =
        b0 - A0;

    double theta00 =
        angles.front();
    vector<double> omegaInitials =
    {
        0.2,
        0.4,
        0.6,
        0.8,
        1.0,
        1.2,
        1.5,
        2.0,
        2.5,
        3.0
    };


    double bestCost =
        numeric_limits<double>::infinity();
    for (double Omega0 : omegaInitials)
    {
        double params[5] =
        {
            A0,
            d0,
            Omega0,
            0.0,
            theta00
        };


        ceres::Problem problem;
        for (size_t i = 0;
             i < times.size();
             ++i)
        {
            auto* costFunction =
                new ceres::AutoDiffCostFunction<
                    AngleResidual,
                    1,
                    5>(
                    new AngleResidual(
                        times[i],
                        angles[i]
                    )
                );


            problem.AddResidualBlock(
                costFunction,
                nullptr,
                params
            );
        }
        problem.SetParameterLowerBound(
            params,
            0,
            1e-6
        );

        problem.SetParameterLowerBound(
            params,
            1,
            1e-6
        );

        problem.SetParameterLowerBound(
            params,
            2,
            1e-6
        );
        ceres::Solver::Options options;

        options.linear_solver_type =
            ceres::DENSE_QR;

        options.max_num_iterations =
            200;

        options.minimizer_progress_to_stdout =
            false;


        ceres::Solver::Summary summary;

        ceres::Solve(
            options,
            &problem,
            &summary
        );


        if (!summary.IsSolutionUsable())
        {
            continue;
        }


        if (summary.final_cost < bestCost)
        {
            bestCost =
                summary.final_cost;

            result.A =
                params[0];

            result.b =
                params[0]
                + params[1];

            result.Omega =
                params[2];

            result.phi =
                wrapToPi(
                    params[3]
                );

            result.theta0 =
                params[4];

            result.finalCost =
                summary.final_cost;

            result.success =
                true;
        }
    }


    return result;
}
double predictAngle(
    double t,
    const AngleModelParams& p)
{
    return
        p.theta0
        + p.b * t
        + (p.A / p.Omega)
          * (
                cos(p.phi)
                -
                cos(
                    p.Omega * t
                    + p.phi
                )
            );
}
vector<double> generateFittedAngles(
    const vector<double>& times,
    const AngleModelParams& p)
{
    vector<double> fitted;

    fitted.reserve(
        times.size()
    );

    for (double t : times)
    {
        fitted.push_back(
            predictAngle(t, p)
        );
    }

    return fitted;
}
vector<double> computeAngleResiduals(
    const vector<double>& observed,
    const vector<double>& fitted)
{
    vector<double> residuals;

    residuals.reserve(
        observed.size()
    );

    for (size_t i = 0;
         i < observed.size();
         ++i)
    {
        residuals.push_back(
            observed[i]
            - fitted[i]
        );
    }

    return residuals;
}
double computeAngleRMSE(
    const vector<double>& residuals)
{
    if (residuals.empty())
    {
        return 0.0;
    }

    double sumSquare = 0.0;

    for (double r : residuals)
    {
        sumSquare +=
            r * r;
    }

    return sqrt(
        sumSquare
        / residuals.size()
    );
}
vector<double> generateFittedAngularVelocity(
    const vector<double>& times,
    const AngleModelParams& p)
{
    vector<double> velocity;

    velocity.reserve(
        times.size()
    );

    for (double t : times)
    {
        double omega =
            p.b
            + p.A
              * sin(
                    p.Omega * t
                    + p.phi
                );

        velocity.push_back(
            omega
        );
    }

    return velocity;
}
Point mapToPlot(
    double x,
    double y,
    double xMin,
    double xMax,
    double yMin,
    double yMax,
    int left,
    int top,
    int plotWidth,
    int plotHeight)
{
    double nx =
        (x - xMin)
        / (xMax - xMin);

    double ny =
        (y - yMin)
        / (yMax - yMin);

    int px =
        left
        + static_cast<int>(
            nx * plotWidth
        );

    int py =
        top
        + plotHeight
        - static_cast<int>(
            ny * plotHeight
        );

    return Point(px, py);
}
void saveFitComparison(
    const vector<double>& times,
    const vector<double>& observed,
    const vector<double>& fitted,
    const string& path)
{
    int width = 1200;
    int height = 700;

    int left = 90;
    int right = 40;
    int top = 40;
    int bottom = 80;

    int plotWidth =
        width - left - right;

    int plotHeight =
        height - top - bottom;


    double xMin =
        times.front();

    double xMax =
        times.back();


    double yMin =
        min(
            *min_element(
                observed.begin(),
                observed.end()
            ),
            *min_element(
                fitted.begin(),
                fitted.end()
            )
        );

    double yMax =
        max(
            *max_element(
                observed.begin(),
                observed.end()
            ),
            *max_element(
                fitted.begin(),
                fitted.end()
            )
        );


    // 避免范围为 0
    if (abs(yMax - yMin) < 1e-9)
    {
        yMax += 1.0;
        yMin -= 1.0;
    }


    Mat plot(
        height,
        width,
        CV_8UC3,
        Scalar(255, 255, 255)
    );


    // X 轴
    line(
        plot,
        Point(left, top + plotHeight),
        Point(left + plotWidth, top + plotHeight),
        Scalar(0, 0, 0),
        2
    );

    // Y 轴
    line(
        plot,
        Point(left, top),
        Point(left, top + plotHeight),
        Scalar(0, 0, 0),
        2
    );


    // 观测点
    for (size_t i = 0;
         i < times.size();
         i += 4)
    {
        Point p =
            mapToPlot(
                times[i],
                observed[i],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        circle(
            plot,
            p,
            2,
            Scalar(0, 0, 255),
            -1
        );
    }


    // 拟合曲线
    for (size_t i = 1;
         i < times.size();
         ++i)
    {
        Point p1 =
            mapToPlot(
                times[i - 1],
                fitted[i - 1],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        Point p2 =
            mapToPlot(
                times[i],
                fitted[i],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        line(
            plot,
            p1,
            p2,
            Scalar(255, 0, 0),
            2
        );
    }


    putText(
        plot,
        "Angle fitting comparison",
        Point(400, 30),
        FONT_HERSHEY_SIMPLEX,
        0.8,
        Scalar(0, 0, 0),
        2
    );

    putText(
        plot,
        "time / s",
        Point(width / 2, height - 25),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        Scalar(0, 0, 0),
        2
    );

    putText(
        plot,
        "theta / rad",
        Point(10, 30),
        FONT_HERSHEY_SIMPLEX,
        0.6,
        Scalar(0, 0, 0),
        2
    );


    imwrite(
        path,
        plot
    );
}
void saveResidualPlot(
    const vector<double>& times,
    const vector<double>& residuals,
    const string& path)
{
    int width = 1200;
    int height = 700;

    int left = 90;
    int right = 40;
    int top = 40;
    int bottom = 80;

    int plotWidth =
        width - left - right;

    int plotHeight =
        height - top - bottom;


    double xMin =
        times.front();

    double xMax =
        times.back();


    double maxAbs = 0.0;

    for (double r : residuals)
    {
        maxAbs =
            max(
                maxAbs,
                abs(r)
            );
    }

    if (maxAbs < 1e-6)
    {
        maxAbs = 1.0;
    }

    double yMin =
        -1.1 * maxAbs;

    double yMax =
        1.1 * maxAbs;


    Mat plot(
        height,
        width,
        CV_8UC3,
        Scalar(255, 255, 255)
    );


    // 坐标轴
    line(
        plot,
        Point(left, top),
        Point(left, top + plotHeight),
        Scalar(0, 0, 0),
        2
    );

    // y = 0
    Point zeroLeft =
        mapToPlot(
            xMin,
            0.0,
            xMin,
            xMax,
            yMin,
            yMax,
            left,
            top,
            plotWidth,
            plotHeight
        );

    Point zeroRight =
        mapToPlot(
            xMax,
            0.0,
            xMin,
            xMax,
            yMin,
            yMax,
            left,
            top,
            plotWidth,
            plotHeight
        );

    line(
        plot,
        zeroLeft,
        zeroRight,
        Scalar(0, 0, 0),
        1
    );


    // residual 曲线
    for (size_t i = 1;
         i < times.size();
         ++i)
    {
        Point p1 =
            mapToPlot(
                times[i - 1],
                residuals[i - 1],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        Point p2 =
            mapToPlot(
                times[i],
                residuals[i],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        line(
            plot,
            p1,
            p2,
            Scalar(255, 0, 0),
            1
        );
    }


    putText(
        plot,
        "Angle residuals",
        Point(450, 30),
        FONT_HERSHEY_SIMPLEX,
        0.8,
        Scalar(0, 0, 0),
        2
    );

    putText(
        plot,
        "time / s",
        Point(width / 2, height - 25),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        Scalar(0, 0, 0),
        2
    );


    imwrite(
        path,
        plot
    );
}
void saveAngularVelocityPlot(
    const vector<double>& times,
    const vector<double>& velocity,
    const string& path)
{
    int width = 1200;
    int height = 700;

    int left = 90;
    int right = 40;
    int top = 40;
    int bottom = 80;

    int plotWidth =
        width - left - right;

    int plotHeight =
        height - top - bottom;


    double xMin =
        times.front();

    double xMax =
        times.back();

    double yMin =
        *min_element(
            velocity.begin(),
            velocity.end()
        );

    double yMax =
        *max_element(
            velocity.begin(),
            velocity.end()
        );


    double margin =
        0.1 * (yMax - yMin);

    if (margin < 1e-6)
    {
        margin = 1.0;
    }

    yMin -= margin;
    yMax += margin;


    Mat plot(
        height,
        width,
        CV_8UC3,
        Scalar(255, 255, 255)
    );


    line(
        plot,
        Point(left, top + plotHeight),
        Point(left + plotWidth, top + plotHeight),
        Scalar(0, 0, 0),
        2
    );

    line(
        plot,
        Point(left, top),
        Point(left, top + plotHeight),
        Scalar(0, 0, 0),
        2
    );


    for (size_t i = 1;
         i < times.size();
         ++i)
    {
        Point p1 =
            mapToPlot(
                times[i - 1],
                velocity[i - 1],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        Point p2 =
            mapToPlot(
                times[i],
                velocity[i],
                xMin,
                xMax,
                yMin,
                yMax,
                left,
                top,
                plotWidth,
                plotHeight
            );

        line(
            plot,
            p1,
            p2,
            Scalar(255, 0, 0),
            2
        );
    }


    putText(
        plot,
        "Estimated angular velocity",
        Point(390, 30),
        FONT_HERSHEY_SIMPLEX,
        0.8,
        Scalar(0, 0, 0),
        2
    );

    putText(
        plot,
        "time / s",
        Point(width / 2, height - 25),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        Scalar(0, 0, 0),
        2
    );


    imwrite(
        path,
        plot
    );
}
void writeFitResultMarkdown(
    const string& path,
    const AngleModelParams& p,
    double rmse,
    size_t validSamples,
    int firstFrame,
    int lastFrame,
    double fps)
{
    ofstream out(path);

    if (!out.is_open())
    {
        cerr
            << "Cannot write result markdown!"
            << endl;

        return;
    }


    double period =
        2.0 * CV_PI / p.Omega;


    out << "# Task 2: Synthetic Rotation Video Parameter Fitting\n\n";

    out << "## Model\n\n";

    out << "$$\\omega(t)=b+A\\sin(\\Omega t+\\phi)$$\n\n";

    out << "$$\\theta(t)=\\theta_0+bt+"
           "\\frac{A}{\\Omega}"
           "[\\cos\\phi-\\cos(\\Omega t+\\phi)]$$\n\n";


    out << "## Estimated Parameters\n\n";

    out << "- A = "
        << p.A
        << " rad/s\n";

    out << "- b = "
        << p.b
        << " rad/s\n";

    out << "- Omega = "
        << p.Omega
        << " rad/s\n";

    out << "- phi = "
        << p.phi
        << " rad\n";

    out << "- theta0 = "
        << p.theta0
        << " rad\n";

    out << "- Velocity variation period = "
        << period
        << " s\n\n";


    out << "## Method\n\n";

    out << "The cyan target center is extracted using HSV "
           "segmentation, contour detection and image moments. "
           "The target angle is calculated using atan2 and then "
           "unwrapped. The integrated angle model is fitted using "
           "Ceres Solver with automatic differentiation.\n\n";

    out << "The parameterization b = A + d with A > 0 and d > 0 "
           "is used to enforce b > A. Omega is also constrained "
           "to be positive.\n\n";


    out << "## Error\n\n";

    out << "- Angle RMSE = "
        << rmse
        << " rad\n";

    out << "- Valid samples = "
        << validSamples
        << "\n";

    out << "- Frame range = "
        << firstFrame
        << " - "
        << lastFrame
        << "\n";

    out << "- FPS = "
        << fps
        << "\n\n";


    out << "## Outputs\n\n";

    out << "- `result/task2_fit/tracking_overlay.mp4`\n";
    out << "- `result/task2_fit/fit_comparison.png`\n";
    out << "- `result/task2_fit/angular_velocity.png`\n";
    out << "- `result/task2_fit/residuals.png`\n";
}
int main()
{
	VideoCapture cap("resources/task_2.mp4");
	if(!cap.isOpened())
	{
		cerr<<"	Cannot open video!"<<endl;
		return -1;
	}
	 double fps = cap.get(CAP_PROP_FPS);
    int width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    int frameCount = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));

    cout << "width = " << width << endl;
    cout << "height = " << height << endl;
    cout << "fps = " << fps << endl;
    cout << "frame count = " << frameCount << endl;
Mat frame;

    Point2d rotationCenter(
        480.0,
        360.0
    );
 vector<double> times;

    vector<double> anglesWrapped;

    vector<double> anglesUnwrapped;
vector<int> validFrameIndices;
bool hasPreviousAngle =
        false;

    double previousWrapped =
        0.0;

    double previousUnwrapped =
        0.0;
VideoWriter writer(
    "result/task2_fit/tracking_overlay.mp4",
    VideoWriter::fourcc('m', 'p', '4', 'v'),
    fps,
    Size(width, height)
);

if (!writer.isOpened())
{
    cerr << "Cannot create output video!" << endl;
    return -1;
}
int frameIndex=0;
while (cap.read(frame))
{
   double t =
    static_cast<double>(frameIndex) / fps;
    Point2d targetCenter;
    Mat mask;

   bool detected= detectCyanCenter(frame, targetCenter, mask);
if (detected)
{
    double thetaWrapped =
        computeWrappedAngle(
            rotationCenter,
            targetCenter
        );
    double thetaUnwrapped =
        unwrapAngle(
            thetaWrapped,
            hasPreviousAngle,
            previousWrapped,
            previousUnwrapped
        );
    times.push_back(t);

    anglesWrapped.push_back(
        thetaWrapped
    );

    anglesUnwrapped.push_back(
        thetaUnwrapped
    );
validFrameIndices.push_back(
        frameIndex
    );
    drawTrackingOverlay(
        frame,
        rotationCenter,
        targetCenter,
        thetaWrapped,
        frameIndex
    );
}
    imshow("frame", frame);
    imshow("mask", mask);
    writer.write(frame);
    frameIndex++;
    if (waitKey(1) == 27){
        break;
}
}
   cap.release();
    writer.release();

    destroyAllWindows();
if (validFrameIndices.empty())
{
    cerr << "No valid frames detected!" << endl;
    return -1;
}

int firstFrame =
    validFrameIndices.front();

int lastFrame =
    validFrameIndices.back();
	AngleModelParams fittedParams =
    fitAngleModel(
        times,
        anglesUnwrapped
    );

if (!fittedParams.success)
{
    cerr
        << "Ceres fitting failed!"
        << endl;

    return -1;
}


vector<double> fittedAngles =
    generateFittedAngles(
        times,
        fittedParams
    );


vector<double> residuals =
    computeAngleResiduals(
        anglesUnwrapped,
        fittedAngles
    );


double rmse =
    computeAngleRMSE(
        residuals
    );


vector<double> fittedAngularVelocity =
    generateFittedAngularVelocity(
        times,
        fittedParams
    );
saveFitComparison(
    times,
    anglesUnwrapped,
    fittedAngles,
    "result/task2_fit/fit_comparison.png"
);

saveResidualPlot(
    times,
    residuals,
    "result/task2_fit/residuals.png"
);

saveAngularVelocityPlot(
    times,
    fittedAngularVelocity,
    "result/task2_fit/angular_velocity.png"
);
writeFitResultMarkdown(
    "result/task2_fit_result.md",
    fittedParams,
    rmse,
    times.size(),
    firstFrame,
    lastFrame,
    fps
);
cout << endl;
cout << "===== Fitting Result =====" << endl;

cout << "A      = "
     << fittedParams.A
     << " rad/s"
     << endl;

cout << "b      = "
     << fittedParams.b
     << " rad/s"
     << endl;

cout << "Omega  = "
     << fittedParams.Omega
     << " rad/s"
     << endl;

cout << "phi    = "
     << fittedParams.phi
     << " rad"
     << endl;

cout << "theta0 = "
     << fittedParams.theta0
     << " rad"
     << endl;
cout << "RMSE = "
     << rmse
     << " rad"
     << endl;
    return 0;
}
