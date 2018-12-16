#include "Lidar.h"



Lidar::Lidar()
{
}


Lidar::~Lidar()
{
}

void Lidar::newMeasurement()
{
	points.clear();
}

void Lidar::addMeasurement(PolarPoint p)
{
	points.push_back(p);
}

void Lidar::addMeasurements(std::vector<PolarPoint> _points)
{
	points.insert(points.end(), _points.begin(), _points.end());
}

void Lidar::setMaxRadius(double val)
{
	maxRadius = MAX(maxRadius, val);
}

void Lidar::setAngleStep(double val) {
    if (angleStep == -1) angleStep = val;
}

Lidar::transformation Lidar::drawProcess(Map* map, transformation position) {
	cv::Size imgSize(1000, 1000);
	cv::Mat img(imgSize, CV_8UC3);
    img = cv::Vec3b(255,255,255);
    int i = 1;

    cv::circle(img, cv::Point(500, 500), 6, cv::Vec3b(0, 0, 255), -1);
    for (int i = 0; i < points.size(); i++) cv::circle(img, ((points[i].asPoint() + Point(maxRadius, maxRadius)) * 500 / maxRadius).asCV(), 3, cv::Vec3b(0, 255, 0) * ((double)i / points.size()));

    cv::imwrite("Localization_" + std::to_string(i++) + ".png", img);
    //cv::imshow("window", img);
    //cv::waitKey();

    std::vector<BoundedLine> lines = getLines();

    for (BoundedLine l : lines) {
        Edge e = l.line.asEdge(Point(-maxRadius), Point(maxRadius), false);
        cv::Point A = ((e.A + Point(maxRadius)) * 500 / maxRadius).asCV();
        cv::Point B = ((e.B + Point(maxRadius)) * 500 / maxRadius).asCV();
        cv::line(img, A, B, cv::Vec3b(0, 0, 255), 2);
    }

    for (BoundedLine l : lines) {
        cv::Point A = ((l.ends[0] + Point(maxRadius)) * 500 / maxRadius).asCV();
        cv::Point B = ((l.ends[1] + Point(maxRadius)) * 500 / maxRadius).asCV();
        cv::line(img, A, B, cv::Vec3b(255, 0, 0), 2);
    }

    cv::imwrite("Localization_" + std::to_string(i++) + ".png", img);
    //cv::imshow("window", img);
    //cv::waitKey();

	std::vector<Point> intersections = getCorners();
    for (Point pt : intersections) cv::circle(img, pt.asCV(Point(maxRadius), imgSize), 6, cv::Vec3b(255, 0, 255) * 0.8, -1);

    cv::imwrite("Localization_" + std::to_string(i++) + ".png", img);
	cv::imshow("window", img);
	cv::waitKey();

    // How much a pixel is i meters.
    double mToP = 2 * 25.4 / 72;

    map->seperateIntoRooms();
    double mapScale = map->scale;
    std::vector<Point> mapCorners = map->cornerVertices;
    for (Point& pt : mapCorners)
    {
        pt -= Point(map->map.cols(), map->map.rows()) / 2.0;
        pt *= mToP * mapScale;
    }

    position = ICP(intersections, mapCorners, position);

    points.clear();

    return position;
}

Line Lidar::leastSquareFit(std::vector<PolarPoint> points)
{
	struct Pt {
		double d, angle;
	};

    double a1 = 0, a2 = 0;

	double n = points.size();
	double sumOfWeights = n;

	double num1 = 0, num2 = 0, den1 = 0, den2 = 0;
	for (int i = 0; i < n; i++) {
		double iP = points[i].d, iTheta = points[i].angle;

		num1 += iP * iP * sin(2 * iTheta);
		den1 += iP * iP * cos(2 * iTheta);

		for (int j = 0; j < n; j++) {
			double jP = points[j].d, jTheta = points[j].angle;

			num2 += iP * jP * cos(iTheta) * sin(jTheta);
			den2 += iP * jP * cos(iTheta + jTheta);

		}
	}

    //*/
    a1 = 0.5 * atan2((num1 - (2.0 / sumOfWeights) * num2), (den1 - (1.0 / sumOfWeights) * den2)) + M_PI/2;
    a2 = 0.5 * atan2((num1 - (2.0 / sumOfWeights) * num2), (den1 - (1.0 / sumOfWeights) * den2));

    double r1 = 0, r2 = 0;
    for (int i = 0; i < n; i++) {
        r1 += points[i].d * cos(points[i].angle - a1);
        r2 += points[i].d * cos(points[i].angle - a2);
    }
    r1 /= sumOfWeights;
    r2 /= sumOfWeights;


    Line l1({a1, r1}), l2({a2,r2});

    double e1 = 0, e2 = 0;

    for (PolarPoint pt : points) {
        e1 += pow(pt.asPoint().getDistance(l1), 2);
        e2 += pow(pt.asPoint().getDistance(l2), 2);
    }

    if (e1 < e2) return l1;
    return l2;
    //*/
}

std::vector<Lidar::BoundedLine> Lidar::getLines()
{
    //*/ Incremental
    std::vector<BoundedLine> lines;
    double threshold = 0.002;
    BoundedLine currentLine;
    for (int i = 0; i < 2; i++) currentLine.ends[i] = points[i];
    std::vector<PolarPoint> currentPoints = {points[0], points[1]};
    currentLine.line = leastSquareFit(currentPoints);

    size_t pointIndex = 2;
    int n = 2;
    for (; pointIndex < points.size(); pointIndex++) {
        PolarPoint& pt = points[pointIndex];

        if (pt.asPoint().getDistance(currentLine.line) > threshold) {
            if (currentPoints.size() > 5) {
                double angle1 = Angle(currentPoints[0].angle - angleStep * n);
                Line line1;
                line1.A = tan(angle1);
                line1.B = 0;
                Point end1 = currentLine.line.getIntersection(line1);
                double angle2 = Angle(currentPoints[currentPoints.size() - 1].angle + angleStep * n);
                Line line2;
                line2.A = tan(angle2);
                line2.B = 0;
                Point end2 = currentLine.line.getIntersection(line2);
                currentLine.ends[0] = end1;
                currentLine.ends[1] = end2;
                lines.push_back(currentLine);
            }
            if (points.size() - pointIndex > 1) {
                currentPoints = {points[pointIndex], points[pointIndex + 1]};
                currentLine.line = leastSquareFit(currentPoints);
                pointIndex++;
            }
        } else {
            currentPoints.push_back(pt);
            currentLine.line = leastSquareFit(currentPoints);
        }
    }
    double angle1 = Angle(currentPoints[0].angle - angleStep * n);
    Line line1;
    line1.A = tan(angle1);
    line1.B = 0;
    Point end1 = currentLine.line.getIntersection(line1);
    double angle2 = Angle(currentPoints[currentPoints.size() - 1].angle + angleStep * n);
    Line line2;
    line2.A = tan(angle2);
    line2.B = 0;
    Point end2 = currentLine.line.getIntersection(line2);
    currentLine.ends[0] = end1;
    currentLine.ends[1] = end2;
    lines.push_back(currentLine);

    //*/
	return lines;
}

std::vector<Point> Lidar::getCorners()
{
	std::vector<Point> corners;
    std::vector<BoundedLine> lines = getLines();

    for (std::vector<BoundedLine>::iterator lA = lines.begin(); lA != lines.end(); lA++) {
        for (std::vector<BoundedLine>::iterator lB = lA + 1; lB != lines.end(); lB++) {
            if (abs(lA->line.a - lB->line.a) > M_PI_4 / 2)  {
                Point corner =  lA->line.getIntersection(lB->line);
                if (Point::isWithin(corner, {lA->ends[0], lA->ends[1]}) && Point::isWithin(corner, {lB->ends[0], lB->ends[1]})) corners.push_back(corner);
            }
		}
	}

	return corners;
	return std::vector<Point>();
}

Lidar::transformation Lidar::ICP(std::vector<Point> cloudA, std::vector<Point> cloudB, transformation position)
{
		transformation T = position;
		std::vector<IterationData> data;

        for (Point& p : cloudA) {
            p = p.rotate(3 * M_PI / 2);
            p = p.rotate(T.angle);
            p += T.p;
		}

		std::vector<Point> _cloudA = cloudA;

		int nA = _cloudA.size(), nB = _cloudA.size();
		int N = 100;
        int i = 0;

		for (int i = 0; i < N; i++) {
			Point centroidA = Point(0, 0), centroidB = Point(0, 0);
			std::vector<sNearestPoint> nPs;

			for (Point pA : _cloudA) {
				centroidA += pA;

				Point nearestPoint;
				double minDist = INT_MAX;
				for (Point pB : cloudB) {
					double dist = pA.getDistance(pB);
					if (dist < minDist) {
						minDist = dist;
						nearestPoint = pB;
					}
				}

				centroidB += nearestPoint;
				nPs.push_back(sNearestPoint({ pA, nearestPoint }));
			}

			centroidA *= 1.0 / nA;
			centroidB *= 1.0 / nA;

			cv::Mat H(2, 2, CV_64F);
			H = 0;
            // Changed the rotation to also be based on just the matched points instead of every point as cloudA will always contain a small amount of the points in cloudB.
            std::vector<Point> matchedCloudB;
            for (sNearestPoint np : nPs) matchedCloudB.push_back(np.nearestPoint);
            int N = MIN(_cloudA.size(), matchedCloudB.size());
			for (int i = 0; i < N; i++) {
                Point pA = _cloudA[i] - centroidA, pB = matchedCloudB[i] - centroidB;
				double pa[2] = { pA.x(), pA.y() };
				double pb[2] = { pB.x(), pB.y() };

				for (int k = 0; k < 4; k++) {
					int x = k % 2, y = k / 2;
					H.at<double>(x, y) += pa[x] * pb[y];
				}
			}

			cv::SVD svd;
			svd(H);
			cv::Mat u = svd.u;
			cv::Mat vt = svd.vt;
			cv::Mat R;
			cv::gemm(u, vt, 1, vt, 0, R);

			double a = acos(R.at<double>(0, 0));
			Point cAR = centroidA.rotate(a);
			Point cBR = centroidB.rotate(a);
			Point translation = centroidB - cAR;

			T.angle += a;
			T.p = T.p.rotate(a);
			T.p += translation;

			for (Point& p : _cloudA) {
				p = p.rotate(a);
				p += translation;
			}

			IterationData d;
			d.centroids[0] = centroidA, d.centroids[1] = centroidB;
			d.nPs = nPs;
			data.push_back(d);
		}

		Point min(0, 0), max(0, 0);

		std::vector<Point> points;
		for (IterationData d : data) {
			for (int i = 0; i < d.nPs.size(); i++) {
				points.push_back(d.nPs[i].p);
				points.push_back(d.nPs[i].nearestPoint);
			}
		}
		for (Point p : cloudA) points.push_back(p);
		for (Point p : cloudB) points.push_back(p);


		for (Point p : points) {
			min.x() = MIN(p.x(), min.x());
			min.y() = MIN(p.y(), min.y());
			max.x() = MAX(p.x(), max.x());
			max.y() = MAX(p.y(), max.y());
		}

		Point translation = Point(0, 0) - min;
		Point scale = Point(abs(max.x() - min.x()), abs(max.y() - min.y()));

		for (Point& p : cloudA) {
			p += translation;
			p.x() /= scale.x();
			p.y() /= scale.y();
		}

		for (Point& p : cloudB) {
			p += translation;
			p.x() /= scale.x();
			p.y() /= scale.y();
		}

		int width = 1720, height = 980, r = 3;
		cv::Vec3b colorCloud = cv::Vec3b(120, 40, 0), colorActiveTarget = cv::Vec3b(0, 40, 120), colorPassiveTarget = colorActiveTarget * 0.5, colorLine = cv::Vec3b(60, 60, 60), colorCentroidA = cv::Vec3b(255, 0, 0), colorCentroidB = cv::Vec3b(0, 0, 255);

		for (IterationData d : data) {
			cv::Mat img(cv::Size(width, height), CV_8UC3);
            img = cv::Vec3b(255,255,255);
            for (int i = 0; i < 2; i++) {
				d.centroids[i] += translation;
				d.centroids[i].x() *= width / scale.x();
				d.centroids[i].y() *= height / scale.y();
			}
			for (sNearestPoint& np : d.nPs) {
				np.nearestPoint += translation;
				np.nearestPoint.x() *= width / scale.x();
				np.nearestPoint.y() *= height / scale.y();

				np.p += translation;
				np.p.x() *= width / scale.x();
				np.p.y() *= height / scale.y();
			}
			for (int i = 0; i < d.nPs.size(); i++) cv::line(img, d.nPs[i].nearestPoint.asCV(), d.nPs[i].p.asCV(), colorLine);
			cv::Rect rectA(d.centroids[0].asCV(), cv::Size(r, r) * 2);
			cv::rectangle(img, rectA, colorCentroidA, -1);
			cv::Rect rectB(d.centroids[1].asCV(), cv::Size(r, r) * 2);
			cv::rectangle(img, rectB, colorCentroidB, -1);
			for (Point p : cloudB) cv::circle(img, p.asCV(width, height), r, colorPassiveTarget, -1);
			for (sNearestPoint np : d.nPs) {
				cv::circle(img, np.nearestPoint.asCV(), r, colorActiveTarget, -1);
				cv::circle(img, np.p.asCV(), r, colorCloud, -1);
			}
			cv::imshow("Image", img);
            cv::imwrite("PSR_Test_" + std::to_string(++i) + ".png", img);
			cv::waitKey();
		}

		return T;
}
