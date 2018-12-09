#include "GraphDrawer.h"



GraphDrawer::GraphDrawer()
{
}


GraphDrawer::~GraphDrawer()
{
}

void GraphDrawer::addPoint(Point p, std::string pointset)
{
	if (isPointset(pointset)) {
		pointsets[pointset].pts.push_back(p);
	}
	else {
		sPointset ps;
		ps.pts = std::vector<Point>();
		ps.pts.push_back(p);
		ps.name = pointset;
		pointsetNames.push_back(pointset);
		pointsets[pointset] = ps;
	}
}

void GraphDrawer::addPoints(std::vector<Point> pts, std::string pointset)
{
	if (isPointset(pointset)) {
		pointsets[pointset].pts.insert(pointsets[pointset].pts.begin(), pts.begin(), pts.end());
	}
	else {
		sPointset ps;
		ps.pts = pts;
		ps.name = pointset;
		pointsetNames.push_back(pointset);
		pointsets[pointset] = ps;
	}
}

void GraphDrawer::addLine(Line l, std::string lineName)
{
	lines.push_back(l);
}

void GraphDrawer::setColor(cv::Vec3b color, std::string pointset)
{
	if (isPointset(pointset)) pointsets[pointset].color = color;
}

void GraphDrawer::setImageSize(int width, int height)
{
	imageSize = cv::Size(width, height);
}

void GraphDrawer::setImageSize(cv::Size size)
{
	imageSize = size;
}

void GraphDrawer::setRadius(int r, std::string pointset)
{
	if (isPointset(pointset)) pointsets[pointset].radius = r;
}

cv::Mat GraphDrawer::getImage()
{
	cv::Mat image(imageSize, CV_8UC3);
	if (imageSize != cv::Size()) {
		Point scale, translation;

		Point min(0, 0), max(0, 0);

		for (std::string s : pointsetNames) {
			for (Point p : pointsets[s].pts) {
				min.x() = MIN(p.x(), min.x());
				min.y() = MIN(p.y(), min.y());
				max.x() = MAX(p.x(), max.x());
				max.y() = MAX(p.y(), max.y());
			}
		}

		scale.x() = (double)imageSize.width / (double)(max.x() - min.x());
		scale.y() = -1 * (double)imageSize.height / (double)(max.y() - min.y());

		translation.x() = -1 * min.x() * scale.x();
		translation.y() = -1 * max.y() * scale.y();

		for (std::string s : pointsetNames) {
			sPointset* ps = &pointsets[s];
			for (Point p : ps->pts) cv::circle(image, (p.times(scale) + translation).getCVPoint(), ps->radius, ps->color, ps->thickness);
		}

		for (Line l : lines) {
			Edge e = l.asEdge(min, max);
			cv::line(image, (e.A.times(scale) + translation).getCVPoint(), (e.B.times(scale) + translation).getCVPoint(), cv::Vec3b(0,0,0));
		}
	}
	return image;
}

bool GraphDrawer::isPointset(std::string pointset)
{
	for (std::string s : pointsetNames) if (s == pointset) return true;
	return false;
}
