#pragma once

#include "Point.h"
#include "Edge.h"

#include <unordered_map>
#include <opencv2/opencv.hpp>


class GraphDrawer
{
public:
	GraphDrawer();
    GraphDrawer(cv::Size imageSize) : imageSize(imageSize) {}
	~GraphDrawer();

	void addPoint(Point p, std::string pointset = "");
	void addPoints(std::vector<Point> pts, std::string pointset = "");
	void addLine(Line l, std::string lineName = "");
	void setColor(cv::Vec3b color, std::string pointset = "");
	void setImageSize(int width, int height);
	void setImageSize(cv::Size size);
	void setRadius(int r, std::string pointset = "");

	cv::Mat getImage();
private:
	struct sPointset {
		std::string name;
		std::vector<Point> pts;
		cv::Vec3b color;
		int radius = 3;
		int thickness = -1;
	};

	cv::Size imageSize;
	std::vector<std::string> pointsetNames;
	std::unordered_map<std::string, sPointset> pointsets;
	std::vector<Line> lines;

	bool isPointset(std::string pointset);
};

