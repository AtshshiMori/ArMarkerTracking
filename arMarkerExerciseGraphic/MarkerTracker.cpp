// armarkertoracking.cpp : �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include <math.h>
#include <opencv2/opencv.hpp>
#include "PoseEstimation.h"
#include "MarkerTracker.h"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define PI 3.141592653589793

using namespace std;
using namespace cv;

const Scalar red(0, 0, 255);
const Scalar green(0, 255, 0);
const Scalar blue(255, 0, 0);
const Scalar yellow(0, 255, 255);

void cycleArray(Point2f[4]);

MarkerTracker::MarkerTracker(const double MarkerSize) {
	kMarkerSize = MarkerSize;
};

bool MarkerTracker::findMarker(cv::Mat& img, float resultMatrix[16])
{


	// �摜���ǂݍ��܂�Ȃ�������v���O�����I��
	if (img.empty()) {
		throw "�t�@�C�����ǂݍ��߂܂���";
	}
	//�O���[�X�P�[���ϊ�
	Mat img_gray;
	cvtColor(img, img_gray, CV_RGB2GRAY);

	//��l�ϊ�
	Mat img_bin;//�o�C�i���C���[�W
	threshold(img_gray, img_bin, 150, 255, CV_THRESH_BINARY_INV);

	//�֊s���o
	vector< vector<Point> > contours;//�֊s�̏W��
	vector<Vec4i> hierarchy;
	findContours(img_bin, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	//�}�[�J�[�̂݌��o
	vector< vector<Point> > approxContours;
	vector<Point> approxContour;
	vector<Point>::iterator itr;
	for (auto contour = contours.begin(); contour != contours.end(); contour++) {

		approxPolyDP(*contour, approxContour, 5, true);
		double size = contourArea(approxContour);
		if (size > 500 && approxContour.size() == 4) { //�T�C�Y���傫���A���_���S�_�̂��̂������o
			approxContours.push_back(approxContour);
		}

	}
	if (approxContours.size() == 0)return false;

	//�e�ӂ��Ƃɐ��m�ȗ֊s�𓾂�
	vector<Vec4f> lines(4);
	for (int k = 0; k < approxContours.size();k++) {
	for (int j = 0; j < 4; j++) {

		Point corner1 = approxContours.at(k).at(j);
		Point corner2;
		if (j == 3) {
			corner2 = approxContours.at(k).at(0);
		}
		else {
			corner2 = approxContours.at(k).at(j + 1);
		}

		Point splitPoint[6];
		vector<Point> borderPoints;
		//�V���������_���Ƃɏ���
		for (int i = 0; i < 6; i++) {
			//�_�̎擾
			splitPoint[i] = ((6 - i)*corner1 + (i + 1)*corner2) / 7;

			//�摜����]
			int dx = corner1.x - corner2.x;
			int dy = corner1.y - corner2.y;
			double angle = atan2(dy, dx) * 180.0 / PI;
			Mat mapMat = getRotationMatrix2D(splitPoint[i], angle, 1);

			Mat img_rot;
			warpAffine(img_gray, img_rot, mapMat, Size());

			//stripe��؂蔲��
			int stripeLength = (int)(0.3*sqrt(dx*dx + dy * dy));
			if (stripeLength < 5) {
				stripeLength = 5;
			}

			Mat stripe;
			Rect roi(splitPoint[i].x - 1, splitPoint[i].y - stripeLength / 2, 3, stripeLength);

			if((roi & cv::Rect(0, 0, img_rot.cols, img_rot.rows)) == roi){
				stripe = img_rot(roi);
			}
			else{
				return false;
			}
			

			//Sobel Operator
			stripe.convertTo(stripe, CV_64F);
			Sobel(stripe, stripe, CV_64F, 0, 1);
			stripe = abs(stripe);


			//�ő�s�����߂�
			Point maxLoc;
			minMaxLoc(stripe, NULL, NULL, NULL, &maxLoc);
			int maxLow = maxLoc.y;

			double p1, p2, p3;
			p1 = stripe.at<double>(maxLow - 1, 1);
			p2 = stripe.at<double>(maxLow, 1);
			p3 = stripe.at<double>(maxLow + 1, 1);

			//�A���ꎟ������������
			Mat A = (Mat_<double>(3, 3) << 1, -1, 1, 0, 0, 1, 1, 1, 1);
			Mat P = (Mat_<double>(3, 1) << p1, p2, p3);

			Mat coe;//coefficient(�񎟋Ȑ��̌W��)
			solve(A, P, coe);

			//�X�g���C�v��̓_���猳�̉摜�ł̓_�𓾂�
			double peak = coe.at<double>(1) / (-2 * coe.at<double>(0));//2���Ȑ��̒��_

			Point2d p{ 1, maxLow + peak };//stripe�ł̋��E�̓_�̍��W
			Point2i pcenter{ 1, stripeLength / 2 };//stripe�̒��S

			Point2d vec{ p.x - pcenter.x, p.y - pcenter.y };

			double angleIn = angle * PI / 180.0;
			Mat rot = (Mat_<double>(2, 2) << cos(angleIn), -sin(angleIn), sin(angleIn), cos(angleIn));
			Mat M = rot * Mat(vec);
			vec.x = M.at<double>(0, 0);
			vec.y = M.at<double>(0, 1);

			Point2d borderPoint = (Point2d)splitPoint[i] + vec;//�␳��̋��E�_
			borderPoints.push_back(borderPoint);

			rectangle(img, borderPoints[i], borderPoints[i], red);//�m�F�̂��߂̏o��
		}

		//���߂��U�_���璼���Ƀt�B�b�e�B���O
		Vec4f line;
		fitLine(borderPoints, line, CV_DIST_L2, 0, 0.01, 0.01);
		lines[j] = line;

		//�m�F�̂��ߏo��
		Point2f p{ line[2] - line[0] * 10, line[3] - line[1] * 10 };
		Point2f p2{ line[2] + line[0] * 10, line[3] + line[1] * 10 };
		cv::line(img, p, p2, green);

	}
	//�S�̌�_�����߂�
	Point2f lineInters[4];//��_�̏W��
	for (int i = 0; i < 4; i++) {
		int j;
		if (i == 3) {
			j = 0;
		}
		else {
			j = i + 1;
		}
		double vx1 = lines[i][0];
		double vy1 = lines[i][1];
		double x01 = lines[i][2];
		double y01 = lines[i][3];
		double vx2 = lines[j][0];
		double vy2 = lines[j][1];
		double x02 = lines[j][2];
		double y02 = lines[j][3];

		Mat A = (Mat_<double>(2, 2) << vy1, -vx1, vy2, -vx2);
		Mat B = (Mat_<double>(2, 1) << -vx1 * y01 + vy1 * x01, -vx2 * y02 + vy2 * x02);

		Mat lineInter;//��_

		solve(A, B, lineInter);
		lineInters[i] = (Point2f)lineInter;
		

	}

	int smallestId = 1000000;//�ŏ���Id
	int smallestIdnum;//�ŏ���Id�̂Ƃ��̔ԍ�
	bool noMarkerFlag = false;
	for (int a=0; a < 4; a++) {

		//�����ϊ�
		Mat idImg = Mat::zeros(6, 6, CV_8UC1);
		Point2f idImgPoints[4] = { Point2f{ 0,5 },Point2f{ 5,5 },Point2f{ 5,0 },Point2f{ 0,0 } };

		Mat homography_mat = getPerspectiveTransform(lineInters, idImgPoints);
		warpPerspective(img_gray, idImg, homography_mat, idImg.size());

		//��l�ϊ�
		threshold(idImg, idImg, 135, 255, CV_THRESH_BINARY_INV);

		Mat idImg44(idImg, Rect(1, 1, 4, 4));
		//id�擾
		int rowId[4];
		int count = 0;//1�̐��܂荕�̕����̐��𐔂���
		for (int i = 0; i < 4; i++) {

			Mat row = idImg44.row(i) / 255;
			//cout  << row << endl;
			int num = row.at<unsigned char>(3) + (row.at<unsigned char>(2) * 2) + (row.at<unsigned char>(1) * 4) + (row.at<unsigned char>(0) * 8);
			rowId[i] = num;

			for (int j = 0; j < 4; j++) {
				if (row.at<unsigned char>(j) == 1)count++;
			}
		}
		//cout << endl;
		//cout << endl;

		//���܂��͔�������������̃}�[�J�[�łȂ��Ɣ��f���ď��O
		if (count < 3 || count >13) {
			noMarkerFlag = true;
			break;
			
		}
		int id = rowId[3] + rowId[2] * 16 + rowId[1] * 16 * 16 + rowId[0] * 16 * 16 * 16;

		if (id < smallestId) {
			smallestId = id;
			smallestIdnum = a;
		}
		//�S�̌�_���Ԃ����ւ��ČJ��Ԃ�
		cycleArray(lineInters);
	}
	//�}�[�J�[�ȊO�̕��Ȃ�~�߂�
	if (noMarkerFlag == true)continue;

	//�ŏ��̎��̏��ԂɌ�_����ёւ���
	for (int a = 0; a < smallestIdnum; a++) {
		cycleArray(lineInters);
	}

	//�S�_�̕`��
	for (int i = 0; i < 4; i++) {
		circle(img, lineInters[i], 1, blue, 3, 8);

	}

	//���S�ɂ��炷
	for (int i = 0; i < 4; i++) {
		lineInters[i] -= Point2f{ img.cols / 2.0f - 0.5f,img.rows / 2.0f - 0.5f };

	}
	//�|�[�Y����
	estimateSquarePose(resultMatrix, lineInters, kMarkerSize);

	
	//system("cls");
	/*for (int i = 0; i < 16; i++) {

		cout << resultMatrix[i] << "\t";
		if (i % 4 == 3)cout << endl;
	}*/
	return true;
}
return false;
	/*-------------------------------------
		debug�p
	--------------------------------------*/

	
	//resize(idImg, idImg, Size(), 10, 10, INTER_AREA);
	//imshow("idImg", idImg);

	//drawContours(img, approxContours, -1, red, 1, 8);

	//�摜�\��
	imshow("image", img);

	waitKey();
}

void cycleArray(Point2f array[4]) {
	Point tmp = array[0];
	array[0] = array[3];
	array[3] = array[2];
	array[2] = array[1];
	array[1] = tmp;

};