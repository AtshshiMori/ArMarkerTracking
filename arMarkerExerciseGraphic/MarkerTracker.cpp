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

void cycleArray(vector<Point2f>&);

MarkerTracker::MarkerTracker(const double MarkerSize) {
	kMarkerSize = MarkerSize;
};

void MarkerTracker::findMarker(cv::Mat& srcimg, map<int, vector<float>> &markerSet)
{
	img = srcimg;
	// �摜���ǂݍ��܂�Ȃ�������v���O�����I��
	if (img.empty()) {
		throw "�t�@�C�����ǂݍ��߂܂���";
	}
	//�O���[�X�P�[���ϊ�
	cvtColor(img, img_gray, CV_RGB2GRAY);
	
	//��l�ϊ�
	Mat img_bin;//�o�C�i���C���[�W
	threshold(img_gray, img_bin, 100, 255, CV_THRESH_BINARY_INV);
	
	//�֊s���o
	vector< vector<Point> > contours;//�֊s�̏W��
	vector<Vec4i> hierarchy;
	findContours(img_bin, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	//�}�[�J�[�̂݌��o
	vector< vector<Point> > approxContours;
	for (auto contour = contours.begin(); contour != contours.end(); contour++) {
			
		approxPolyDP(*contour, *contour, arcLength(*contour, true)*0.02 , true);
		double size = contourArea(*contour);
		if (size > 500 && (*contour).size() == 4) { //�T�C�Y���傫���A���_���S�_�̂��̂������o
			approxContours.push_back(*contour);
		}

	}
	if (approxContours.size() == 0)return;


	/*-------------------------------------
		�f�o�b�O�p
	--------------------------------------*/
	for (auto cont = approxContours.begin(); cont != approxContours.end(); cont++) {
		for (auto p = (*cont).begin(); p != (*cont).end(); p++) {
			//circle(img, *p, 5, red, -1);//�m�F�̂��߂̏o��
		}
	}
	/*-------------------------------------
		�f�o�b�O�I���
	--------------------------------------*/

	
	bool noMarkerFlag = false;//�}�[�J�[���ǂ���
	vector<Vec4f> lines(4);//4��

	//�e�֊s�ɂ���id�ƕϊ��s������߂�
	for (auto square_itr = approxContours.begin(); square_itr != approxContours.end(); square_itr++) {
		
		vector<Point> square = *square_itr;

		//�ӂ�␳����
		if(!correctSide(square, lines))continue;//���s�����玟�̎l�p�`�Ɉڂ�
		
		//�S�̌�_�����߂�
		vector<Point2f> lineInters(4);//��_�̏W��
		getInters(lines, lineInters);

		for (auto p = lineInters.begin(); p != lineInters.end(); p++) {
			circle(img,*p, 5, blue, -1);//�m�F�̂��߂̏o��
		}

		int smallestId = 100000000;//�ŏ���Id
		int smallestIdnum;//�ŏ���Id�̂Ƃ��̔ԍ�
		
		//�l�����̌�����id���v�Z���čŏ��̂��̂Ŋm�肳����
		for (int a=0; a < 4; a++) {

			//�����ϊ�
			Mat idImg = Mat::zeros(6, 6, CV_8UC1);
			Point2f idImgPoints[4] = { Point2f{ 0,5 },Point2f{ 5,5 },Point2f{ 5,0 },Point2f{ 0,0 } };

			Mat homography_mat = getPerspectiveTransform(&lineInters.front(), idImgPoints);
			warpPerspective(img_gray, idImg, homography_mat, idImg.size());

			//��l�ϊ�
			threshold(idImg, idImg, 135, 255, CV_THRESH_BINARY_INV);

			Mat idImg44(idImg, Rect(1, 1, 4, 4));

			//id�擾
			int rowId[4];
			int count = 0;//1�̐��܂荕�̕����̐��𐔂���
			for (int i = 0; i < 4; i++) {

				Mat row = idImg44.row(i) / 255;
				int num = row.at<unsigned char>(3) + (row.at<unsigned char>(2) * 2) + (row.at<unsigned char>(1) * 4) + (row.at<unsigned char>(0) * 8);
				rowId[i] = num;

				for (int j = 0; j < 4; j++) {
					if (row.at<unsigned char>(j) == 1)count++;
				}
			}

			//���܂��͔�������������̃}�[�J�[�łȂ��Ɣ��f���ď��O
			if (count < 3 || count >13) {
				noMarkerFlag = true;
				break;
			
			}


			/*-------------------------------------
			 �f�o�b�O�p
			--------------------------------------*/
			//cout << "id:";
			//for (auto i : rowId) {
			//	cout << i << " ";
			//}
			//Mat idImg44_debug;
			//resize(idImg44, idImg44_debug, Size(), 100, 100, INTER_AREA);//�����₷���悤�g��
			//imshow("id"+a, idImg44_debug);
			//waitKey(0);
			/*-------------------------------------
				�f�o�b�O�I���
			--------------------------------------*/


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
		
		//id���`�F�b�N�������Ƃ��Ɏg���֐�
		//checkMarkerId(lineInters, smallestId);

		//���S�ɂ��炷
		for (int i = 0; i < 4; i++) {
			lineInters[i] -= Point2f{ img.cols / 2.0f - 0.5f,img.rows / 2.0f - 0.5f };

		}
		//�|�[�Y����
		vector<float> resultMatrix(16);
		estimateSquarePose(&resultMatrix.front(), &lineInters.front(), kMarkerSize);

		markerSet[smallestId] =  resultMatrix;
	}
	return;
	/*-------------------------------------
		debug�p
	--------------------------------------*/

	
	//resize(idImg, idImg, Size(), 10, 10, INTER_AREA);
	//imshow("idImg", idImg);
	imshow("idImg", img);
	waitKey(0);
	//drawContours(img, approxContours, -1, red, 1, 8);

}

void cycleArray(vector<Point2f> &array) {
	Point tmp = array[0];
	array[0] = array[3];
	array[3] = array[2];
	array[2] = array[1];
	array[1] = tmp;

};


/*-------------------------------------
	�l�p�`�̕ӂ��C������֐�
		square	�l�p�`�̎l�_�̍��W�z��
		lines	�ӂ̌��ʂ�����z��
--------------------------------------*/

bool MarkerTracker::correctSide(vector<Point> square, vector<Vec4f> &lines) {
	
	//�e�ӂ��Ƃɐ��m�ȗ֊s�𓾂�
	for (int j = 0; j < 4; j++) {

		Point corner1 = square.at(j);
		Point corner2;
		if (j == 3) {
			corner2 = square.at(0);
		}
		else {
			corner2 = square.at(j + 1);
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
			int stripeLength = (int)(0.05*sqrt(dx*dx + dy * dy));
			if (stripeLength < 5) {
				stripeLength = 5;
			}

			Mat stripe;//�ӂ���؂�����X�g���C�v
			Mat stripe_sobel;//stripe�̔����l
			Rect roi(splitPoint[i].x - 1, splitPoint[i].y - stripeLength / 2, 3, stripeLength);

			if ((roi & cv::Rect(0, 0, img_rot.cols, img_rot.rows)) == roi) {
				stripe = img_rot(roi);
			}
			else {
				return false;
			}

			//Sobel Operator
			stripe.convertTo(stripe_sobel, CV_64F);
			Sobel(stripe_sobel, stripe_sobel, CV_64F, 0, 1);
			stripe_sobel = abs(stripe_sobel);

			//�ő�s�����߂�
			Point maxLoc;
			minMaxLoc(stripe_sobel, NULL, NULL, NULL, &maxLoc);
			int maxLow = maxLoc.y;

			double p1, p2, p3;
			p1 = stripe_sobel.at<double>(maxLow - 1, 1);
			p2 = stripe_sobel.at<double>(maxLow, 1);
			p3 = stripe_sobel.at<double>(maxLow + 1, 1);

			//�A���ꎟ������������
			Mat A = (Mat_<double>(3, 3) << 1, -1, 1, 0, 0, 1, 1, 1, 1);
			Mat P = (Mat_<double>(3, 1) << p1, p2, p3);

			Mat coe;//coefficient(�񎟋Ȑ��̌W��)
			solve(A, P, coe);

			//�X�g���C�v��̓_���猳�̉摜�ł̓_�𓾂�
			double peak = coe.at<double>(1) / (-2 * coe.at<double>(0));//2���Ȑ��̒��_

			Point2d p{ 1, maxLow + peak };//stripe�ł̋��E�̓_�̍��W
			Point2i pcenter{ 1, stripeLength / 2 };//stripe�̒��S


			/*-------------------------------------
				�f�o�b�O�p
			--------------------------------------*/
			//cv:Mat stripe_after;
			//circle(img, corner1, 5, yellow, -1);//���_
			//circle(img, corner2, 5, yellow, -1);//���_
			//circle(img, splitPoint[i], 5, red, -1);//�����_
			//circle(stripe, p, 1, red, -1);//stripe��ɋ��E�_�Ƃ��Č��܂����ʒu��\��
			//resize(stripe, stripe_after, Size(), 10, 10, INTER_AREA);//�����₷���悤�g��
			//
			//imshow("stripe", stripe_after);
			//imshow("img", img);
			//waitKey(0);

			/*-------------------------------------
				�f�o�b�O�I���
			--------------------------------------*/


			Point2d vec{ p.x - pcenter.x, p.y - pcenter.y };

			double angleIn = angle * PI / 180.0;
			Mat rot = (Mat_<double>(2, 2) << cos(angleIn), -sin(angleIn), sin(angleIn), cos(angleIn));
			Mat M = rot * Mat(vec);
			vec.x = M.at<double>(0, 0);
			vec.y = M.at<double>(0, 1);

			Point2d borderPoint = (Point2d)splitPoint[i] + vec;//�␳��̋��E�_
			borderPoints.push_back(borderPoint);
		}
		
		//���߂��U�_���璼���Ƀt�B�b�e�B���O
		Vec4f line;
		fitLine(borderPoints, line, CV_DIST_L2, 0, 0.01, 0.01);
		lines[j] = line;
	}
	return true;
}
/*-------------------------------------
4�ӂ����_�����߂�֐�
	lines	4�ӂ̔z��
	lineInters ���߂���_������z��
--------------------------------------*/
void MarkerTracker::getInters(vector<Vec4f> lines, vector<Point2f> &lineInters) {
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
}

void MarkerTracker::checkMarkerId(vector<Point2f> lineInters, int smallestId) {
	//�����ϊ�
	Mat idImg = Mat::zeros(6, 6, CV_8UC1);
	Point2f idImgPoints[4] = { Point2f{ 0,5 },Point2f{ 5,5 },Point2f{ 5,0 },Point2f{ 0,0 } };

	Mat homography_mat = getPerspectiveTransform(&lineInters.front(), idImgPoints);
	warpPerspective(img_gray, idImg, homography_mat, idImg.size());

	//��l�ϊ�
	threshold(idImg, idImg, 135, 255, CV_THRESH_BINARY_INV);

	Mat idImg44(idImg, Rect(1, 1, 4, 4));

	cout << "id:" << smallestId << endl;
	Mat idImg44_debug;
	resize(idImg44, idImg44_debug, Size(), 100, 100, INTER_AREA);//�����₷���悤�g��
	imshow("id", idImg44_debug);
	waitKey(0);
}