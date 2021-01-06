/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * samples/poses example.
 * Shows the common PDF and 2D/3D pose manipulation operations.
 */

#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <iostream>

#include <fstream>
#include <unistd.h>
#include <sophus/se3.hpp>
#include <iomanip>


using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;
typedef vector<CPose3DPDFGaussian, Eigen::aligned_allocator<CPose3DPDFGaussian>> PosecovType;
typedef vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> PoseType;

// ------------------------------------------------------
//				TestPosePDFOperations
// ------------------------------------------------------
void TestPosePDFOperations(const string &posepath, const string &covpath)
{
    PosecovType vec_p6;
    PoseType pose7b;
    CPose3DQuatPDFGaussian p7a;
    CPose3DQuatPDFGaussian p7b;
    CPose3DQuatPDFGaussian p7;
    int i = 0;

    p7a.mean[0] = -7.137748e-03;
    p7a.mean[1] = -7.482656e-02;
    p7a.mean[2] = -3.336324e-01;
    p7a.mean[3] = 0.5012;
    p7a.mean[4] = 0.5023;
    p7a.mean[5] = -0.4988;
    p7a.mean[6] = 0.4977;

    for(int i=0; i< 7;i++)
    {
        for(int j=0; j< 7;j++)
        {
            p7a.cov(i, j) = 0;
        }
    }


    ifstream fin(posepath);
    if (!fin)
    {
        cerr << "trajectory " << posepath << " not found." << endl;
    }

    while (!fin.eof())
    {
        Eigen::Matrix<double, 7, 1> pose;
        double x, y, z, qw, qx, qy, qz;
        fin >> x >> y >> z >> qw >> qx >> qy >> qz;
        pose[0] = x;
        pose[1] = y;
        pose[2] = z;
        pose[3] = qw;
        pose[4] = qx;
        pose[5] = qy;
        pose[6] = qz;
        pose7b.push_back(pose);
    }


    ifstream fin1(covpath);
    if (!fin1)
    {
        cerr << "cov " << covpath << " not found." << endl;
    }

    while (!fin1.eof())
    {

        double cov00, cov01, cov02, cov03, cov04, cov05, cov06;
        double cov10, cov11, cov12, cov13, cov14, cov15, cov16;
        double cov20, cov21, cov22, cov23, cov24, cov25, cov26;
        double cov30, cov31, cov32, cov33, cov34, cov35, cov36;
        double cov40, cov41, cov42, cov43, cov44, cov45, cov46;
        double cov50, cov51, cov52, cov53, cov54, cov55, cov56;
        double cov60, cov61, cov62, cov63, cov64, cov65, cov66;
        fin1 >> cov00 >> cov01 >> cov02 >> cov03 >> cov04 >> cov05 >> cov06
                >> cov10 >> cov11 >> cov12 >> cov13 >> cov14 >> cov15 >> cov16
                >> cov20 >> cov21 >> cov22 >> cov23 >> cov24 >> cov25 >> cov26
                >> cov30 >> cov31 >> cov32 >> cov33 >> cov34 >> cov35 >> cov36
                >> cov40 >> cov41 >> cov42 >> cov43 >> cov44 >> cov45 >> cov46
                >> cov50 >> cov51 >> cov52 >> cov53 >> cov54 >> cov55 >> cov56
                >> cov60 >> cov61 >> cov62 >> cov63 >> cov64 >> cov65 >> cov66;

        p7b.cov(0, 0)= cov00;
        p7b.cov(0, 1)= cov01;
        p7b.cov(0, 2)= cov02;
        p7b.cov(0, 3)= cov03;
        p7b.cov(0, 4)= cov04;
        p7b.cov(0, 5)= cov05;
        p7b.cov(0, 6)= cov06;

        p7b.cov(1, 0)= cov10;
        p7b.cov(1, 1)= cov11;
        p7b.cov(1, 2)= cov12;
        p7b.cov(1, 3)= cov13;
        p7b.cov(1, 4)= cov14;
        p7b.cov(1, 5)= cov15;
        p7b.cov(1, 6)= cov16;

        p7b.cov(2, 0)= cov20;
        p7b.cov(2, 1)= cov21;
        p7b.cov(2, 2)= cov22;
        p7b.cov(2, 3)= cov23;
        p7b.cov(2, 4)= cov24;
        p7b.cov(2, 5)= cov25;
        p7b.cov(2, 6)= cov26;

        p7b.cov(3, 0)= cov30;
        p7b.cov(3, 1)= cov31;
        p7b.cov(3, 2)= cov32;
        p7b.cov(3, 3)= cov33;
        p7b.cov(3, 4)= cov34;
        p7b.cov(3, 5)= cov35;
        p7b.cov(3, 6)= cov36;

        p7b.cov(4, 0)= cov40;
        p7b.cov(4, 1)= cov41;
        p7b.cov(4, 2)= cov42;
        p7b.cov(4, 3)= cov43;
        p7b.cov(4, 4)= cov44;
        p7b.cov(4, 5)= cov45;
        p7b.cov(4, 6)= cov46;

        p7b.cov(5, 0)= cov50;
        p7b.cov(5, 1)= cov51;
        p7b.cov(5, 2)= cov52;
        p7b.cov(5, 3)= cov53;
        p7b.cov(5, 4)= cov54;
        p7b.cov(5, 5)= cov55;
        p7b.cov(5, 6)= cov56;

        p7b.cov(6, 0)= cov60;
        p7b.cov(6, 1)= cov61;
        p7b.cov(6, 2)= cov62;
        p7b.cov(6, 3)= cov63;
        p7b.cov(6, 4)= cov64;
        p7b.cov(6, 5)= cov65;
        p7b.cov(6, 6)= cov66;
        p7b.mean[0] =  pose7b[i][0];
        p7b.mean[1] =  pose7b[i][1];
        p7b.mean[2] =  pose7b[i][2];
        p7b.mean[3] =  pose7b[i][3];
        p7b.mean[4] =  pose7b[i][4];
        p7b.mean[5] =  pose7b[i][5];
        p7b.mean[6] =  pose7b[i][6];
        p7 = p7a + p7b;
        CPose3DPDFGaussian p6 = CPose3DPDFGaussian(p7);


        string filename = "/home/songming/cov_ypr.txt";
        ofstream f0;
        f0.open(filename.c_str(),ios::app);
        f0 << fixed;
        f0 << setprecision(9) << p6.cov(0, 0) << " " << p6.cov(1, 1)  << " " << p6.cov(2, 2) << " "  << p6.cov(3, 3) << " " <<
              p6.cov(4, 4) << " "  << p6.cov(5, 5) << std::endl;
        f0.close();
        vec_p6.push_back(p6);
        i = i + 1;
    }

    /**
     * Construct the CPointPDFGaussian instances
     * initialize and pass a mean to the p1, p2 instances and a covariance
     * matrix. The latter is done by explicitly adding the cov. matrix elemnts
     */
    /*CPointPDFGaussian p1, p2, p;
    p1.mean = CPoint3D(0, -0.12, 0);
    p2.mean = CPoint3D(0, -0.1, 0);

    p1.cov.setZero();
    p1.cov(0, 0) = 0.06f;
    p1.cov(0, 1) = 0.002f;
    p1.cov(1, 0) = 0.002f;
    p1.cov(1, 1) = 0.02f;
    p1.cov(2, 2) = 0.0002f;

    p2.cov.setZero();
    p2.cov(0, 0) = 0.02f;
    p2.cov(0, 1) = -0.004f;
    p2.cov(1, 0) = -0.004f;
    p2.cov(1, 1) = 0.01f;
    p2.cov(2, 2) = 0.0002f;

    // Integral of Product of gaussians:
    CTicTac tictac;
    double v;

    tictac.Tic();
    for (int i = 0; i < 10000; i++) v = p1.productIntegralWith(p2);

    printf("Time for computing: %.04fus\n", tictac.Tac() * 1e+6f / 10000);

    printf("product p1,p2 -> %f\n", v);
    printf("product p2,p1 -> %f\n", p2.productIntegralNormalizedWith(p1));

    // Bayesian fusion:
    p.bayesianFusion(p1, p2);
    p.saveToTextFile("BayesFusion.txt");

    cout << "Bayesian fusing of p1 & p2: " << endl;
    cout << " MEAN: " << p.mean << endl <<  " COV:" << endl << p.cov << endl;*/
}

// ------------------------------------------------------
//				TestPoseComposition
// ------------------------------------------------------
void TestPoseComposition()
{
    CTicTac tictac;

    // ---------------------------------------------------------------
    /**
     * CPose3D default constructor method takes the arguments in the
     * (X, Y, Z, YAW=0, PITCH=0, ROLL=0) format. The angles are optional
     */
    CPose3D A(0, 0, 0), B(1, 1, 0, 45.0_deg, 0, 0), C;

    C = A - B;

    cout << "A:\n" << A << endl;
    cout << "B:\n" << B << endl;
    cout << "C:\n" << C << endl;

    // ---------------------------------------------------------------
    CPose2D p(0.5f, 0.2f, DEG2RAD(10.0f));

    // stores a sequence of relative, incremental 2D poses
    CPoses2DSequence seq;

    CPose2D a(1, 2, (float)0.0_deg);
    CPose2D b(2, 3, (float)45.0_deg);
    CPose2D D;

    CPose2D x(1, 0, (float)0.0_deg);
    CPose2D y(1, 0, (float)45.0_deg);

    cout << "a= " << a << endl;
    cout << "b= " << b << endl;

    // ------------------------------------------
    tictac.Tic();
    D = a + b;
    printf("%f us\t", tictac.Tac() * 1e6);
    cout << "a+b= " << D << endl;
    // ------------------------------------------
    tictac.Tic();
    D = b - a;
    printf("%f us\t", tictac.Tac() * 1e6);
    cout << "b-a= " << D << endl;
    // ------------------------------------------
    tictac.Tic();
    D = a + (b - a);
    printf("%f us\t", tictac.Tac() * 1e6);
    cout << "a + (b-a)= " << D << endl;
    // ------------------------------------------

    /**
     * Incrementally update the pose of the "robot".
     * Appending the pose is equivalent to a position/rotation change with
     * regards to the body-fixed frame of reference
     * For more information refer to:
     * https://reference.mrpt.org/stable/_c_poses2_d_sequence_8h_source.html
     */
    seq.appendPose(y);
    cout << "last= " << seq.absolutePoseAfterAll() << endl;
    seq.appendPose(y);
    cout << "last= " << seq.absolutePoseAfterAll() << endl;
    seq.appendPose(x);
    cout << "last= " << seq.absolutePoseAfterAll() << endl;

    // play the poses from the beginning using the getPose method
    seq.getPose(0, D);
    cout << "Pose[0] in seq.= " << D << endl;
    seq.getPose(1, D);
    cout << "Pose[1] in seq.= " << D << endl;
    seq.getPose(2, D);
    cout << "Pose[2] in seq.= " << D << endl;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{

    string posepath = "/home/songming/lidarrelodometry.txt";
    string covpath = "/home/songming/lodometry_cov.txt";

    try
    {
        //TestPoseComposition();
        TestPosePDFOperations(posepath, covpath);



        return 0;
    }
    catch (exception& e)
    {
        cout << "MRPT exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Untyped exception!!");
        return -1;
    }
}
