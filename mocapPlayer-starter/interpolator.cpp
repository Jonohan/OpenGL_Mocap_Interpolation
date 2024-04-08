#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"
#include "performanceCounter.h"
#include <thread>
#include <cstdlib>
#include <ctime>
#include <algorithm>

PerformanceCounter perfCounter;
int newN;

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
    //LinearInterpolationEulerRandom(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
    //LinearInterpolationQuaternionRandom(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
    //BezierInterpolationEulerRandom(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
    //BezierInterpolationQuaternionRandom(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

// when input is N, the range of random number is in [1,N]
int Interpolator::randomNew(int N) 
{
    newN = rand() % N + 1;
    return newN;
}


void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  // start timer
  perfCounter.StartCounter();

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  //stop timer
  perfCounter.StopCounter();
  double elaTime = perfCounter.GetElapsedTime();
  printf("Linear Interpolation Euler time: %f\n", elaTime);

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

}

void Interpolator::LinearInterpolationEulerRandom(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;

    // initialize ramdom number generator
    srand((unsigned)time(0));

    // start timer
    perfCounter.StartCounter();

    while (startKeyframe + randomNew(N) + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + newN + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= newN; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (newN + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
                interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    //stop timer
    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Linear Interpolation Euler time: %f\n", elaTime);

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
    // Rotation matrix
    double RX[4][4], RY[4][4], RZ[4][4];

    rotationX(RX, angles[0]);
    rotationY(RY, angles[1]);
    rotationZ(RZ, angles[2]);

    // Calculate rotation matrix: R = Z * Y * X
    double TempZY[4][4], TempZYX[4][4];
    matrix_mult(RZ, RY, TempZY);
    matrix_mult(TempZY, RX, TempZYX);

    //Only need rotation (3*3), store into array
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++) 
        {
            R[i * 3 + j] = TempZYX[i][j];
        }
    }
}

// N = num of dropping consecutive frames,
void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this    
    int numFrame = pInputMotion->GetNumFrames();

    int startKeyFrame = 0;
    vector p0, p1, p2, p3;
    vector aTemp1, a1, aTemp2, b2;

    perfCounter.StartCounter();
    while (startKeyFrame + N + 1 < numFrame)
    {
        int prevKeyFrame = startKeyFrame - N - 1;
        int endKeyFrame = startKeyFrame + N + 1;
        int nextKeyFrame = endKeyFrame + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyFrame);
        Posture* endPosture = pInputMotion->GetPosture(endKeyFrame);
        Posture* nextPosture;
        Posture* prevPosture;

        pOutputMotion->SetPosture(startKeyFrame, *startPosture);
        pOutputMotion->SetPosture(endKeyFrame, *endPosture);

        Posture inBetPosture;

        // interpolate the frame in between key frames, start at frame 1
        for (int inBetFrame = 1; inBetFrame <= N; inBetFrame++)
        {
            p1 = startPosture->root_pos;
            p2 = endPosture->root_pos;

            double t; // for DeCasteljau interpolation
            t = 1.0 * inBetFrame / (N + 1); // current frame/ total frame between each key frame

            if (startKeyFrame == 0) // first frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                p3 = nextPosture->root_pos;

                // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                aTemp1 = LinearInt(2.0, p3, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt(-1.0 / 3.0, p2, aTemp2);
            }
            else if (nextKeyFrame > numFrame - 1) // last frame, -1 since Keyframe starts at 0
            {
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;

                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                aTemp2 = LinearInt(2.0, p0, p1);
                b2 = LinearInt(1.0 / 3.0, p2, aTemp2);

            }
            else // in between frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;
                p3 = nextPosture->root_pos;
                
                // get control point a1 from first 3 points: p0, p1, p2
                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                // get control point b2 from next 3 points:  p1, p2, p3
                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt(-1.0 / 3.0, p2, aTemp2);
            }
            // Spline 1: q1, a1, b2, q2
            // Spline 2: q2, a2, b3, q3 etc.
            inBetPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

            // in every frame, interpolate all the bones' movement
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                p1 = startPosture->bone_rotation[bone];
                p2 = endPosture->bone_rotation[bone];

                if (startKeyFrame == 0) // first bone
                {
                    nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                    p3 = nextPosture->bone_rotation[bone];

                    // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                    aTemp1 = LinearInt(2.0, p3, p2);
                    a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                    aTemp2 = LinearInt(2.0, p1, p2);
                    aTemp2 = LinearInt(0.5, aTemp2, p3);
                    b2 = LinearInt((- 1.0) / 3.0, p2, aTemp2);
                }
                else if (nextKeyFrame > numFrame - 1) // last bone
                {
                    prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                    p0 = prevPosture->bone_rotation[bone];

                    aTemp1 = LinearInt(2.0, p0, p1);
                    aTemp1 = LinearInt(0.5, aTemp1, p2);
                    a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                    //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                    aTemp2 = LinearInt(2.0, p0, p1);
                    b2 = LinearInt(1.0 / 3.0, p2, aTemp2);

                }
                else
                {
                    nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                    prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                    p0 = prevPosture->bone_rotation[bone];
                    p3 = nextPosture->bone_rotation[bone];

                    // get control point a1 from first 3 points: p0, p1, p2
                    aTemp1 = LinearInt(2.0, p0, p1);
                    aTemp1 = LinearInt(0.5, aTemp1, p2);
                    a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                    // get control point b2 from next 3 points:  p1, p2, p3
                    aTemp2 = LinearInt(2.0, p1, p2);
                    aTemp2 = LinearInt(0.5, aTemp2, p3);
                    b2 = LinearInt((- 1.0) / 3.0, p2, aTemp2);
                }
                inBetPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a1, b2, p2);
            }
            // interpolate in between frame
            pOutputMotion->SetPosture(startKeyFrame + inBetFrame, inBetPosture);
        }
        // Next
        startKeyFrame = endKeyFrame;
    }

    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Bezier Interpolation Euler time: %f\n", elaTime);

    // add rest of frames
    for (int frame = startKeyFrame + 1; frame < numFrame; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::BezierInterpolationEulerRandom(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this    
    int numFrame = pInputMotion->GetNumFrames();

    int startKeyFrame = 0;
    vector p0, p1, p2, p3;
    vector aTemp1, a1, aTemp2, b2;

    srand((unsigned)time(0));

    perfCounter.StartCounter();
    while (startKeyFrame + randomNew(N) + 1 < numFrame)
    {
        int prevKeyFrame = max(0, startKeyFrame - newN - 1); // avoid negetive when run random value
        int endKeyFrame = startKeyFrame + newN + 1;
        int nextKeyFrame = endKeyFrame + newN + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyFrame);
        Posture* endPosture = pInputMotion->GetPosture(endKeyFrame);
        Posture* nextPosture;
        Posture* prevPosture;

        pOutputMotion->SetPosture(startKeyFrame, *startPosture);
        pOutputMotion->SetPosture(endKeyFrame, *endPosture);

        Posture inBetPosture;

        // interpolate the frame in between key frames, start at frame 1
        for (int inBetFrame = 1; inBetFrame <= newN; inBetFrame++)
        {
            p1 = startPosture->root_pos;
            p2 = endPosture->root_pos;

            double t; // for DeCasteljau interpolation
            t = 1.0 * inBetFrame / (newN + 1); // current frame/ total frame between each key frame

            if (startKeyFrame == 0) // first frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                p3 = nextPosture->root_pos;

                // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                aTemp1 = LinearInt(2.0, p3, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt(-1.0 / 3.0, p2, aTemp2);
            }
            else if (nextKeyFrame > numFrame - 1) // last frame, -1 since Keyframe starts at 0
            {
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;

                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                aTemp2 = LinearInt(2.0, p0, p1);
                b2 = LinearInt(1.0 / 3.0, p2, aTemp2);

            }
            else // in between frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;
                p3 = nextPosture->root_pos;

                // get control point a1 from first 3 points: p0, p1, p2
                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                // get control point b2 from next 3 points:  p1, p2, p3
                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt(-1.0 / 3.0, p2, aTemp2);
            }
            // Spline 1: q1, a1, b2, q2
            // Spline 2: q2, a2, b3, q3 etc.
            inBetPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

            // in every frame, interpolate all the bones' movement
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                p1 = startPosture->bone_rotation[bone];
                p2 = endPosture->bone_rotation[bone];

                if (startKeyFrame == 0) // first bone
                {
                    nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                    p3 = nextPosture->bone_rotation[bone];

                    // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                    aTemp1 = LinearInt(2.0, p3, p2);
                    a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                    aTemp2 = LinearInt(2.0, p1, p2);
                    aTemp2 = LinearInt(0.5, aTemp2, p3);
                    b2 = LinearInt((-1.0) / 3.0, p2, aTemp2);
                }
                else if (nextKeyFrame > numFrame - 1) // last bone
                {
                    prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                    p0 = prevPosture->bone_rotation[bone];

                    aTemp1 = LinearInt(2.0, p0, p1);
                    aTemp1 = LinearInt(0.5, aTemp1, p2);
                    a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                    //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                    aTemp2 = LinearInt(2.0, p0, p1);
                    b2 = LinearInt(1.0 / 3.0, p2, aTemp2);

                }
                else
                {
                    nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                    prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                    p0 = prevPosture->bone_rotation[bone];
                    p3 = nextPosture->bone_rotation[bone];

                    // get control point a1 from first 3 points: p0, p1, p2
                    aTemp1 = LinearInt(2.0, p0, p1);
                    aTemp1 = LinearInt(0.5, aTemp1, p2);
                    a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                    // get control point b2 from next 3 points:  p1, p2, p3
                    aTemp2 = LinearInt(2.0, p1, p2);
                    aTemp2 = LinearInt(0.5, aTemp2, p3);
                    b2 = LinearInt((-1.0) / 3.0, p2, aTemp2);
                }
                inBetPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a1, b2, p2);
            }
            // interpolate in between frame
            pOutputMotion->SetPosture(startKeyFrame + inBetFrame, inBetPosture);
        }
        // Next
        startKeyFrame = endKeyFrame;
    }

    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Bezier Interpolation Euler time: %f\n", elaTime);

    // add rest of frames
    for (int frame = startKeyFrame + 1; frame < numFrame; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    int inputLength = pInputMotion->GetNumFrames();

    int startKeyframe = 0;

    perfCounter.StartCounter();
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int inBetframe = 1; inBetframe <= N; inBetframe++)
        {
            Posture inBetPosture;
            double t = 1.0 * inBetframe / (N + 1);

            // with quaternion interpolation, use Euler interpolation to interpolate the root position
            inBetPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {   
                double startAngle[3], endAngle[3], inBetAngle[3];
                Quaternion<double> startQua, endQua, inBetQua;

                // Get euler angle of start and end posture
                startPosture->bone_rotation[bone].getValue(startAngle);
                endPosture->bone_rotation[bone].getValue(endAngle);

                // euler to quaternion
                Euler2Quaternion(startAngle, startQua);
                Euler2Quaternion(endAngle, endQua);
                
                // slerp for in between
                inBetQua = Slerp(t, startQua, endQua);

                // back to euler
                Quaternion2Euler(inBetQua, inBetAngle);

                inBetPosture.bone_rotation[bone].setValue(inBetAngle);
            }
            pOutputMotion->SetPosture(startKeyframe + inBetframe, inBetPosture);
        }

        startKeyframe = endKeyframe;
    }

    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Linear Interpolation Quaternion time: %f\n", elaTime);

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::LinearInterpolationQuaternionRandom(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames();

    int startKeyframe = 0;

    perfCounter.StartCounter();
    while (startKeyframe + randomNew(N) + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + newN + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int inBetframe = 1; inBetframe <= newN; inBetframe++)
        {
            Posture inBetPosture;
            double t = 1.0 * inBetframe / (newN + 1);

            // with quaternion interpolation, use Euler interpolation to interpolate the root position
            inBetPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double startAngle[3], endAngle[3], inBetAngle[3];
                Quaternion<double> startQua, endQua, inBetQua;

                // Get euler angle of start and end posture
                startPosture->bone_rotation[bone].getValue(startAngle);
                endPosture->bone_rotation[bone].getValue(endAngle);

                // euler to quaternion
                Euler2Quaternion(startAngle, startQua);
                Euler2Quaternion(endAngle, endQua);

                // slerp for in between
                inBetQua = Slerp(t, startQua, endQua);

                // back to euler
                Quaternion2Euler(inBetQua, inBetAngle);

                inBetPosture.bone_rotation[bone].setValue(inBetAngle);
            }
            pOutputMotion->SetPosture(startKeyframe + inBetframe, inBetPosture);
        }

        startKeyframe = endKeyframe;
    }

    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Linear Interpolation Quaternion time: %f\n", elaTime);

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
  // students should implement this
    int numFrame = pInputMotion->GetNumFrames();

    int startKeyFrame = 0;
    vector p0, p1, p2, p3;
    vector aTemp1, a1, aTemp2, a2, b2;

    perfCounter.StartCounter();
    while (startKeyFrame + N + 1 < numFrame)
    {
        int prevKeyFrame = startKeyFrame - N - 1;
        int endKeyFrame = startKeyFrame + N + 1;
        int nextKeyFrame = endKeyFrame + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyFrame);
        Posture* endPosture = pInputMotion->GetPosture(endKeyFrame);
        Posture* nextPosture;
        Posture* prevPosture;

        pOutputMotion->SetPosture(startKeyFrame, *startPosture);
        pOutputMotion->SetPosture(endKeyFrame, *endPosture);

        Posture inBetPosture;

        // interpolate the frame in between key frames, start at frame 1
        for (int inBetFrame = 1; inBetFrame <= N; inBetFrame++)
        {
            p1 = startPosture->root_pos;
            p2 = endPosture->root_pos;

            double t; // for DeCasteljau interpolation
            t = 1.0 * inBetFrame / (N + 1); // current frame/ total frame between each key frame

            // roots are euler
            if (startKeyFrame == 0) // first frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                p3 = nextPosture->root_pos;

                // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                aTemp1 = LinearInt(2.0, p3, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt((- 1.0) / 3.0, p2, aTemp2);
            }
            else if (nextKeyFrame > numFrame - 1) // last frame, -1 since Keyframe starts at 0
            {
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;

                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                aTemp2 = LinearInt(2.0, p0, p1);
                b2 = LinearInt(1.0 / 3.0, p2, aTemp2);

            }
            else // in between frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;
                p3 = nextPosture->root_pos;

                // get control point a1 from first 3 points: p0, p1, p2
                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                // get control point b2 from next 3 points:  p1, p2, p3
                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt((- 1.0) / 3.0, p2, aTemp2);
            }
            // Spline 1: q1, a1, b2, q2
            // Spline 2: q2, a2, b3, q3 etc.
            inBetPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

            // in every frame, interpolate all the bones' movement
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double prevAngle[3], startAngle[3], endAngle[3], nextAngle[3], inBetAngle[3];
                Quaternion<double> q0, q1, q2, q3, qa1, qb2, qaTemp1, qaTemp2, inBetQua;

                startPosture->bone_rotation[bone].getValue(startAngle);
                endPosture->bone_rotation[bone].getValue(endAngle);
                Euler2Quaternion(startAngle, q1);
                Euler2Quaternion(endAngle, q2);

                if (startKeyFrame == 0) // first bone
                {
                    nextPosture->bone_rotation[bone].getValue(nextAngle);
                    Euler2Quaternion(nextAngle, q3);

                    // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                    qaTemp1 = Slerp(2.0, q3, q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qaTemp1);

                    qaTemp2 = Slerp(2.0, q1, q2);
                    qaTemp2 = Slerp(0.5, qaTemp2, q3);
                    qb2 = Slerp((-1.0) / 3.0, q2, qaTemp2);
                }
                else if (nextKeyFrame > numFrame - 1) // last bone
                {
                    prevPosture->bone_rotation[bone].getValue(prevAngle);
                    Euler2Quaternion(prevAngle, q0);

                    qaTemp1 = Slerp(2.0, q0, q1);
                    qaTemp1 = Slerp(0.5, qaTemp1, q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qaTemp1);

                    //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                    qaTemp2 = Slerp(2.0, q0, q1);
                    qb2 = Slerp(1.0 / 3.0, q2, qaTemp2);
                }
                else
                {
                    nextPosture->bone_rotation[bone].getValue(nextAngle);
                    prevPosture->bone_rotation[bone].getValue(prevAngle);
                    Euler2Quaternion(prevAngle, q0);
                    Euler2Quaternion(nextAngle, q3);

                    // get control point a1 from first 3 points: q0, q1, q2
                    qaTemp1 = Slerp(2.0, q0, q1);
                    qaTemp1 = Slerp(0.5, qaTemp1, q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qaTemp1);

                    // get control point b2 from next 3 points:  q1, q2, q3
                    qaTemp2 = Slerp(2.0, q1, q2);
                    qaTemp2 = Slerp(0.5, qaTemp2, q3);
                    qb2 = Slerp((-1.0) / 3.0, q2, qaTemp2);
                }
                inBetQua = DeCasteljauQuaternion(t, q1, qa1, qb2, q2);

                Quaternion2Euler(inBetQua, inBetAngle);
                inBetPosture.bone_rotation[bone].setValue(inBetAngle);
            }
            // interpolate in between frame
            pOutputMotion->SetPosture(startKeyFrame + inBetFrame, inBetPosture);
        }
        // Next
        startKeyFrame = endKeyFrame;
    }

    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Bezier Interpolation Quaternion time: %f\n", elaTime);

    // add rest of frames
    for (int frame = startKeyFrame + 1; frame < numFrame; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::BezierInterpolationQuaternionRandom(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
    int numFrame = pInputMotion->GetNumFrames();

    int startKeyFrame = 0;
    vector p0, p1, p2, p3;
    vector aTemp1, a1, aTemp2, a2, b2;

    perfCounter.StartCounter();
    while (startKeyFrame + randomNew(N) + 1 < numFrame)
    {
        int prevKeyFrame = max(0, startKeyFrame - newN - 1); // avoid negetive when run random value
        int endKeyFrame = startKeyFrame + newN + 1;
        int nextKeyFrame = endKeyFrame + newN + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyFrame);
        Posture* endPosture = pInputMotion->GetPosture(endKeyFrame);
        Posture* nextPosture;
        Posture* prevPosture;

        pOutputMotion->SetPosture(startKeyFrame, *startPosture);
        pOutputMotion->SetPosture(endKeyFrame, *endPosture);

        Posture inBetPosture;

        // interpolate the frame in between key frames, start at frame 1
        for (int inBetFrame = 1; inBetFrame <= newN; inBetFrame++)
        {
            p1 = startPosture->root_pos;
            p2 = endPosture->root_pos;

            double t; // for DeCasteljau interpolation
            t = 1.0 * inBetFrame / (newN + 1); // current frame/ total frame between each key frame

            // roots are euler
            if (startKeyFrame == 0) // first frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                p3 = nextPosture->root_pos;

                // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                aTemp1 = LinearInt(2.0, p3, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt((-1.0) / 3.0, p2, aTemp2);
            }
            else if (nextKeyFrame > numFrame - 1) // last frame, -1 since Keyframe starts at 0
            {
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;

                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                aTemp2 = LinearInt(2.0, p0, p1);
                b2 = LinearInt(1.0 / 3.0, p2, aTemp2);

            }
            else // in between frame
            {
                nextPosture = pInputMotion->GetPosture(nextKeyFrame);
                prevPosture = pInputMotion->GetPosture(prevKeyFrame);
                p0 = prevPosture->root_pos;
                p3 = nextPosture->root_pos;

                // get control point a1 from first 3 points: p0, p1, p2
                aTemp1 = LinearInt(2.0, p0, p1);
                aTemp1 = LinearInt(0.5, aTemp1, p2);
                a1 = LinearInt(1.0 / 3.0, p1, aTemp1);

                // get control point b2 from next 3 points:  p1, p2, p3
                aTemp2 = LinearInt(2.0, p1, p2);
                aTemp2 = LinearInt(0.5, aTemp2, p3);
                b2 = LinearInt((-1.0) / 3.0, p2, aTemp2);
            }
            // Spline 1: q1, a1, b2, q2
            // Spline 2: q2, a2, b3, q3 etc.
            inBetPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

            // in every frame, interpolate all the bones' movement
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double prevAngle[3], startAngle[3], endAngle[3], nextAngle[3], inBetAngle[3];
                Quaternion<double> q0, q1, q2, q3, qa1, qb2, qaTemp1, qaTemp2, inBetQua;

                startPosture->bone_rotation[bone].getValue(startAngle);
                endPosture->bone_rotation[bone].getValue(endAngle);
                Euler2Quaternion(startAngle, q1);
                Euler2Quaternion(endAngle, q2);

                if (startKeyFrame == 0) // first bone
                {
                    nextPosture->bone_rotation[bone].getValue(nextAngle);
                    Euler2Quaternion(nextAngle, q3);

                    // like a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3)
                    qaTemp1 = Slerp(2.0, q3, q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qaTemp1);

                    qaTemp2 = Slerp(2.0, q1, q2);
                    qaTemp2 = Slerp(0.5, qaTemp2, q3);
                    qb2 = Slerp((-1.0) / 3.0, q2, qaTemp2);
                }
                else if (nextKeyFrame > numFrame - 1) // last bone
                {
                    prevPosture->bone_rotation[bone].getValue(prevAngle);
                    Euler2Quaternion(prevAngle, q0);

                    qaTemp1 = Slerp(2.0, q0, q1);
                    qaTemp1 = Slerp(0.5, qaTemp1, q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qaTemp1);

                    //like bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3)
                    qaTemp2 = Slerp(2.0, q0, q1);
                    qb2 = Slerp(1.0 / 3.0, q2, qaTemp2);
                }
                else
                {
                    nextPosture->bone_rotation[bone].getValue(nextAngle);
                    prevPosture->bone_rotation[bone].getValue(prevAngle);
                    Euler2Quaternion(prevAngle, q0);
                    Euler2Quaternion(nextAngle, q3);

                    // get control point a1 from first 3 points: q0, q1, q2
                    qaTemp1 = Slerp(2.0, q0, q1);
                    qaTemp1 = Slerp(0.5, qaTemp1, q2);
                    qa1 = Slerp(1.0 / 3.0, q1, qaTemp1);

                    // get control point b2 from next 3 points:  q1, q2, q3
                    qaTemp2 = Slerp(2.0, q1, q2);
                    qaTemp2 = Slerp(0.5, qaTemp2, q3);
                    qb2 = Slerp((-1.0) / 3.0, q2, qaTemp2);
                }
                inBetQua = DeCasteljauQuaternion(t, q1, qa1, qb2, q2);

                Quaternion2Euler(inBetQua, inBetAngle);
                inBetPosture.bone_rotation[bone].setValue(inBetAngle);
            }
            // interpolate in between frame
            pOutputMotion->SetPosture(startKeyFrame + inBetFrame, inBetPosture);
        }
        // Next
        startKeyFrame = endKeyFrame;
    }

    perfCounter.StopCounter();
    double elaTime = perfCounter.GetElapsedTime();
    printf("Bezier Interpolation Quaternion time: %f\n", elaTime);

    // add rest of frames
    for (int frame = startKeyFrame + 1; frame < numFrame; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
    double R[9];// Rotation matrix
    Euler2Rotation(angles, R); // euler angle to rotation matrix
    q = Quaternion<double>::Matrix2Quaternion(R);
    // We use unit quaternion for rotation
    q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
    double R[9];
    q.Quaternion2Matrix(R);
    Rotation2Euler(R, angles); // rotation matrix to euler angle
}


//Linear Interpolation: start * (1 - t) + end * t
vector LinearInt(double t, vector& start, vector& end)
{
    vector result = start * (1 - t) + end * t;
    return result;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double>& qStart, Quaternion<double>& qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  // costTheta = q1 * q2 = s1s2 + x1x2 +y1y2 + z1z2, Theta is angle between q1, q2
  double cosTheta = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();
  double sinTheta;
  double Theta;

  if (cosTheta >= 0)
  {
      // return radian
      Theta = acos(cosTheta);
  }
  else//>90 degree, go from other side
  {
      Theta = acos(-cosTheta);
      qEnd_ = (- 1.0) * qEnd_;
  }
  
  // avoid dividing by very small number
  if (Theta < 0.0001)
  { 
      result = qStart;
  }
  else
  {
      sinTheta = sin(Theta);// input is radian, not angle
      result = (sin((1 - t) * Theta) / sinTheta) * qStart + (sin(t * Theta) / sinTheta) * qEnd_;
  }
  result.Normalize();
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  result = Slerp(2.0, p, q);
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  // add lost frames
  vector Q0, Q1, Q2;
  vector R0, R1;

  Q0 = p0 * (1 - t) + p1 * t;
  Q1 = p1 * (1 - t) + p2 * t;
  Q2 = p2 * (1 - t) + p3 * t;
  R0 = Q0 * (1 - t) + Q1 * t;
  R1 = Q1 * (1 - t) + Q2 * t;

  result = R0 * (1 - t) + R1 * t;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> Q0, Q1, Q2;
  Quaternion<double> R0, R1;

  Q0 = Slerp(t, p0, p1);
  Q1 = Slerp(t, p1, p2);
  Q2 = Slerp(t, p2, p3);
  R0 = Slerp(t, Q0, Q1);
  R1 = Slerp(t, Q1, Q2);

  result = Slerp(t, R0, R1);
  return result;
}

