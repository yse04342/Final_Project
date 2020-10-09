//by Junsu Lee, Mechatronics Engeering, Korea Polytechnic University, 


//******************************************
//header file
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <wiringPi.h>
//******************************************



//******************************************
//namespace
using namespace cv;
using namespace std;
//*******************************************



//여기부터 메인문
int main(int, char**)
{
    
    int val; //하얀픽셀의 갯수 
    int value = 0; //카메라를 활성화시킬지 결정하는 변수
    int count = 0; //count를 두번 읽어야 인터럽트 발생
    

    wiringPiSetup(); //와이어링파이를 사용하기 위한 함수
    pinMode(1, OUTPUT); //1번핀 설정
    pinMode(4, INPUT); //4번핀 설정 
    digitalWrite(1, LOW); //처음에 1번핀 LOW로 설정


    //Matrix declare 
    Mat original; //원본영상 매트릭스  
    Mat smoothing; //스무싱 매트릭스
    Mat erode; // 침식 매트릭스
    Mat result; //결과 사진 매트릭스  
   
    
    

    Ptr<BackgroundSubtractor> pMOG2; //이 두개 함수는 차영상을 만들기 위한 함수 
    pMOG2 = createBackgroundSubtractorMOG2();
    

////////////////////////////////////////////////이 부분 영상처리에서 그냥 당연하게 입력해주는 부분 
VideoCapture cap; //동영상을 만들기 위한 비디오캡쳐
    
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
////////////////////////////////////////////////////////    
    

cap.open(deviceID + apiID); //번호에 맞는 카메라에 접근 
   
    
    if (!cap.isOpened()) //카메라가 안켜지면 아래문구 뛰우고 종료 
    {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }


    cout << "Start grabbing" << endl << "Press any key to terminate" << endl; //카메카 켜졌다는 출력
        
        
    while(1)
    {
       value = digitalRead(4);  //while문 시작했을 때부터 4번핀 값을 읽어온다.
        
           
            // wait for a new frame from camera and store it into 'frame'
            cap.read(original); //원본매트릭스 영상을 읽어온다.
            resize(original, original, Size(300,300), 0, 0, 0); //영상사이즈를 가로세로 300으로 바꿔줌
            GaussianBlur(original, smoothing, Size(5,5), 3, 3); //블러처리를 해주는 함수 
            
            pMOG2->apply(smoothing, result); //스무싱영상을 결과영상에 적용
            Mat element5(5, 5, CV_8U, Scalar(1));
            Mat opened; //열림연산 매트릭스 
            morphologyEx(result, opened, MORPH_OPEN, element5); //결과매트릭스를 오픈매트릭스에 적용
            // check if we succeeded

            if (original.empty()) //원본영상이 안켜지면 종료 
            {
            cerr << "ERROR! blank frame grabbed\n";
            break;
            }
            
       
            imshow("original", original); //원본영상 보여줌 
            imshow("result", result); //결과영상 보여줌 
            imshow("open", opened); //열린영상 보여줌 

            //imshow("integration", integration);
            
            for(int r = 0; r < opened.rows; r++) //가로픽셀수 
            {
                for(int c = 0; c < opened.cols; c++) //세로픽셀수 
                {
                    if(opened.at<uchar>(r, c) > 240) //각 픽셀의 값이 240이상이면 val++
                    {
                        val++;
                    }
                        
                }
                    
            }
            
            
            cout << val << endl; //val값을 출력 
            if(val >7000)  //val값이 7000이상이면 
            {
                cout << "--------val----------" << endl; // val이라고 띄우고 
                
                count++; //count++해줌 
            }
             
            if(count ==2) //count가 2되면 
            {
                if(value)
                {
                digitalWrite(1, HIGH); //인터럽트를 보내준다. 
                            digitalWrite(1, LOW);

                cout << "*********HIGH*********" << endl; //인터럽트가 보내졌다고 출력해준다.
                cout << "*********HIGH*********" << endl;
                cout << "*********HIGH*********" << endl;


                }
                count = 0; //그리고 count를 초기화해준다. 
            }
            
            val = 0;  // val도 초기화


            if (waitKey(5) >= 0) break; //이거는 그냥 영상처리에서 넣어주는거 크게 신경안써도됨
        
        
        else val = 0; 
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
